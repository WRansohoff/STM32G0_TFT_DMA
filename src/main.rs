#![no_std]
#![no_main]

// Halt when the program panics.
extern crate panic_halt;

// Core library includes.
use core::ptr;
// Generic ARM Cortex-M includes.
use cortex_m::peripheral::syst::SystClkSource;
use cortex_m_rt::entry;
// Chip-specific PAC include.
use stm32_pac::stm32g0x1 as stm32;

// System core clock speed.
static mut SYS_CLK_HZ : u32 = 16_000_000;

// Framebuffer for the TFT display.
const ILI9163C_W  : u8 = 128;
const ILI9163C_H  : u8 = 128;
const ILI9163C_A  : usize = ( ILI9163C_W as usize * ILI9163C_H as usize );
static mut ILI9163C_FB : [ u16; ( ILI9163C_A ) ] = [ 0x0000; ILI9163C_A ];

// Note: For the STM32G0 SPI peripheral, we need to use
// `ptr::write_volatile` for single-byte writes.
// `spi.dr.write(...)` sends the register's full 16 bits.
fn spi_w8( spi: &stm32::SPI1, byte: u8 ) -> () {
  while spi.sr.read().txe().bit_is_clear() {};
  unsafe { ptr::write_volatile( &spi.dr as *const _ as *mut u8, byte ) };
}

// 16-bit writes can use the normal write method, but be aware
// that the order the bytes send is 0x2211.
fn spi_w16( spi: &stm32::SPI1, bytes: u16 ) -> () {
  while spi.sr.read().txe().bit_is_clear() {};
  unsafe { spi.dr.write( |w| w.dr().bits( bytes ) ) };
}

#[entry]
fn main() -> ! {
  // Checkout ARM Cortex-M peripheral singleton..
  let cm_p = cortex_m::Peripherals::take().unwrap();
  let mut syst = cm_p.SYST;
  // Checkout STM32 peripheral singleton.
  let p = stm32::Peripherals::take().unwrap();
  let rcc = p.RCC;
  let dma = p.DMA;
  let dmamux = p.DMAMUX;
  let flash = p.FLASH;
  let spi = p.SPI1;
  let tim3 = p.TIM3;
  let gpioa = p.GPIOA;
  let gpiob = p.GPIOB;

  // (Optional) Initialize system clock to 64MHz.
  // Use the default HSI16 osc to drive the PLL.
  unsafe {
    // Set 2 Flash wait states.
    flash.acr.modify( |_r,w| w.latency().bits( 0b010 ) );
    // Configure PLL settings.
    // freq = ( 16MHz * ( N / M ) ) / R
    // For 64MHz, R = 2, M = 1, N = 8.
    rcc.pllsyscfgr.modify( |_r,w| w.pllr().bits( 0b001 )
                                   .pllren().set_bit()
                                   .plln().bits( 0b0001000 )
                                   .pllm().bits( 0b000 )
                                   .pllsrc().bits( 0b10 ) );
    // Enable and select PLL.
    rcc.cr.modify( |_r,w| w.pllon().set_bit() );
    while !rcc.cr.read().pllrdy().bit_is_set() {};
    rcc.cfgr.modify( |_r,w| w.sw().bits( 0b010 ) );
    while rcc.cfgr.read().sws().bits() != 0b010 {};
    // Update system clock value.
    SYS_CLK_HZ = 64_000_000;
  }

  // Enable GPIO ports A and B.
  rcc.iopenr.write( |w| w.iopaen().set_bit()
                         .iopben().set_bit() );
  // Enable TIM3, SPI1, DMA peripherals..
  rcc.apbenr1.write( |w| w.tim3en().set_bit() );
  rcc.apbenr2.write( |w| w.spi1en().set_bit() );
  rcc.ahbenr.write( |w| w.dmaen().set_bit() );

  // Setup pin B4 as alternate function #1 (TIM3_CH1),
  // and B3/5 as alternate function #0 (SPI1)
  unsafe {
    gpiob.moder.modify( |_r,w| w.moder3().bits( 2 )
                                .moder4().bits( 2 )
                                .moder5().bits( 2 ) );
    gpiob.afrl.modify( |_r,w| w.afsel3().bits( 0 )
                               .afsel4().bits( 1 )
                               .afsel5().bits( 0 ) );
  }
  /*
  // TODO: Setup pin A15 as alt. func. #0 (SPI1 CS pin)
  gpioa.moder.modify( |_r,w| w.moder15().alternate() );
  unsafe { gpioa.afrh.modify( |_r,w| w.afsel15().bits( 0 ) );
  */
  // Setup pins A0/1 (A0/Reset) to push-pull output.
  // 'A0' ~= 'Data/Command' from other 4-wire SPIs,
  // and it's a coincidence that I'm using PA0 for that pin.
  // Setup pin A15 as push-pull output for now; software CS.
  unsafe {
    gpioa.moder.modify( |_r,w| w.moder0().bits( 1 )
                                .moder1().bits( 1 )
                                .moder15().bits( 1 ) );
  }
  // Set initial pin states: high for CS/Reset, low for A0.
  gpioa.odr.modify( |_r,w| w.odr0().clear_bit()
                            .odr1().set_bit()
                            .odr15().set_bit() );

  // Setup TIM3 to generate a 50% duty cycle PWM signal
  // to test brightness control.
  unsafe {
    // Prescaler = 0.
    tim3.psc.write( |w| w.psc().bits( 0 ) );
    // Arbitrary auto-reload and capture/compare values.
    // Duty cycle = CCR / ARR.
    tim3.arr.write( |w| w.arr_l().bits( 0x0200 ) );
    tim3.ccr1.write( |w| w.ccr1_l().bits( 0x0100 ) );
    // Enable CCR output and set 'PWM mode 1' (0b0110).
    // (This connects the timer channel to the GPIO pin.)
    tim3.ccer.write( |w| w.cc1e().set_bit() );
    tim3.ccmr1.ccmr1_output.write( |w| w.oc1m().bits( 0b0110 ) );
    // Set the 'Update Generation' bit to apply the settings.
    tim3.egr.write( |w| w.ug().set_bit() );
    // Start the timer.
    tim3.cr1.write( |w| w.cen().set_bit() );
  }

  // Setup DMA channel 0 to send framebuffer data over SPI1.
  unsafe {
    // - Priority level: high (2)
    // - Memory/Peripheral request size: 16-bit (1)
    // - Circular mode: Enabled.
    // - Increment memory ptr, do not increment periph ptr.
    // - Disable 'memory-to-memory' mode.
    // - Set 'memory -> peripheral' transfer direction.
    dma.ccr1.write( |w| w.pl().bits( 0b10 )
                         .msize().bits( 0b01 )
                         .psize().bits( 0b01 )
                         .circ().set_bit()
                         .minc().set_bit()
                         .pinc().clear_bit()
                         .mem2mem().clear_bit()
                         .dir().set_bit() );
    // Route DMA channel 0 to SPI1 transmitter.
    dmamux.dmamux_c0cr.write( |w| w.dmareq_id().bits( 17 ) );
    // Set DMA transfer size and source/destination addresses.
    // TODO: Get register addresses from PAC values.
    let ili9163c_fb_addr : u32 = &ILI9163C_FB as *const [ u16; ILI9163C_A as usize ] as u32;
    dma.cndtr1.write( |w| w.ndt().bits( ILI9163C_A as u16 ) );
    dma.cpar1.write( |w| w.pa().bits( 0x4001300C ) );
    dma.cmar1.write( |w| w.ma().bits( ili9163c_fb_addr ) );

    // Toggle pin A1 to reset the display.
    gpioa.odr.modify( |_r,w| w.odr1().clear_bit() );
    // Setup the SysTick peripheral to trigger every 1ms.
    syst.set_clock_source( SystClkSource::Core );
    syst.set_reload( SYS_CLK_HZ / 1000 );
    // Delay 100ms.
    syst.clear_current();
    syst.enable_counter();
    for _i in 0..100 {
      while !syst.has_wrapped() {};
    }
    // Stop SysTick counter and pull the reset pin high again.
    syst.disable_counter();
    gpioa.odr.modify( |_r,w| w.odr1().set_bit() );

    // Initialize SPI for talking to an ILI9163C display.
    // - Clock phase/polarity: 1/1
    // - Assert internal CS signal (software CS pin control)
    // - MSB-first
    // - 8-bit frames
    // - Baud rate prescaler of 4 (or 128 for debugging)
    // - TX DMA requests enabled.
    spi.cr1.modify( |_r,w| w.ssm().set_bit()
                            .ssi().set_bit()
                            .lsbfirst().clear_bit()
                            .br().bits( 0b001 )
                            //.br().bits( 0b110 )
                            .mstr().set_bit()
                            .cpol().set_bit()
                            .cpha().set_bit() );
    spi.cr2.modify( |_r,w| w.ds().bits( 0b0111 )
                            .txdmaen().set_bit() );
    spi.cr1.modify( |_r,w| w.spe().set_bit() );
    // Pull pin A15 low to enable communication.
    gpioa.odr.modify( |_r,w| w.odr15().clear_bit() );
    // Send initialization commands. This can't use DMA,
    // because we need to toggle the 'A0' pin for some values.
    // Software reset.
    gpioa.odr.modify( |_r,w| w.odr0().clear_bit() );
    spi_w8( &spi, 0x01 );
    // 100ms delay.
    syst.clear_current();
    syst.enable_counter();
    for _i in 0..100 {
      while !syst.has_wrapped() {};
    }
    syst.disable_counter();
    // Display off.
    spi_w8( &spi, 0x28 );
    // Color mode: 16bpp.
    spi_w8( &spi, 0x3A );
    while spi.sr.read().bsy().bit_is_set() {};
    gpioa.odr.modify( |_r,w| w.odr0().set_bit() );
    spi_w8( &spi, 0x55 );
    while spi.sr.read().bsy().bit_is_set() {};
    // Exit sleep mode, delay.
    gpioa.odr.modify( |_r,w| w.odr0().clear_bit() );
    spi_w8( &spi, 0x11 );
    syst.clear_current();
    syst.enable_counter();
    for _i in 0..100 {
      while !syst.has_wrapped() {};
    }
    syst.disable_counter();
    // Display on, delay.
    spi_w8( &spi, 0x29 );
    syst.clear_current();
    syst.enable_counter();
    for _i in 0..100 {
      while !syst.has_wrapped() {};
    }
    syst.disable_counter();
    // The displays I got are offset by a few pixels.
    // So instead of setting X/Y ranges of [0:127]...
    // Column set: [2:129]
    spi_w8( &spi, 0x2A );
    while spi.sr.read().bsy().bit_is_set() {};
    gpioa.odr.modify( |_r,w| w.odr0().set_bit() );
    spi_w16( &spi, 0x0200 );
    spi_w16( &spi, 0x8100 );
    while spi.sr.read().bsy().bit_is_set() {};
    // Row set: [1:128]
    gpioa.odr.modify( |_r,w| w.odr0().clear_bit() );
    spi_w8( &spi, 0x2B );
    while spi.sr.read().bsy().bit_is_set() {};
    gpioa.odr.modify( |_r,w| w.odr0().set_bit() );
    spi_w16( &spi, 0x0100 );
    spi_w16( &spi, 0x8000 );
    while spi.sr.read().bsy().bit_is_set() {};
    // Set 'write to RAM' mode.
    gpioa.odr.modify( |_r,w| w.odr0().clear_bit() );
    spi_w8( &spi, 0x2C );
    while spi.sr.read().bsy().bit_is_set() {};

    // Pull pin A0 high for pixel data.
    gpioa.odr.modify( |_r,w| w.odr0().set_bit() );
    // Enable the DMA channel connected to the framebuffer.
    dma.ccr1.modify( |_r,w| w.en().set_bit() );
  }

  // Main loop.
  let mut val : u16 = 0;
  let mut pos : usize = 0;
  loop {
    // Put a simple incrementing pattern into the framebuffer.
    unsafe { ILI9163C_FB[ pos ] = val };
    if pos < ( ILI9163C_A - 1 ) { pos = pos + 1; }
    else { pos = 0; }
    if val < 65534 { val = val + 1; }
    else { val = 0; }
  }
}
