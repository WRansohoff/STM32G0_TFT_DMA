# Overview

Small example Rust program which demonstrates how to draw to a small SPI TFT display using an STM32G0 microcontroller's DMA peripheral.

The target display is a 128x128-pixel panel with an ILI9163C controller. The code should also work with ILI9341 displays, but the STM32G071 used in this example has 36KB of RAM, which isn't enough for a 16-bit 240x320-pixel framebuffer.

Pin assignments:
* A15 = CS
* B3 = SCK
* B4 = LED (PWM signal for the backlight)
* B5 = SDA
* A0 = A0 (Might also be marked 'D/C' or 'RS')
* A1 = Reset

Sorry that the code is a little messy, I'm still learning about embedded Rust. A lot of the Cargo configuration comes from the embedded Rust working group's 'ARM Cortex-M Quickstart' project.
