MEMORY
{
  FLASH  : ORIGIN = 0x08000000, LENGTH = 128K
  /* Note: If you enable parity check, there's only 32K of RAM. */
  RAM    : ORIGIN = 0x20000000, LENGTH = 36K
}
