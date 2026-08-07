MEMORY
{
    /* QEMU `netduinoplus2` machine (STM32F405): 1 MB flash, 128 KB RAM.
       (QEMU models 128K SRAM; the real F405 has 192K.) */
    FLASH : ORIGIN = 0x08000000, LENGTH = 1024K
    RAM   : ORIGIN = 0x20000000, LENGTH = 128K
}
