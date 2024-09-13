__FLASHLOADER_START = 0;
# It would be possible to limit the bootloader to be 8k instead of 12k. 4k would also be doable, but would require we don't link with pico-stdlib
__FLASHLOADER_LENGTH = 3 * 4k;
__APPLICATION_START = __FLASHLOADER_START + __FLASHLOADER_LENGTH;
