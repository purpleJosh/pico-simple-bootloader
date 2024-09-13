//****************************************************************************
// Copyright 2021 Richard Hulme
//
// SPDX-License-Identifier: BSD-3-Clause
//
// Flashloader for the RP2040
//
// If there is no new application to be flashed (according to the watchdog
// scratch registers), the flashloader will simply transfer control to the
// existing application starting in the erase block following the flashloader.
// This is the normal case.
//
// If the watchdog scratch registers are correctly set, the flashloader will
// replace the existing application with the new one stored in the location
// provided and then reboot the processor.
//
// If the existing application is invalid, the flashloader will check
// each erase block for a valid update image and flash that if successful.
//
// If all else fails (application is invalid and no update image can be found)
// the flashloader will drop back to bootrom bootloader.

#include <stdint.h>
#include "hardware/regs/addressmap.h"
#include "hardware/regs/m0plus.h"
#include "hardware/structs/watchdog.h"
#include "hardware/clocks.h"
#include "hardware/pll.h"
#include "hardware/xosc.h"
#include "hardware/resets.h"
#include "hardware/dma.h"
#include "hardware/flash.h"
#include "hardware/watchdog.h"
#include "pico/bootrom.h"
#include "pico/binary_info.h"
#include "flashloader.h"

// Including the stdlib.h file significantly increases the bootloader size
// Would need to manually implement certain functions if we ever wanted to prevent this
#include "pico/stdlib.h"


// Not the best to include ws2812b.c directly
// All header files MUST be included above this
#include "ws2812b.c"

bi_decl(bi_program_version_string("1.10"));

#if !PICO_FLASH_SIZE_BYTES
    #error PICO_FLASH_SIZE_BYTES not defined!
#endif

extern void* __APPLICATION_START;

//****************************************************************************
// We don't normally want to link against pico_stdlib as that pulls in lots of
// other stuff we don't need here and we end up at just under 8k before we've
// even started.
// Sometimes though it *is* helpful to use pico_stdlib for checking weird
// errors aren't because of some missing initialisation here so uncomment this
// define to make the necessary changes here but don't forget to change the
// start address for the application in the linker script!
#define USE_PICO_STDLIB

#define flashoffset(x) (((uint32_t)x) - XIP_BASE)
#define bl2crc(x)      (*((uint32_t*)(((uint32_t)(x) + 0xfc))))

static const uint32_t  sStart = XIP_BASE + (uint32_t)&__APPLICATION_START;

// The maximum number of times the flashloader will try to flash an image
// before it gives up and boots in the bootrom bootloader
static const uint32_t  sMaxRetries = 3;

// Nothing else is running on the system, so it doesn't matter which
// DMA channel we use
static const uint8_t sDMAChannel = 0;

//// Buffer to store a page's worth of data for flashing.
//// Must be aligned to a 256 byte boundary to allow use as a DMA ring buffer
static uint8_t sPageBuffer[256] __attribute__ ((aligned(256)));


#ifndef USE_PICO_STDLIB
//****************************************************************************
// These functions are normally provided as part of pico_stdlib so we have to
// provide them here if not we're not using it.
void exit(int ret)
{
    (void)ret;
    while(true)
        tight_loop_contents();
}

void panic(const char* fmt,...)
{
    (void)fmt;
    while(true)
        tight_loop_contents();
}

void hard_assertion_failure(void)
{
    while(true)
        tight_loop_contents();
}

//****************************************************************************
// Provide our own assert function to prevent the default version pulling
// in 'printf' functions in debug builds
void __assert_func(const char *filename,
                   int line,
                   const char *assert_func,
                   const char *expr)
{
    (void)filename;
    (void)line;
    (void)assert_func;
    (void)expr;

    __breakpoint();

    while(true)
        tight_loop_contents();
}
#endif

//****************************************************************************
// Calculate the CRC32 (no reflection, no final XOR) of a block of data.
// This makes use of the DMA sniffer to calculate the CRC for us.  Speed is
// not really a huge issue as most of the time we just need to check the
// boot2 image is valid (252 bytes) but using DMA ought to be faster than
// looping over the data without a lookup table and is certainly a lot smaller
// than the lookup table.
uint32_t crc32(const void *data, size_t len, uint32_t crc)
{
    uint8_t dummy;

    dma_channel_config c = dma_channel_get_default_config(sDMAChannel);
    channel_config_set_transfer_data_size(&c, DMA_SIZE_8);
    channel_config_set_read_increment(&c, true);
    channel_config_set_write_increment(&c, false);
    channel_config_set_sniff_enable(&c, true);

    // Turn on CRC32 (non-bit-reversed data)
    dma_sniffer_enable(sDMAChannel, 0x00, true);
    dma_hw->sniff_data = crc;

    dma_channel_configure(
        sDMAChannel,
        &c,
        &dummy,
        data,
        len,
        true    // Start immediately
    );

    dma_channel_wait_for_finish_blocking(sDMAChannel);

    return(dma_hw->sniff_data);
}

//****************************************************************************
// Prepare DMA channel to read a page (256 bytes) worth of data into the
// page buffer starting from the given address.
// The copying will be triggered by calling 'copyPage'
void copyPageInit(const void* src)
{
    dma_channel_config c = dma_channel_get_default_config(sDMAChannel);
    channel_config_set_transfer_data_size(&c, DMA_SIZE_32);
    channel_config_set_read_increment(&c, true);
    channel_config_set_write_increment(&c, true);
    channel_config_set_sniff_enable(&c, true);
    channel_config_set_ring(&c, true, 8); // (log2(256) == 8)

    // Turn on CRC32 (non-bit-reversed data)
    dma_sniffer_enable(sDMAChannel, 0x00, true);
    dma_hw->sniff_data = 0xffffffff;

    dma_channel_configure(
        sDMAChannel,
        &c,
        sPageBuffer,
        src,
        256 / 4,
        false    // Do not start immediately
    );
}

//****************************************************************************
// Copy the next page of data to the page buffer and return the updated CRC32
uint32_t copyPage()
{
    dma_channel_start(sDMAChannel);
    dma_channel_wait_for_finish_blocking(sDMAChannel);
    return(dma_hw->sniff_data);
}

//****************************************************************************
// Start the main application if its boot2 image is valid.
// Will not return unless the image is invalid
int startMainApplication()
{
    if(crc32((const void*)sStart, 252, 0xffffffff) == bl2crc(sStart))
    {
        // Main application appears to be OK so we can map the application's
        // vector table and jump to the start of its code

        // First make sure we don't get retriggered
        if((watchdog_hw->scratch[0] == FLASH_MAGIC1) ||
           (watchdog_hw->scratch[0] == ~FLASH_MAGIC1))
        {
            watchdog_hw->scratch[0] = 0;
        }

        // Hold DMA block in reset again (in case the application doesn't
        // need DMA and doesn't want to waste power)
        reset_block(RESETS_RESET_DMA_BITS);



        // This is the where we jump to the application.
        // Assembly is needed for things like setting the stack pointer
        asm volatile (
        "mov r0, %[start]\n" 
        "ldr r1, =%[vtable]\n" 
        "str r0, [r1]\n" 
        "ldmia r0, {r0, r1}\n" 
        "msr msp, r0\n" 
        "bx r1\n" 
        : 
        : [start] "r" (sStart + 0x100), [vtable] "X" (PPB_BASE + M0PLUS_VTOR_OFFSET)
        : 
        );
        // : outputs (none listed above)
        // : inputs ......
        // : list of clobbered registers (none listed above)
    }

    // We will only return if the main application couldn't be started
    return 0;
}

//****************************************************************************
// Configure one of either clk_ref or clk_sys.
//
// This is mostly lifted from clock_configure with some code removed that
// doesn't apply to clk_ref or clk_sys and using a fixed divisor
void configClock(enum clock_index clk_index, uint32_t src, uint32_t auxsrc)
{
    clock_hw_t *clock = &clocks_hw->clk[clk_index];

    clock->div = 0x100; // divisor == 1.00

    if(src == CLOCKS_CLK_SYS_CTRL_SRC_VALUE_CLKSRC_CLK_SYS_AUX)
    {
        hw_clear_bits(&clock->ctrl, CLOCKS_CLK_REF_CTRL_SRC_BITS);
        while(!(clock->selected & 1u))
            tight_loop_contents();
    }

    // Set aux mux first, and then glitchless mux
    hw_write_masked(&clock->ctrl,
                    (auxsrc << CLOCKS_CLK_SYS_CTRL_AUXSRC_LSB),
                    CLOCKS_CLK_SYS_CTRL_AUXSRC_BITS);

    hw_write_masked(&clock->ctrl,
                    src << CLOCKS_CLK_REF_CTRL_SRC_LSB,
                    CLOCKS_CLK_REF_CTRL_SRC_BITS);
    while(!(clock->selected & (1u << src)))
        tight_loop_contents();
}

//****************************************************************************
// Use the external oscillator as the clock reference to gain a bit of speed!
void initClock()
{
    // Start tick in watchdog
    watchdog_start_tick(XOSC_MHZ);

    clocks_hw->resus.ctrl = 0;

    xosc_init();

    // Before we touch PLLs, switch sys and ref cleanly away from their aux sources.
    hw_clear_bits(&clocks_hw->clk[clk_sys].ctrl, CLOCKS_CLK_SYS_CTRL_SRC_BITS);
    while (clocks_hw->clk[clk_sys].selected != 0x1)
        tight_loop_contents();
    hw_clear_bits(&clocks_hw->clk[clk_ref].ctrl, CLOCKS_CLK_REF_CTRL_SRC_BITS);
    while (clocks_hw->clk[clk_ref].selected != 0x1)
        tight_loop_contents();

    pll_init(pll_sys, 1, 1500 * MHZ, 6, 2);

    configClock(clk_ref,
                CLOCKS_CLK_REF_CTRL_SRC_VALUE_XOSC_CLKSRC,
                0); // No aux mux

    configClock(clk_sys,
                CLOCKS_CLK_SYS_CTRL_SRC_VALUE_CLKSRC_CLK_SYS_AUX,
                CLOCKS_CLK_SYS_CTRL_AUXSRC_VALUE_CLKSRC_PLL_SYS);
}


#define PUSH_BUTTON_PIN 1
bool is_button_pushed(){
    return !gpio_get(PUSH_BUTTON_PIN);
}

//****************************************************************************
int main(void)
{
    
    uint32_t scratch = watchdog_hw->scratch[0];
    uint32_t image   = watchdog_hw->scratch[1];

    // Use xosc, which will give us a speed boost
    initClock();

    // Take DMA block out of reset so we can use it to calculate CRCs
    unreset_block_wait(RESETS_RESET_DMA_BITS);
	
    
    // This is used purely for the LED code
    /* 100MHz is a clean number and used to calculate the cycle delays */
    set_sys_clock_khz(100000, true);
	
    /* Wait a bit to ensure clock is running and force LEDs to reset*/
    sleep_ms(10);


    bool leds_are_inited = false;
    if ( is_button_pushed() ){
        gpio_init(LED_PIN);
        gpio_set_dir(LED_PIN, GPIO_OUT);
        gpio_put(LED_PIN, false); /* Important to start low to tell the LEDs that it's time for new data */

        leds_are_inited = true;

        // One of the LEDs is turning on after we init
        // This is a poor attempt to handle that, and
        // results in the single LED only briefly flashing
        for(int i = 0; i < 10; i++){
            set_all(0, 0, 0);
            send_led_data();
            sleep_ms(1);
        }
        

        int cnt_ms = 3000;
        int step_ms = 150;
        
        while ( is_button_pushed() && cnt_ms > 0){
            cnt_ms -= step_ms;
            sleep_ms(step_ms);

            if ( cnt_ms % (step_ms * 2) == 0 ){
                // Flash the LEDs white at a lower brightness
                set_all(20, 20, 20);
            }
            else{
                // Turn the LEDs off
                set_all(0, 0, 0);
            }

            // Actually sends the modified LED buffer to be displayed
            send_led_data();
        }

        if ( cnt_ms < 1 ){
            // Go into Bootsel instead of starting the application
            set_all(20, 20, 20);
            send_led_data();

            reset_usb_boot(0, 0);
            return 0;
        }
        else{
            // Turn the LEDs off
            set_all(0, 0, 0);
            sleep_ms(1);
            send_led_data();
        }
    }
        
    if((scratch == FLASH_MAGIC1) && ((image & 0xfff) == 0) && (image > sStart))
    {
        // Invert the magic number (so we know we've been here) and
        // initialise the retry counter
        watchdog_hw->scratch[0] = ~FLASH_MAGIC1;
        watchdog_hw->scratch[2] = 0;
    }
    else
    if(scratch == ~FLASH_MAGIC1)
        watchdog_hw->scratch[2]++;
    else
    if(!startMainApplication())
    {
        // Tried and failed to start the main application so try to find
        // an update image
        image = sStart + 0x1000;

        // In case there are any problems during flashing, make it look like
        // an update was requested so that the retry mechanism is correctly
        // initialised.
        watchdog_hw->scratch[0] = ~FLASH_MAGIC1;
        watchdog_hw->scratch[1] = image;
        watchdog_hw->scratch[2] = 0;
    }

    
    // Something's gone wrong.  The main application is corrupt

    // If we were originally explicitly triggered by a watchdog reset, try
    // to start the normal application since we didn't before
    if((scratch == FLASH_MAGIC1) || (scratch == ~FLASH_MAGIC1))
        startMainApplication();

    // Init the LEDs if they aren't already
    if ( ! leds_are_inited ){
        gpio_init(LED_PIN);
        gpio_set_dir(LED_PIN, GPIO_OUT);
        gpio_put(LED_PIN, false); /* Important to start low to tell the LEDs that it's time for new data */

        leds_are_inited = true;
    }

    set_all(20, 20, 20);
    sleep_ms(1);
    send_led_data();

    reset_usb_boot(0, 0);
}
