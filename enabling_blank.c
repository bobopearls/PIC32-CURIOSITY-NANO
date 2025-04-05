// april 5, 2025 comment: this is the code where we learn how to read the datasheet and properly enable the modes
# include <stdio.h>
# include <stdlib.h>
# include <xc.h>
 int read_count(){
    // Allow read access of COUNT register
    // Set the READSYNC command in CTRLBSET first (CTRLBSET.CMD=READSYNC)
    TC0_REGS->COUNT16.TC_CTRLBSET = TC_CTRLBSET_CMD_READSYNC;

    // Wait for synchronization before reading the COUNT register
    while (TC0_REGS->COUNT16.TC_SYNCBUSY & TC_SYNCBUSY_COUNT);

    // Return the current value of the counter
    return TC0_REGS->COUNT16.TC_COUNT; // Return the counter value
    }
 
int main () {
    /* Enabling the TC0 Bus Clock */
    // set clk gen and peripheral clock for tc0
    GCLK_REGS -> GCLK_PCHCTRL[23] = 1 << 6;
    while ((GCLK_REGS -> GCLK_PCHCTRL[23] & (1 << 6)) == 0) ; //sync channel

    /* Setting up the TC0 -> CTRLA Register */
    TC0_REGS->COUNT16.TC_CTRLA = (1<<0); // Software reset at the start
    while(TC0_REGS->COUNT16.TC_SYNCBUSY & (1 << 0)); // cleared by hardware

    TC0_REGS->COUNT16.TC_CTRLA |= (0x0 << 2); // Set to 16-bit mode
    TC0_REGS->COUNT16.TC_CTRLA |= (0x0 << 4); // Set the Prescaler and Counter Sync

    TC0_REGS->COUNT16.TC_CTRLA |= (0x7 << 8); // Set the Prescaler Factor

    /* Setting up the WAVE Register */
    TC0_REGS->COUNT16.TC_WAVE |= (0x1 << 0); // Match Frequency Operation mode
 
    // setting cc0 as the top value, it clears
    // zero becomes reload

    /* Setting the Top Value */
    TC0_REGS->COUNT16.TC_CC[0] = 0xFFFF ; // Set CC0 (Top) value to some value

    TC0_REGS->COUNT16.TC_CTRLA |=(1 << 1); // Enable the TC0 Peripheral
    while (1) {
        //Implement your code here
        if(read_count() ){
            
        }
    }
    return ( EXIT_SUCCESS );
 }
