/* 
Maria Louise Quitoriano
MP1 Project, Checked last November 25, 2024
2022-06275
*/

#include <stdio.h>
#include <stdlib.h>
#include <xc.h>
#include <stdbool.h>
#include <stdint.h>
#include <component/eic.h>

void __attribute__((interrupt())) EIC_EXTINT_0_Handler(void);
void __attribute__((interrupt())) EIC_EXTINT_1_Handler(void);
static void LED_Init(void);
void TCC3_Init(void);
void Clock_Source(void);
static void SW_Init(void);
static void EIC_Init(void);
static void NVIC_Init(void);

void POT_Init(void);
void CLOCK_Init(void);
void ADC_Init(void);
void ADC_Enable(void);
void ADC_ConversionStart(void);
uint16_t ADC_ConversionResultGet(void);
bool ADC_ConversionStatusGet(void);
void delay_ms(int delay);
void LED_SetColor(uint8_t color_index, uint8_t brightness);
void cycle_color(int delay_time, uint8_t pot_percentage);

uint16_t adc_value = 0x0000;
//uint16_t mask = 0x0000;
//uint16_t move = 0x0000;

//void LED_SetColor(0, 255/2);

volatile unsigned int pressed_SW1_LED = 0;
volatile unsigned int pressed_SW2_LED = 0;
volatile unsigned int pressed_SW1;
volatile unsigned int pressed_SW2;
volatile unsigned int curr_Brightness = 128;
uint8_t curr_Color_IDX = 0;

uint8_t colors[5][3] = {
    {122, 31, 206}, // color 0 - 0 
    {16, 233, 110}, // color 1 - 6
    {22, 221, 229}, // color 2 - 2
    {244, 164, 96}, // color 3 - 7
    {220, 20, 60}  // color 4 - 5
};

int main() {
    Clock_Source();
    LED_Init();
    TCC3_Init();
    SW_Init();
    EIC_Init();
    NVIC_Init();
    POT_Init();
    CLOCK_Init();
    
    ADC_Init();
    ADC_Enable();

    // Display color 0 with 50% brightness initially if no buttons have been pressed
    if (!pressed_SW1 && !pressed_SW2) {
        LED_SetColor(0, 128);  // Show color 0 with 50% brightness
    }

    int delay_time = 0;
    
    while (1) {
        // Start ADC conversion to read potentiometer value
        ADC_ConversionStart();
        while (!ADC_ConversionStatusGet());
        adc_value = ADC_ConversionResultGet();
         
        uint8_t pot_percentage = (((float)adc_value / 1023) * 100); // Convert ADC value to percentage

        // **SW1 Behavior**: Adjust brightness based on potentiometer
        if (pressed_SW1) {
            curr_Brightness = (pot_percentage * 255) / 100;  // Scale brightness to 0-255
            LED_SetColor(curr_Color_IDX, curr_Brightness);  // Update LED with new brightness
            pressed_SW1 = 0;  // Reset SW1 press flag
        }

        // **SW2 Behavior**: Control color cycling based on potentiometer position
        if (pressed_SW2) {
            //pressed_SW2_LED++;  // Count how many times SW2 has been pressed
            pressed_SW2 = 0;
            // Set delay_time based on pot_percentage
            if (pot_percentage <= 20) {
                delay_time = 400;
                //cycle_color(delay_time, pot_percentage);
            } else if (pot_percentage <= 40) {
                delay_time = 800;
                //cycle_color(delay_time, pot_percentage);
            } else if (pot_percentage <= 60) {
                delay_time = 0; // Frozen state (no cycling)
            } else if (pot_percentage <= 80) {
                delay_time = 800;
                //cycle_color(delay_time, pot_percentage);
            } else {
                delay_time = 400;
                //cycle_color(delay_time, pot_percentage);
            }
        
            //pressed_SW2 = 0;
        }
        if (delay_time > 0){
            cycle_color(delay_time, pot_percentage);
        }
        //pressed_SW2 = 0;
        
    }
    return(EXIT_SUCCESS);
}

void cycle_color(int delay_time, uint8_t pot_percentage) {
    static uint8_t last_pot_percentage = 128; // Store last potentiometer value to detect changes

    // If the potentiometer has changed, adjust the cycle time and direction
    if (pot_percentage != last_pot_percentage) {
        last_pot_percentage = pot_percentage;
        // Update delay_time based on the new pot_percentage
        if (pot_percentage <= 20) {
            delay_time = 400;
        } else if (pot_percentage <= 40) {
            delay_time = 800;
        } else if (pot_percentage <= 60) {
            delay_time = 0;  // Frozen state (no cycling)
        } else if (pot_percentage < 80) {
            delay_time = 800;
        } else {
            delay_time = 400;
        }
        //pressed_SW2 = 0;
    }

    // Cycle colors based on updated delay_time and pot_percentage
    if (pot_percentage <= 20 || pot_percentage <= 40) {
        delay_ms(delay_time);
        curr_Color_IDX = (curr_Color_IDX - 1 + 5) % 5;  // Normal order
        LED_SetColor(curr_Color_IDX, curr_Brightness);  // Update LED with current color and brightness
    } 
    else if (pot_percentage <= 60) {
        delay_ms(0);  // Frozen state (no cycling)
    } 
    else if (pot_percentage < 80 || pot_percentage <= 100) {
        delay_ms(delay_time);
        curr_Color_IDX = (curr_Color_IDX + 1) % 5;  // Reverse order
        LED_SetColor(curr_Color_IDX, curr_Brightness);
    }
}


void __attribute__((interrupt())) EIC_EXTINT_0_Handler(void){
    EIC_SEC_REGS->EIC_INTFLAG |= (1<<0);
    pressed_SW1 = 1;
}

void __attribute__((interrupt())) EIC_EXTINT_1_Handler(void){
    EIC_SEC_REGS->EIC_INTFLAG |= (1<<1);
    pressed_SW2 = 1;
}

void LED_SetColor(uint8_t color_index, uint8_t brightness) {
    // Get color values for the selected index
    uint8_t red = (colors[color_index][0] * brightness) / 255;   // Scale by brightness
    uint8_t green = (colors[color_index][1] * brightness) / 255; // Scale by brightness
    uint8_t blue = (colors[color_index][2] * brightness) / 255;  // Scale by brightness
    
    // Set duty cycles for the channels
    TCC3_REGS->TCC_CC[1] = ((red * (TCC3_REGS->TCC_PER + 1)) / 255);  // RED CHANNEL
    TCC3_REGS->TCC_CC[0] = ((green * (TCC3_REGS->TCC_PER + 1)) / 255); // GREEN CHANNEL
    TCC3_REGS->TCC_CC[3] = ((blue * (TCC3_REGS->TCC_PER + 1)) / 255); // BLUE CHANNEL
}

static void LED_Init(void){
    // Configure PB03 for the blue channel (Group 1)
    PORT_SEC_REGS->GROUP[1].PORT_DIRSET = (1 << 3);       // Set PB03 as output
    PORT_SEC_REGS->GROUP[1].PORT_PINCFG[3] = 0x1;         // Enable input and PMUX for PB03
    PORT_SEC_REGS->GROUP[1].PORT_PMUX[1] = 0x90;          // Set PMUX for PWM control

    // Configure PA06 for the green channel (Group 0)
    PORT_SEC_REGS->GROUP[0].PORT_DIRSET = (1 << 6);       // Set PA06 as output
    PORT_SEC_REGS->GROUP[0].PORT_PINCFG[6] = 0x1;         // Enable input and PMUX for PA06
    PORT_SEC_REGS->GROUP[0].PORT_PMUX[3] = 0x09;          // Even pin configuration for PWM control

    // Configure PA03 for the red channel (Group 0)
    PORT_SEC_REGS->GROUP[0].PORT_DIRSET = (1 << 3);       // Set PA03 as output
    PORT_SEC_REGS->GROUP[0].PORT_PINCFG[3] = 0x1;         // Enable input and PMUX for PA03
    PORT_SEC_REGS->GROUP[0].PORT_PMUX[1] = 0x90;          // Set PMUX for PWM control
}

void TCC3_Init(void){
    GCLK_REGS->GCLK_PCHCTRL[27] = (1<<6); // Peripheral Channel Enabled
    while((GCLK_REGS->GCLK_PCHCTRL[27] & (1<<6) == 0));
    
    TCC3_REGS->TCC_CTRLA = (1<<0); // SWRST
    while(TCC3_REGS->TCC_SYNCBUSY & ~(1<<0));
    
    TCC3_REGS->TCC_CTRLA = (0x1 << 4) | (0x7 << 8); //PRESC | PRESCALER, 1024
    TCC3_REGS->TCC_WEXCTRL = TCC_WEXCTRL_OTMX(0UL); //DEFAULT CONFIG
    TCC3_REGS->TCC_WAVE = (0x2 << 0); // Normal PWM
    
    // Calculate PER for 100 Hz frequency
    uint32_t GCLK_frequency = 48000000; // Example GCLK frequency (48 MHz)
    uint32_t prescaler = 1024; // Prescaler set above
    uint32_t desired_frequency = 100; // 100 Hz

    uint32_t PER = (GCLK_frequency / (prescaler * desired_frequency)) - 1;

    TCC3_REGS->TCC_PER = PER;
    
    // Get color values for color 0
    uint8_t red = (colors[0][0]);   // 122 / 2
    uint8_t green = (colors[0][1]); // 31 / 2
    uint8_t blue = (colors[0][2]);  // 206 / 2 
    
    //CALULATED DUTY CYCLES
    TCC3_REGS->TCC_CC[1] = ((red * (PER + 1)) / 255);  // RED CHANNEL
    TCC3_REGS->TCC_CC[0] = ((green * (PER + 1)) / 255); // GREEN CHANNEL
    TCC3_REGS->TCC_CC[3] = ((blue * (PER + 1)) / 255); // BLUE CHANNEL

    TCC3_REGS->TCC_CTRLA |= (1<<1); // ENABLED TCC
    while(TCC3_REGS->TCC_SYNCBUSY &~(1<<1)); // wait for sync
}

void Clock_Source(void){
    PM_REGS->PM_INTFLAG = 0x01;
    PM_REGS->PM_PLCFG = 0x02;
    while((PM_REGS->PM_INTFLAG & 0x01)==0)
        asm ("nop");
    PM_REGS->PM_INTFLAG = 0x01;
    
    // use gen2 to retain a 4MHz clock source
    GCLK_REGS->GCLK_GENCTRL[2] = 0x00010105;
    while((GCLK_REGS->GCLK_SYNCBUSY & (1<<4)) != 0)
        asm("nop");
    
     /****************** Controls Initialization  *********************/    
    NVMCTRL_SEC_REGS->NVMCTRL_CTRLB = (2<<1);    
    SUPC_REGS->SUPC_VREGPLL = (1 << 1);
    
    /****************** DFLL Initialization  *********************/
    OSCCTRL_REGS->OSCCTRL_DFLLCTRL = 0;
    while((OSCCTRL_REGS->OSCCTRL_STATUS & (1<<24)) != (1<<24));  /* Waiting for the Ready state */

    /*Load Calibration Value*/
    uint8_t calibCoarse = (uint8_t)(((*(uint32_t*)0x00806020U) >> 25U ) & 0x3fU);
    OSCCTRL_REGS->OSCCTRL_DFLLVAL = OSCCTRL_DFLLVAL_COARSE((uint32_t)calibCoarse) | OSCCTRL_DFLLVAL_FINE((uint32_t)512U);
    while((OSCCTRL_REGS->OSCCTRL_STATUS & (1<<24)) != (1<<24)); /* Waiting for the Ready state */

    /* Configure DFLL    */
    OSCCTRL_REGS->OSCCTRL_DFLLCTRL = (1<<1) ;
    while((OSCCTRL_REGS->OSCCTRL_STATUS & (1<<24)) != (1<<24)); /* Waiting for DFLL to be ready */
    
    /****************** GCLK 0 Initialization  *********************/    
    GCLK_REGS->GCLK_GENCTRL[0] = (1<<16) | (7<<0) | (1<<8);
    while(GCLK_REGS->GCLK_SYNCBUSY & ~(1<<2));
}

static void SW_Init(void){
    //setting the switches with a pull down, active low configuration
    PORT_SEC_REGS->GROUP[0].PORT_DIRCLR = (1 << 0); //switch 1 PA00, even
    PORT_SEC_REGS->GROUP[0].PORT_PINCFG[0] = 0x03; // INEN AND PMUX
    PORT_SEC_REGS->GROUP[0].PORT_PMUX[0] = (0x0 << 0);
    
    PORT_SEC_REGS->GROUP[0].PORT_DIRCLR = (1 << 1); //switch 2 PA01, odd
    PORT_SEC_REGS->GROUP[0].PORT_PINCFG[1] = 0x03;
    PORT_SEC_REGS->GROUP[0].PORT_PMUX[0] = (0x0 << 4);

}

static void EIC_Init(void){
    SW_Init();
    
    /* Enable the EIC peripheral clock */
    MCLK_REGS->MCLK_APBAMASK |= MCLK_APBAMASK_EIC_Msk;
    
    /* To enable the filter and debouncer in EIC, the GCLK_EIC should be enabled */
    GCLK_REGS->GCLK_PCHCTRL[4] = 0x00000040;
	while ((GCLK_REGS->GCLK_PCHCTRL[4] & 0x00000040) == 0);
    
    EIC_SEC_REGS->EIC_CTRLA = 0x01;
    while((EIC_SEC_REGS->EIC_SYNCBUSY & 0x01) == 0x01);
    
    // Configure EXTINT0 (PA00) and EXTINT1 (PA01) for falling edge with filter enabled
    EIC_SEC_REGS->EIC_CONFIG0 = (0x2 << 0) | (1 << 3) | (0x2 << 4) | (1 << 7); // falling edge | filter enable
    EIC_SEC_REGS->EIC_DEBOUNCEN = (1 << 0) | (1 << 1); // Enable debouncer for EXTINT0 and EXTINT1
    EIC_SEC_REGS->EIC_DPRESCALER = 0x000100FF; // Set prescaler
    
    // Enable the external interrupts for EXTINT0 and EXTINT1
    EIC_SEC_REGS->EIC_INTENSET = (1 << 0) | (1 << 1); 
    EIC_SEC_REGS->EIC_CTRLA = (1 << 1); // Enable the EIC
    while ((EIC_SEC_REGS->EIC_SYNCBUSY & (1 << 1)) == (1 << 1));
    
    // Clear INTFLAG for EXTINT0 and EXTINT1
    EIC_SEC_REGS->EIC_INTFLAG |= (1 << 0) | (1 << 1); 
    
    // Configure PA00 and PA01 as inputs with pull-down configuration
    PORT_SEC_REGS->GROUP[0].PORT_DIRCLR = (1 << 0) | (1 << 1);
    PORT_SEC_REGS->GROUP[0].PORT_PINCFG[0] = (1 << 1) | (1 << 0); // Enable input and PMUX for PA00
    PORT_SEC_REGS->GROUP[0].PORT_PINCFG[1] = (1 << 1) | (1 << 0); // Enable input and PMUX for PA01
}

static void NVIC_Init(void){
    __DMB();
    __enable_irq();
    
    // setting prio for extint 0 and 1
    NVIC_SetPriority(EIC_EXTINT_0_IRQn, 3); // extint 0 priority to 3
    NVIC_EnableIRQ(EIC_EXTINT_0_IRQn);
    
    NVIC_SetPriority(EIC_EXTINT_1_IRQn, 3); // priority to 3 
    NVIC_EnableIRQ(EIC_EXTINT_1_IRQn);  
    // giving them equal priorities
}

void POT_Init(void){
    //PB02 as the potentiometer input
    PORT_SEC_REGS->GROUP[1].PORT_PINCFG[2] = 0x1U;
    PORT_SEC_REGS->GROUP[1].PORT_PMUX[1] = 0x1U;
    /*
    // PA03 AS THE OUTPUT PIN GPIO_PA03
    PORT_SEC_REGS->GROUP[0].PORT_DIRSET = (1<<3) ;
    PORT_SEC_REGS->GROUP[0].PORT_PINCFG[3] = 0x0U;
    PORT_SEC_REGS->GROUP[0].PORT_PMUX[1] = 0x0U;
    
    // PA06 AS THE OUTPUT PIN GPIO_PA06
    PORT_SEC_REGS->GROUP[0].PORT_DIRSET = (1<<6) ;
    PORT_SEC_REGS->GROUP[0].PORT_PINCFG[6] = 0x0U;
    PORT_SEC_REGS->GROUP[0].PORT_PMUX[3] = 0x0U;
    
    // PB03 AS THE OUTPUT PIN GPIO_PB03
    PORT_SEC_REGS->GROUP[1].PORT_DIRSET = (1<<3) ;
    PORT_SEC_REGS->GROUP[1].PORT_PINCFG[3] = 0x0U;
    PORT_SEC_REGS->GROUP[1].PORT_PMUX[1] = 0x0U;
    */
    
    LED_Init();
    
}

void CLOCK_Init(void){
    // GCK0 : Div factor 1 | Source Select 7 | Generic Clock Generator Enable
    GCLK_REGS->GCLK_GENCTRL[0] = (1<<16) | (7<<0) | (1<<8);
    while ((GCLK_REGS->GCLK_SYNCBUSY & (1<<2)) == 0);
    // ADC Bus Clock : Generic Clock Generator Value | Channel Enable
    GCLK_REGS->GCLK_PCHCTRL[28] = (0<<0) | (1<<6);
    while ((GCLK_REGS->GCLK_PCHCTRL[28] & (1<<6)) != (1<<6));
}

void ADC_Init(void){
    /* Reset ADC */
    ADC_REGS->ADC_CTRLA = (1<<0) ;
    while ((ADC_REGS->ADC_SYNCBUSY & (1<<0) ) == (1<<0)) ;
    /* Prescaler */
    ADC_REGS->ADC_CTRLB = (2<<0) ;
    /* Sampling length */
    ADC_REGS->ADC_SAMPCTRL = (3<<0) ;
    /* Reference */
    ADC_REGS->ADC_REFCTRL = (0x5 <<0) ; //AVDD REFERENCE SELECTION
    /* Input pin */
    ADC_REGS->ADC_INPUTCTRL = (0x0A <<0); // adc 10 
    /* Resolution & Operation Mode */
    ADC_REGS->ADC_CTRLC = (0x2 <<4) | (0 <<8) ; //resolution was changed
    /* Clear all interrupt flags */
    ADC_REGS->ADC_INTFLAG = 0x07 ;
    while (0U != ADC_REGS->ADC_SYNCBUSY) ;
    
}

/* Enable ADC module */
void ADC_Enable (void){
    ADC_REGS -> ADC_CTRLA |= (1 <<1) ; // adc is enabled
    while (0U != ADC_REGS -> ADC_SYNCBUSY ) ;
}

/* Start the ADC conversion by SW */
void ADC_ConversionStart ( void ){
    ADC_REGS->ADC_SWTRIG |= (1 <<1) ;
    while ((ADC_REGS -> ADC_SYNCBUSY & (1 <<10) ) == (1 <<10)) ;
}
/* Read the conversion result */
uint16_t ADC_ConversionResultGet ( void ){
    return ADC_REGS -> ADC_RESULT ;
}
/* Check whether result is ready */
bool ADC_ConversionStatusGet ( void ){
    bool status ;
    status = ((( ADC_REGS -> ADC_INTFLAG & (1 <<0)) >> 0) != 0U ) ;
    if (status == true ){
        ADC_REGS -> ADC_INTFLAG = (1 <<0) ;
    }
    return status ;
}
void delay_ms ( int delay ) {
    int i ;
    for (; delay > 0; delay--){
        for ( i = 0; i < 2657; i ++) ;
    } 
}