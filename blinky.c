// Christian Carter
// Oct. 14 2019
// Ong Lab
// v1.5
// Code for micro-controller EK-TM4C123GXL on the TI Tiva C Launchpad
// Main goals for the code are to generate a sine wave at a specific frequency for a period of time and then measure the response
// using the on-board ADC to count the number of oscillations in a given period of time.



#include <stdint.h>
#include <stdbool.h>
#include <stdio.h>
#include <math.h>
#include <stdlib.h>
#include <string.h>
#include "inc/hw_ints.h"
#include "inc/hw_memmap.h"
#include "inc/hw_ssi.h"
#include "inc/hw_gpio.h"
#include "inc/hw_types.h"
#include "driverlib/timer.h"
#include "driverlib/gpio.h"
#include "driverlib/interrupt.h"
#include "driverlib/pin_map.h"
#include "driverlib/rom.h"
#include "driverlib/rom_map.h"
#include "driverlib/sysctl.h"
#include "driverlib/ssi.h"
#include "driverlib/systick.h"
#include "driverlib/ssi.c"
#include "driverlib/uart.h"
#include "driverlib/fpu.h"
#include "driverlib/adc.h"

#ifdef DEBUG
void
__error__(char *pcFilename, uint32_t ui32Line)
{
    while(1);
}
#endif

uint32_t STEP_SIZE = 1000;
uint32_t low_bound = 10000;
uint32_t high_bound = 2000000;
uint32_t f_reg;
uint32_t i;
uint32_t frequencyRegister[2][1];
uint32_t rows;
uint32_t lowVoltage;
uint32_t midVoltage;
uint32_t highVoltage;
uint32_t ADCCutoff[1] = {0};
uint32_t voltage1 = 0;
uint32_t voltage2 = 1;
uint32_t voltage3 = 3;
uint32_t voltage4 = 4;
uint32_t refVoltage = 5;
uint32_t voltCode[4];

//The SSI communication code was grabbed from an engineering project from Purdue. The link to the paper is
//https://engineering.purdue.edu/ece477/Archive/2014/Spring/S14-Grp1/docs/software/LM4F-LaunchPad-11%20-%20SPI.pdf
//The bitwise reverse function was taken out because it is not needed.

/* Pins used:
 * A0, A1, A2, A3, A5
 * C4, C5, C6
 * D0, D1, D3
 */


void
PeriphEnable(void){
    SysCtlPeripheralEnable(SYSCTL_PERIPH_SSI0);
    SysCtlPeripheralEnable(SYSCTL_PERIPH_SSI1);
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOA);
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOD);
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOC);
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOE);
    SysCtlPeripheralEnable(SYSCTL_PERIPH_ADC0);
    SysCtlPeripheralReset(SYSCTL_PERIPH_ADC0);
    SysCtlPeripheralEnable(SYSCTL_PERIPH_UART0);
}

void
PinConfig(void){
    GPIOPinTypeADC(GPIO_PORTE_BASE, GPIO_PIN_2);
    GPIOPinConfigure(GPIO_PA0_U0RX);
    GPIOPinConfigure(GPIO_PA1_U0TX);
    GPIOPinConfigure(GPIO_PA2_SSI0CLK);
    GPIOPinConfigure(GPIO_PA3_SSI0FSS);
    GPIOPinConfigure(GPIO_PA5_SSI0TX);
    GPIOPinConfigure(GPIO_PD0_SSI1CLK);
    GPIOPinConfigure(GPIO_PD1_SSI1FSS);
    GPIOPinConfigure(GPIO_PD3_SSI1TX);
    GPIOPinTypeGPIOInput(GPIO_PORTC_BASE, GPIO_PIN_4 | GPIO_PIN_5 | GPIO_PIN_6);
    GPIOPinTypeSSI(GPIO_PORTA_BASE, GPIO_PIN_5|GPIO_PIN_3|GPIO_PIN_2);
    GPIOPinTypeSSI(GPIO_PORTD_BASE, GPIO_PIN_3|GPIO_PIN_1|GPIO_PIN_0);
    GPIOPinTypeUART(GPIO_PORTA_BASE, GPIO_PIN_0 | GPIO_PIN_1);

}
/*
void
UARTIntHandler(void){
    uint32_t ui32Status;
    ui32Status = UARTIntStatus(UART0_BASE, true);
    UARTIntClear(UART0_BASE, ui32Status);
}

void
UARTInit(void){
    UARTConfigSetExpClk(UART0_BASE, SysCtlClockGet(), 115200, (UART_CONFIG_WLEN_8 | UART_CONFIG_STOP_ONE | UART_CONFIG_PAR_NONE));
    IntEnable(INT_UART0);
    UARTIntEnable(UART0_BASE, UART_INT_RX | UART_INT_RT);
    IntMasterEnable();
}
*/
void
SSIInit(void){
    SysCtlClockSet(SYSCTL_SYSDIV_1 | SYSCTL_USE_PLL | SYSCTL_OSC_MAIN | SYSCTL_XTAL_16MHZ);
    SSIConfigSetExpClk(SSI0_BASE,SysCtlClockGet(),SSI_FRF_MOTO_MODE_2,SSI_MODE_MASTER,5000,16);
    SSIConfigSetExpClk(SSI1_BASE,SysCtlClockGet(),SSI_FRF_MOTO_MODE_0,SSI_MODE_MASTER,5000,11);
    SSIEnable(SSI0_BASE);
    SSIEnable(SSI1_BASE);
}
/*
void
DataSend(uint32_t data1, uint32_t data2, uint32_t data3, uint32_t ui32Count){
    //sends the pulse numbers from the pulse counter, through UART, to the computer
    //while(ui32Count--){
    //    UARTCharPutNonBlocking(UART0_BASE, (uint32_t)"yeet");
    //}
    while(ui32Count--){
        // Write the next character to the UART.
        UARTCharPutNonBlocking(UART0_BASE, "y");
    }
    while(ui32Count--){
        // Write the next character to the UART.
        UARTCharPutNonBlocking(UART0_BASE,"b");
    }
    while(ui32Count--){
        // Write the next character to the UART.
        UARTCharPutNonBlocking(UART0_BASE,"p");
    }
}
*/
void
FreqRegBits(uint32_t step){
   // uint32_t rows = (int)((high_bound - low_bound)/STEP_SIZE);
    uint32_t reg;
    //for(step = 0; step < rows; i++){
    reg = (int)((low_bound + (STEP_SIZE * step)) * (10.73741824));
    //reg = (int)(((low_bound + (STEP_SIZE * i)) * pow(2,28))/50000000);
    frequencyRegister[0][0] = (int)((reg % 16384) + 16384);
    frequencyRegister[1][0] = (int)((reg / (2*2*2*2*2*2*2*2*2*2*2*2*2*2)) + 16384);
}
/*
void
ADCInit(void){
//    ADCSequenceDisable(ADC0_BASE, 0);
    ADCSequenceConfigure(ADC0_BASE, 3, ADC_TRIGGER_PROCESSOR, 0);
    ADCSequenceStepConfigure(ADC0_BASE, 3, 0, (ADC_CTL_CH6 | ADC_CTL_IE | ADC_CTL_END));
    ADCSequenceEnable(ADC0_BASE, 3);
    ADCIntClear(ADC0_BASE, 3);
}

int
ADCValue(void){
    i = 0;
    while(i != 1)
        {
            ADCProcessorTrigger(ADC0_BASE, 3);
            while(!ADCIntStatus(ADC0_BASE, 3, false)){}
            ADCIntClear(ADC0_BASE, 3);
            ADCSequenceDataGet(ADC0_BASE, 3, ADCCutoff);
            i = 1;
        }
    return ADCCutoff[0];
}

int
ADCCutoffReader(void){
    //reads the "peak" value from the signal response to determine when to cut off the pulse counting
    uint32_t cutoffVoltage = 0.3; //the cut off value
    uint32_t stopTF; //holds reset bool in binary
    if(ADCValue() > cutoffVoltage){
        stopTF = 0;
    }
    else if(ADCValue() <= cutoffVoltage){
        stopTF = 1;
    }
    return stopTF;
}

int
PulseCounter(void){
    highVoltage = 0;
    midVoltage = 0;
    lowVoltage = 0;
    while(ADCCutoffReader() == 0){
        register uint32_t low = GPIOPinRead(GPIO_PORTC_BASE, GPIO_PIN_4);
        register uint32_t mid = GPIOPinRead(GPIO_PORTC_BASE, GPIO_PIN_5);
        register uint32_t high = GPIOPinRead(GPIO_PORTC_BASE, GPIO_PIN_6);
        if((high & GPIO_PIN_6)){
            highVoltage += 1;
        }
        else if((mid & GPIO_PIN_5)){
            midVoltage += 1;
        }
        else if((low & GPIO_PIN_4)){
            lowVoltage += 1;
        }
    }
    //DataSend(highVoltage, midVoltage, lowVoltage, 16);
    return 1;
}
*/
void
SineWavePulse(void){
    uint32_t rows = (int)((high_bound - low_bound)/STEP_SIZE);
    //highest frequency that can be in the first 14 bits of the frequency register is 976.5 Hz which is below the predetermined start
    //frequency of 1000 Hz. The lower 14 bits will still be programmed in but there will be logic to determine if they're needed or not.
    //The chip writes LSB in the first write and then MSB in the second write
    for(i = 0; i < rows; i++){
        FreqRegBits(i);
        uint32_t j = 0;
        while(j < 1){
            //SSIDataPut(SSI0_BASE, i);
            SSIDataPut(SSI0_BASE, 0x2100);
            SSIDataPut(SSI0_BASE, 0x7FFF);//frequencyRegister[0][0]);
            SSIDataPut(SSI0_BASE, 0x7FFF);//frequencyRegister[1][0]);
            SSIDataPut(SSI0_BASE, 0xC000);
            SSIDataPut(SSI0_BASE, 0x2000);
            while(SSIBusy(SSI0_BASE))
                {
                }
            SysCtlDelay(1600000);
            //SSIDataPut(SSI0_BASE, 0b00000100000000);
            //DataSend(1, 2, 3, 32);
            j = 1;

        }
    }
}

void
voltCodeGenerator(uint32_t voltage1, uint32_t voltage2, uint32_t voltage3, uint32_t voltage4, uint32_t refVoltage){

    voltCode[0] = (int)((voltage1 * 256)/refVoltage);
    voltCode[1] = (int)((voltage2 * 256)/refVoltage);
    voltCode[2] = (int)((voltage3 * 256)/refVoltage);
    voltCode[3] = (int)((voltage4 * 256)/refVoltage);

}

void
DCVoltage(void){
    uint32_t location1 = 0;
    uint32_t location2 = (0b01000000000);
    uint32_t location3 = (0b10000000000);
    uint32_t location4 = (0b11000000000);
    uint32_t voltbits1 = voltCode[0] + location1;
    uint32_t voltbits2 = voltCode[1] + location2;
    uint32_t voltbits3 = voltCode[2] + location3;
    uint32_t voltbits4 = voltCode[3] + location4;
    uint32_t k = 0;
    GPIOPinWrite(GPIO_PORTD_BASE, GPIO_PIN_6, 0b1);
    while(k < 1){
        SSIDataPut(SSI1_BASE, voltbits1);
        SSIDataPut(SSI1_BASE, voltbits2);
        SSIDataPut(SSI1_BASE, voltbits3);
        SSIDataPut(SSI1_BASE, voltbits4);
        k = 1;
    }
    GPIOPinWrite(GPIO_PORTD_BASE, GPIO_PIN_6, 0b0);
    SysCtlDelay(1000);
    GPIOPinWrite(GPIO_PORTD_BASE, GPIO_PIN_6, 0b1);
}

int main(void){
    PeriphEnable();
    PinConfig();
    //ADCInit();
    //UARTInit();
    SSIInit();
    voltCodeGenerator(voltage1, voltage2, voltage3, voltage4, refVoltage);
    DCVoltage();
    SysCtlDelay(100000);
    SineWavePulse();

}
