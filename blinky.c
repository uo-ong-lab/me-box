// Christian Carter
// Oct. 14 2019
// Ong Lab
// Code for micro-controller EK-TM4C123GXL on the TI Tiva C Launchpad
// Main goals for the code are to generate a square wave at a specific frequency for a period of time and then measure the response
// using the on-board ADC to count the number of oscillations in a given period of time.



#include <stdint.h>
#include <stdbool.h>
#include <stdio.h>
#include <math.h>
#include "stdlib.h"
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

#ifdef DEBUG
void
__error__(char *pcFilename, uint32_t ui32Line)
{
    while(1);
}
#endif

#define NUM_SSI_DATA 8

uint32_t STEP_SIZE = 1000;
uint32_t low_bound = 10000;
uint32_t high_bound = 2000000;
uint32_t f_reg;
uint32_t i;
uint32_t frequencyRegister[2][1000];
uint32_t rows;
uint32_t ADCValue;
uint32_t lowVoltage;
uint32_t midVoltage;
uint32_t highVoltage;

void
PeriphEnable(){
    SysCtlPeripheralEnable(SYSCTL_PERIPH_SSI0);
    SysCtlPeripheralEnable(SYSCTL_PERIPH_SSI1);
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOA);
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOD);
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOC);
}

void
PinConfig(){
    GPIOPinConfigure(GPIO_PA2_SSI0CLK);
    GPIOPinConfigure(GPIO_PA3_SSI0FSS);
    GPIOPinConfigure(GPIO_PA5_SSI0TX);
    GPIOPinConfigure(GPIO_PD0_SSI1CLK);
    GPIOPinConfigure(GPIO_PD1_SSI1FSS);
    GPIOPinConfigure(GPIO_PD3_SSI1TX);
    GPIOPinTypeGPIOInput(GPIO_PORTC_BASE, GPIO_PIN_4);
    GPIOPinTypeGPIOInput(GPIO_PORTC_BASE, GPIO_PIN_5);
    GPIOPinTypeGPIOInput(GPIO_PORTC_BASE, GPIO_PIN_6);
    GPIOPinTypeSSI(GPIO_PORTA_BASE, GPIO_PIN_5|GPIO_PIN_3|GPIO_PIN_2);
    GPIOPinTypeSSI(GPIO_PORTD_BASE, GPIO_PIN_3|GPIO_PIN_1|GPIO_PIN_0);

}

void
DataSend(uint32_t data1, uint32_t data2, uint32_t data3){
    //sends the pulse numbers from the pulse counter, through UART, to the computer
}

void
FreqRegBits(){
    uint32_t rows = (int)((high_bound - low_bound)/STEP_SIZE);
    uint32_t reg;
    for(i = 0; i < rows; i++){
        reg = (int)((low_bound + (STEP_SIZE * i)) * (5.368709));
        //reg = (int)(((low_bound + (STEP_SIZE * i)) * pow(2,28))/50000000);
        frequencyRegister[0][i] = (int)((reg % 16384) + 16384);
        frequencyRegister[1][i] = (int)((reg / (2*2*2*2*2*2*2*2*2*2*2*2*2*2)) + 16384);
    }

}

int
ADCReader(){
    //reads the "peak" value from the signal response to determine when to cut off the pulse counting
    uint32_t ammount; //the cut off value
    uint32_t value; //the value given by the ADC
    if(value > ammount){
        return 0;
    }
    else if(value <= ammount){
        return 1;
    }
}

int
PulseCounter(){
    highVoltage = 0;
    midVoltage = 0;
    lowVoltage = 0;
    while(ADCReader() == 0){
        uint32_t low = GPIOPinRead(GPIO_PORTC_BASE, GPIO_PIN_4);
        uint32_t mid = GPIOPinRead(GPIO_PORTC_BASE, GPIO_PIN_5);
        uint32_t high = GPIOPinRead(GPIO_PORTC_BASE, GPIO_PIN_6);
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
    DataSend(highVoltage, midVoltage, lowVoltage);
    return 1;
}

void
SineWavePulse(){
    uint32_t rows = (int)((high_bound - low_bound)/STEP_SIZE);
    //highest frequency that can be in the first 14 bits of the frequency register is 976.5 Hz which is below the predetermined start
    //frequency of 1000 Hz. The lower 14 bits will still be programmed in but there will be logic to determine if they're needed or not.
    //The chip writes LSB in the first write and then MSB in the second write
    for(i = 0; i < rows; i++){

        uint32_t j = 0;
        while(j < 1){
            //SSIDataPut(SSI0_BASE, i);
            SSIDataPut(SSI0_BASE, 0x2100);
            SSIDataPut(SSI0_BASE, frequencyRegister[1][i]);
            SSIDataPut(SSI0_BASE, frequencyRegister[0][i]);
            SSIDataPut(SSI0_BASE, 0xC000);
            SSIDataPut(SSI0_BASE, 0x2000);
            while(SSIBusy(SSI0_BASE))
                {
                }
            SysCtlDelay(160000);
            SSIDataPut(SSI0_BASE, 0b00000100000000);
            j = PulseCounter();

        }
    }
}

uint32_t voltage1 = 0;
uint32_t voltage2 = 1;
uint32_t voltage3 = 3;
uint32_t voltage4 = 4;
uint32_t refVoltage = 5;
uint32_t voltCode[4];

void
voltCodeGenerator(uint32_t voltage1, uint32_t voltage2, uint32_t voltage3, uint32_t voltage4, uint32_t refVoltage){

    voltCode[0] = (int)((voltage1 * 256)/refVoltage);
    voltCode[1] = (int)((voltage2 * 256)/refVoltage);
    voltCode[2] = (int)((voltage3 * 256)/refVoltage);
    voltCode[3] = (int)((voltage4 * 256)/refVoltage);

}

void
DCVoltage(){
    uint32_t location1 = (int)(pow(2, 0));
    uint32_t location2 = (int)(pow(2, 9));
    uint32_t location3 = (int)(pow(2, 10));
    uint32_t location4 = (int)(pow(2, 9) + pow(2, 10));
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

//The SSI communication code was grabbed from an engineering project from Purdue. The link to the paper is
//https://engineering.purdue.edu/ece477/Archive/2014/Spring/S14-Grp1/docs/software/LM4F-LaunchPad-11%20-%20SPI.pdf
//The bitwise reverse function was taken out because it is not needed.

int main(void){

    PeriphEnable();
    PinConfig();
    FreqRegBits();
    SysCtlClockSet(SYSCTL_SYSDIV_1 | SYSCTL_USE_PLL | SYSCTL_OSC_MAIN | SYSCTL_XTAL_16MHZ);
    SSIConfigSetExpClk(SSI0_BASE,SysCtlClockGet(),SSI_FRF_MOTO_MODE_0,SSI_MODE_MASTER,10000,16);
    SSIConfigSetExpClk(SSI1_BASE,SysCtlClockGet(),SSI_FRF_MOTO_MODE_0,SSI_MODE_MASTER,10000,11);
    SSIEnable(SSI0_BASE);
    SSIEnable(SSI1_BASE);
    voltCodeGenerator(voltage1, voltage2, voltage3, voltage4, refVoltage);
    DCVoltage();
    SysCtlDelay(100000);
    SineWavePulse();

}
