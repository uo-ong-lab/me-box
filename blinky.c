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

uint32_t STEP_SIZE = 100;
uint32_t low_bound = 1000;
uint32_t high_bound = 100000;
uint32_t f_reg;
uint32_t i;
uint32_t rows;

void
PeriphEnable(){
    SysCtlPeripheralEnable(SYSCTL_PERIPH_SSI0);
    SysCtlPeripheralEnable(SYSCTL_PERIPH_SSI1);
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOA);
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOD);
}

void
PinConfig(){
    GPIOPinConfigure(GPIO_PA2_SSI0CLK);
    GPIOPinConfigure(GPIO_PA3_SSI0FSS);
    GPIOPinConfigure(GPIO_PA5_SSI0TX);
    GPIOPinConfigure(GPIO_PD0_SSI1CLK);
    GPIOPinConfigure(GPIO_PD1_SSI1FSS);
    GPIOPinConfigure(GPIO_PD3_SSI1TX);
    GPIOPinTypeSSI(GPIO_PORTA_BASE, GPIO_PIN_5|GPIO_PIN_3|GPIO_PIN_2);
    GPIOPinTypeSSI(GPIO_PORTD_BASE, GPIO_PIN_3|GPIO_PIN_1|GPIO_PIN_0);

}

int
FreqReg(uint32_t STEP_SIZE, uint32_t low_bound, uint32_t i){

    f_reg = (int)(((low_bound + (STEP_SIZE * i)) * (2*2*2*2*2*2*2*2*2*2*2*2*2*2*2*2*2*2*2*2*2*2*2*2*2*2*2*2)) / 16000000);
    //used the 2*2... to get a non floating point number that pow(2, 28) gave. Keeps the processing power usage down
    return f_reg;
}

int
MSB(uint32_t STEP_SIZE, uint32_t low_bound, uint32_t i){

    return (int)((FreqReg(STEP_SIZE, low_bound, i) * (2*2*2*2*2*2*2*2*2*2*2*2*2*2)) + 16384);
    //MSB
    //FREQ_REG_BITS[i][1] = 16384 + (FreqReg(STEP_SIZE, low_bound, i) / 2*2*2*2*2*2*2*2*2*2*2*2*2*2) is getting the top 14 bits
    //of the frequency register by excluding the bottom 14 by dividing by 2^14. This math was tested beforehand and it works out well
}

int
LSB(uint32_t STEP_SIZE, uint32_t low_bound, uint32_t i){

    return ((FreqReg(STEP_SIZE, low_bound, i) % 16384) + 16384); // 0100000000000000 -> 16384
    // LSB
}

int*
FreqRegBits(){
    uint32_t rows = (int)((high_bound - low_bound)/STEP_SIZE);
    uint32_t frequencyRegister[2][rows];
    uint32_t reg;
    for(i = 0; i < rows; i++){
        reg = (int)(((low_bound + (STEP_SIZE * i)) * pow(2,28))/16000000);
        frequencyRegister[0][i] = (int)((reg % 16384) + 16384);
        frequencyRegister[1][i] = (int)((reg / pow(2,14)) + 16384);
    }
    return frequencyRegister;

}

void
SineWavePulse(uint32_t freqRegister[][]){
    uint32_t rows = (int)((high_bound - low_bound)/STEP_SIZE);
    /*uint32_t FREQ_REG_BITS[ROWS][2];
    //uint32_t row;
    //for(row = 0; row < ROWS; row++){
    //    FREQ_REG_BITS[row][0] = LSB(STEP_SIZE, low_bound, i);
    //    FREQ_REG_BITS[row][1] = MSB(STEP_SIZE, low_bound, i);
    //}
    //highest frequency that can be in the first 14 bits of the frequency register is 976.5 Hz which is below the predetermined start
    //frequency of 1000 Hz. The lower 14 bits will still be programmed in but there will be logic to determine if they're needed or not.
    //The chip writes LSB in the first write and then MSB in the second write
    //}
    //Generates the wave and keeps it going for a set duration and then turns it off*/
    for(i = 0; i < rows; i++){

        uint32_t MostSB = freqRegister[1][i];
        uint32_t LeastSB = freqRegister[0][i];
        uint32_t j = 0;
        while(j < 1){
            //SSIDataPut(SSI0_BASE, i);
            SSIDataPut(SSI0_BASE, 0x2100);
            //SSIDataPut(SSI0_BASE, LeastSB);
            SSIDataPut(SSI0_BASE, MostSB);
            //SSIDataPut(SSI0_BASE, 0xC000);
            //SSIDataPut(SSI0_BASE, 0x2000);
            while(SSIBusy(SSI0_BASE))
                {
                }
            SysCtlDelay(16000);

            j = 1;
        }
    }
}

uint32_t duration;
uint32_t voltage1 = 0;
uint32_t voltage2 = 1;
uint32_t voltage3 = 3;
uint32_t refVoltage = 5;
uint32_t voltCode[3];

void
voltCodeGenerator(uint32_t voltage1, uint32_t voltage2, uint32_t voltage3, uint32_t refVoltage){

    voltCode[0] = (int)((voltage1 * 256)/refVoltage);
    voltCode[1] = (int)((voltage2 * 256)/refVoltage);
    voltCode[2] = (int)((voltage3 * 256)/refVoltage);

    //return voltCode;
}

void
DCVoltage(){
    uint32_t voltbits1 = voltCode[0];
    uint32_t voltbits2 = voltCode[1];
    uint32_t voltbits3 = voltCode[2];
    uint32_t k = 0;
    GPIOPinWrite(GPIO_PORTD_BASE, GPIO_PIN_6, 0b1);
    while(k < 1){
        SSIDataPut(SSI1_BASE, voltbits1);
        SSIDataPut(SSI1_BASE, voltbits2);
        SSIDataPut(SSI1_BASE, voltbits3);
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
    SysCtlClockSet(SYSCTL_SYSDIV_1 | SYSCTL_USE_PLL | SYSCTL_OSC_MAIN | SYSCTL_XTAL_16MHZ);
    SSIConfigSetExpClk(SSI0_BASE,SysCtlClockGet(),SSI_FRF_MOTO_MODE_0,SSI_MODE_MASTER,10000,16);
    SSIConfigSetExpClk(SSI1_BASE,SysCtlClockGet(),SSI_FRF_MOTO_MODE_0,SSI_MODE_MASTER,10000,11);
    SSIEnable(SSI0_BASE);
    SSIEnable(SSI1_BASE);
    voltCodeGenerator(voltage1, voltage2, voltage3, refVoltage);
    DCVoltage();
    SysCtlDelay(100000);
    SineWavePulse(FreqRegBits());

}
