/*
* File: RxModule.c
* Author: nguye
*
* Created on February 17, 2021, 9:04 PM
*/
#include <xc.h>
#include <stdio.h>
#include <pic18f45k20.h>
#pragma config FOSC = INTIO67 // Oscillator Selection bits (Internal oscillator block, port function on RA6 and RA7)
#pragma config FCMEN = OFF // Fail-Safe Clock Monitor Enable bit (Fail-Safe Clock Monitor disabled)
#pragma config IESO = OFF // Internal/External Oscillator Switchover bit (Oscillator Switchover mode disabled)
#pragma config LVP = OFF // Single-Supply ICSP Enable bit (Single-Supply ICSP disabled)
#pragma config WDTEN = OFF // Watchdog Timer Enable bit (WDT is controlled by SWDTEN bit of the WDTCON register)
#define _XTAL_FREQ 16000000 // frequency
void UARTRx(char[]);
void Init(void);
void GetSignal();
int ADCreadSensor();
void GetControl();
void main()
{
    Init(); // call init function
    while(1)
    {

        GetControl(); // call Getcontrol function           
        GetSignal(); // call Getsignal function   
    }
}
void GetControl()
{
    char b[3];
    char loop=0;
        // control LED
        UARTRx(b); // call receiver function
        if(b[0] == 0xFF)
        {
           
            if(b[1]<=0x64) // b=0xA1
            {
                RD7=1; // LEFT LED on
                RD6=0;
                //control Servo
                for(loop;loop<1;loop++)
                {
                    RA7=1;
                    __delay_ms(1.3);
                    RA7=0;
                    __delay_ms(18.7);
                }
                /*if(TMR0IF)
                {
                    TMR0IF=0;   // reset interrupt flag
                    TMR0L=0x44; // calculate 1.7ms 
                    RA7=!RA7; // rotation servo
                    __delay_ms(20);             
                }*/ 
            }
            else if(b[1]>=0xC8) //b=0xA2
            {
                RD7=0;
                RD6=1; //RIGHT LED on
                //control Servo
                for(loop;loop<1;loop++)
                {
                    RA7=1;
                    __delay_ms(1.7);
                    RA7=0;
                    __delay_ms(18.3);
                }
                /*if(TMR0IF)
                {
                    TMR0IF=0; // reset interrupt flag
                    TMR0L=0x5D; // calculate 1.3ms 
                    RA7=!RA7; // rotation servo
                    __delay_ms(20);                
                }*/
            }
            else
            {
                RD6=0;
                RD7=0;
                //control Servo
                for(loop;loop<1;loop++)
                {
                    RA7=1;
                    __delay_ms(1.5);
                    RA7=0;
                    __delay_ms(18.5);
                }                
           
            }
            if(b[2]<=0x64) //b=0xA3
            {
                RD5=0; // HEAd LED ON
                RD4=1;
                RC0=1;
                RC1=0;                
                /*if(TMR0IF)
                {
                    TMR0IF=0; // reset interrupt flag
                    TMR0L=0x06; // calculate 1.3ms 
                    RC1=!RC1; // rotation servo
                    __delay_ms(20);                
                } */
                //RC0=0;
            }
            else if (b[2]>=0xC8) // b=0xA4
            {
                RD5=1;
                RD4=0; // Tail LED on
                RC0=1;
                RC1=1;                
            }
            else
            { 
                RD5=0;
                RD4=0;
                RC0=0;
                RC1=0;
            }
        }
    }
//}
void GetSignal()
{
    int ADCValue; // create space to hold value
    float GetVolt, LSB; // declare variables
    LSB=3.5/1024;   // calculate LSB
    ADCValue = ADCreadSensor(); // calculate ADCvalue
    GetVolt= ADCValue * LSB; // calculate voltage
    if(GetVolt<=0.06) // if statement
    {
        RD1=1;  // less than 0.07 LED ON
        RC0=0;
    }
    else
        RD1=0;  // bigger 0.07 LED OFF
    
}
void UARTRx(char b[])
{
    int i;
    for(i=0;i<3;i++)
    {
        if(OERR || FERR) // checking error 
        {
            SPEN = 0;   // re-enable Rx device
            CREN = 0;   // re-set Rx device
            CREN = 1;   // re-set Rx device
            SPEN = 1;   // re-enable Rx device
        }
        while(!RCIF); // keep looping while the buffer is empty
        b[i]=RCREG ; // store value
    }       
    i = 0;
}
int ADCreadSensor()
{
    int GetSensorVolt; // create space to hold sensor value
    ADON=1; // turn ON device
    GO_DONE=1; // start convert
    while(GO_DONE); // wait GO_DONE
    GetSensorVolt= (ADRESH*256)+ADRESL; // store value
    ADON=0; // turn OFF device
    return GetSensorVolt; // pass value
}

void Init(void)
{
    // set-up oscillator
    OSCTUNE = 0; // used to tune the freq of the internal oscillator
    OSCCON = 0x7E; // set default 16 Mhz
    //set-up timer0
    T0CON= 0xC4; // enable timer, 8-bit, internal , low to high, assigned prescale, 32-pre
    TMR0L=0x44;
    // set-up baud rate and TX-RX device
    BRGH = 1; //high speed baud rate
    SYNC = 0; // asynchronous mode
    BRG16 = 0 ; // 8-bit register baud generator is used
    SPBRG = 103; // set 9600 baud rate
    INTCON = 0x00; //interrput disabled
    RX9 = 0; //RCSTA set 8 bit reception
    TX9 = 0; // TXSTA set 8 bit tranmision
    SPEN = 1; // RCSTA set port enable
    CREN = 1; // RCSTA set continuous receive disable
    TRISC7=1; // Receiver pin
    TRISC6= 0; // Transmitter pin
    //set up ADC
    ADCON0= 0x00; // enable ANS0 sensor
    ADCON1= 0x00;   // reference voltage
    ADCON2= 0xA2;    // right justifed, 6 Tad, focs/32
    // set-up output LED
    TRISD7 = 0; //LEFT
    RD7 = 0;
    TRISD6 = 0; // RIGHT
    RD6 = 0;
    TRISD5 = 0; //BACK
    RD5 = 0;
    TRISD4 = 0; //BACK
    RD4 = 0;
    //set-up motor
    TRISC0=0; // output E1
    RC0=0;
    TRISC1=0; // output M1
    RC1=0;
    //set-up servo
    TRISA7=0; // output Servo
    RA7=0;
    //set-up sensor
    TRISA0 = 1; // select analog sensor
    ANS0=1; // Analog
    TRISD1=0;   // LED sensor output
    RD1=0;      // assigned RD1 LOW
}