/*TRANSMITTER
* File: TxModule.c
* Author: nguye
*
* Created on February 17, 2021, 8:05 PM
*/
#include <xc.h>
#include <stdio.h>
//#include <pic18f45k20.h>
#pragma config FOSC = INTIO67 // Oscillator Selection bits (Internal oscillator block, port function on RA6 and RA7)
#pragma config WDTEN = OFF // Watchdog Timer Enable bit (WDT is controlled by SWDTEN bit of the WDTCON register)
#pragma config LVP = OFF // Single-Supply ICSP Enable bit (Single-Supply ICSP disabled)
#pragma config MCLRE = ON       // MCLR Pin Enable bit (MCLR pin enabled; RE3 input pin disabled)

#define _XTAL_FREQ 16000000

char ADCreadX();
char ADCreadY();
void UARTTx(char[]);
void Init(void);
void main()
{
    int z;
    Init(); // call Init function
    char data[]={0xC8,0x64,0xC8,0x64};
    char a[3];
    char joyX, joyY;
    a[0] = 0xFF;
    while(1)
    {
        joyX=ADCreadX();// store value X axis left-right
        joyY=ADCreadY(); // store value Y axis forward-reserve
        
        if(joyX>0xC8) // turn left
            a[1]=data[0]; // store value 0xA1
        else if(joyX<0x64) // turn right
            a[1]=data[1]; //store value 0xA2
        else
            a[1] = 0x96;
        
        if(joyY>0xC8) //move reserve
            a[2]=data[2]; //store value 0xA3
        else if(joyY<0x64) // move forward
            a[2]=data[3]; // store value 0xA4
        else
            a[2] = 0x96;
        UARTTx(a); // call UARTTx function
    }
}
char ADCreadX()
{
    char ADjoyXPOS; // defined a variable to hold ADRESH value
    ADCON0 = 0x00; // enable ANS0 X axis
    ADON = 1; // turn ON device
    GO_DONE = 1; // start convert
    while(GO_DONE); // wait for GO_DONE goes LOW then completed convert
    ADjoyXPOS = ADRESH;
    ADON = 0; // turn OFF device
    return ADjoyXPOS;
}
char ADCreadY()
{
    char ADjoyYPOS; // defined a variable to hold ADRESH value
    ADCON0 = 0x1C; // enable ANS7 Y axis
    ADON = 1; // turn ON device
    GO_DONE = 1; // start convert
    while(GO_DONE); // wait for GO_DONE goes LOW then completed convert
    ADjoyYPOS = ADRESH;
    ADON = 0; // turn OFF device
    return ADjoyYPOS;
}
void UARTTx(char a[])
{
    int i;
    for(i=0;i<3;i++)
    {
        while(!TXIF); // keep transmitting even TXREG has data
        TXREG = a[i]; // store data  
        __delay_ms(10);
    }
        __delay_ms(40);
}

void Init(void)
{
    // set-up oscillator
    OSCTUNE = 0; // used to tune the freq of the internal oscillator
    OSCCON = 0x7E; // set default 16 MHz
    // set-up baud rate and TX-RX device
    BRGH = 1; //high speed baud rate
    SYNC = 0; // asynchronous mode
    BRG16 = 0 ; // 8-bit register baud generator is used
    SPBRG = 103; // set 9600 baud rate
    INTCON = 0x00; //interrupt disabled
    RX9 = 0; //RCSTA set 8 bit reception
    TX9 = 0; // TXSTA set 8 bit tranmission
    SPEN = 1; // RCSTA set port enable
    CREN = 0; // RCSTA set continuous receive disable
    TRISC7=1; // Receiver pin
    TRISC6= 0; // Transmitter pin
    // set-up input JOYSTICK
    ADCON1 = 0x00; // use reference voltage
    ADCON2 = 0x1A; // RIGHT justifed, 6Tad, fosc/32
    TRISA0 = 1; // select X axis input
    ANS0 = 1; // assigned X axis analog
    TRISE2 = 1; // select Y axis input
    ANS7 = 1; // assigned Y axis analog
    TXEN = 1;
}