/*
 * File:   i2c.c
 * Author: invlko-015
 *
 * Created on 14 June, 2024, 12:22 PM
 */


#include <xc.h>
#include "ADC.h"

//int I2C_Bus_Is_Idle(void) {
//    // Check if the I2C bus is idle
//    if ((SSP1STATbits.R_W == 0) && (SSP1CON2 & 0x1F) == 0) {
//        return 1; // I2C bus is idle
//    } else {
//        return 0; // I2C bus is busy
//    }
//}
    
void I2C_Master_Init(const unsigned long c) {
        UART_Send_String("I2c init\n");
        // sendSerialData("\r\ni2cIntitalizeSuccess\r\n",24);
        SSP1CON1 = 0x00;

        TRISCbits.TRISC0 = 1; // SCL
        TRISCbits.TRISC1 = 1; // SDA

        ANSELCbits.ANSC0 = 0;
        ANSELCbits.ANSC1 = 0;

        ODCONCbits.ODCC0 = 1;
        ODCONCbits.ODCC1 = 1;

        PPSLOCKbits.PPSLOCKED = 0; // PPS is unlocked
        SSP1DATPPS = 0x11;                             
        SSP1CLKPPS = 0x10;                      // assign RC0 (SCL) to SCL input
// assign RC1 (SDA) to SDA input
        RC0PPS = 0x14;                          // Assign SCL to RC0
        RC1PPS = 0x15;                          // Assign SDA to RC1
        PPSLOCKbits.PPSLOCKED = 1; // PPS is locked

        SSP1STATbits.CKE = 0;
        SSP1STATbits.SMP = 1;
        SSP1ADD = (unsigned char)((_XTAL_FREQ / (4 * c)) - 1); // Set baud rate for I2C

        SSP1CON1bits.SSPM = 0b1000; // Set to hardware master mode
        SSP1CON1bits.SSPEN = 1;

        SSP1CON3 = 0x00;
        SSP1CON3bits.PCIE = 1; // Enable STOP Interrupt
        SSP1CON3bits.SCIE = 1; // Enable START Interrupts  
}


void I2C_Start(void) {
    // Generate a Start condition
   
    SSP1CON2bits.SEN = 1; // Initiate Start condition
      UART_Send_String("I2c start\n");
    while (SSP1CON2bits.SEN); // Wait for Start condition to be completed
   
}

//void I2C_Restart(void) {
//    // Generate a Restart condition
//    SSP1CON2bits.RSEN = 1; // Initiate Repeated Start condition
//    while (SSP1CON2bits.RSEN); // Wait for Repeated Start condition to be completed
//}

void I2C_Stop(void) {
    // Generate a Stop condition
    UART_Send_String("i2c stop\n");
    SSP1CON2bits.PEN = 1; // Initiate Stop condition
    while (SSP1CON2bits.PEN); // Wait for Stop condition to be completed
}

char I2C_Write(unsigned char data) {
    UART_Send_String("write i2c\n");
    SSP1BUF = data; // Load data into the buffer
    while (!PIR3bits.SSP1IF); // Wait until the transmission is complete
    PIR3bits.SSP1IF = 0; // Clear the interrupt flag

    if (SSP1CON2bits.ACKSTAT) {
        return 0; // ACK not received
    } else {
        return 1; // ACK received
    }
}

unsigned char I2C_Read(unsigned char ack) {
    UART_Send_String("read i2c\n");
    SSP1CON2bits.RCEN = 1; // Enable receive mode
    while(!SSP1STATbits.BF); // Wait until data is received
    unsigned char data = SSP1BUF; // Read data from SSPBUF register
    if(ack) {
        SSP1CON2bits.ACKDT = 0; // Acknowledge data
    } else {
        SSP1CON2bits.ACKDT = 1; // Not acknowledge data
    }
    SSP1CON2bits.ACKEN = 1; // Send ACK/NACK bit
    while(SSP1CON2bits.ACKEN); // Wait for ACK/NACK bit to be sent
    return data;
}

//void I2C_Acknowledge(void) {
//    SSP1CON2bits.ACKDT = 0; // ACK
//    SSP1CON2bits.ACKEN = 1; // Send ACK
//    while (SSP1CON2bits.ACKEN); // Wait for ACK to be sent
//}
//
//void I2C_Send_NACK(void) {
//    SSP1CON2bits.ACKDT = 1; // NACK
//    SSP1CON2bits.ACKEN = 1; // Send NACK
//    while (SSP1CON2bits.ACKEN); // Wait for NACK to be sent
//}