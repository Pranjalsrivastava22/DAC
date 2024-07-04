
#include <stdio.h>
#include "config.h"

// Your UART functions and other code here...


void UART_Init(uint32_t baudrate) {
// Configure USART for asynchronous 8-bit transmission, high baud rate
    ANSELB = 0x00;
    TRISBbits.TRISB6 = 1; // TX pin as input
    TRISBbits.TRISB7 = 1; // RX pin as input

    baudrate *= 4; // Adjust for BRGH setting

    uint16_t data = (uint16_t)((_XTAL_FREQ / baudrate) - 1);

    SPBRGH = (data & 0xFF00) >> 8;
    SPBRGL = data & 0x00FF;

    PPSLOCK = 0;
    RXPPS = 0x0F;   // RX pin mapping
    RB6PPS = 0x10;  // TX pin mapping
    PPSLOCK = 1;

    BAUD1CONbits.BRG16 = 1; // 16-bit baud rate generator
    RC1STAbits.SPEN = 1;    // Enable serial port
    TX1STAbits.SYNC = 0;    // Asynchronous mode
    TX1STAbits.BRGH = 1;    // High baud rate select bit
    TX1STAbits.TXEN = 1;    // Enable transmission
    RC1STAbits.CREN = 1;    // Enable continuous receive
}

void UART_Write(char data) {
    while (!TX1STAbits.TRMT); // Wait until transmit shift register is empty
    TX1REG = data; // Transmit data
}

void UART_Write_Text(const char* text) {
    while (*text != '\0') {
        UART_Write(*text++);
    }
}

void UART_Write_Float(float value) {
    char buffer[10];
    sprintf(buffer, "%.2f", value); // Convert float to string with 2 decimal places
    UART_Write_Text(buffer);
}
