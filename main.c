#include <xc.h>
#include "config.h"

// I2C pin configuration (adjust as per your hardware)
#define I2C_SCL_TRIS TRISCbits.TRISC3
#define I2C_SDA_TRIS TRISCbits.TRISC4

void I2C_Init(void) {
        UART_Write_Text("I2c init\n");
        // sendSerialData("\r\ni2cIntitalizeSuccess\r\n",24);
        SSP1CON1 = 0x00;

        TRISCbits.TRISC0 = 1; // SCL
        TRISCbits.TRISC1 = 1; // SDA

        ANSELCbits.ANSC0 = 0;
        ANSELCbits.ANSC1 = 0;

        ODCONCbits.ODCC0 = 1;
        ODCONCbits.ODCC1 = 1;

        PPSLOCKbits.PPSLOCKED = 0; // PPS is unlocked
        SSP1DATPPS = 0x11;                      // assign RC0 (SCL) to SCL input       
        SSP1CLKPPS = 0x10;             // assign RC1 (SDA) to SDA input         
        
        RC0PPS = 0x14;                          // Assign SCL to RC0
        RC1PPS = 0x15;                          // Assign SDA to RC1
        PPSLOCKbits.PPSLOCKED = 1; // PPS is locked

        SSP1STATbits.CKE = 0;
        SSP1STATbits.SMP = 1;
        SSP1ADD = (unsigned char)((_XTAL_FREQ / (4 * I2C_SPEED)) - 1); // Set baud rate for I2C

        SSP1CON1bits.SSPM = 0b1000; // Set to hardware master mode
        SSP1CON1bits.SSPEN = 1;

        SSP1CON3 = 0x00;
        SSP1CON3bits.PCIE = 1; // Enable STOP Interrupt
        SSP1CON3bits.SCIE = 1; // Enable START Interrupts  
}

void I2C_Wait(void) {
    while ((SSP1CON2 & 0x1F) || (SSP1STAT & 0x04)); // Wait for Idle condition
}

void I2C_Start(void) {
    I2C_Wait();
    SSP1CON2bits.SEN = 1; // Initiate Start condition
}

void I2C_Stop(void) {
    I2C_Wait();
    SSP1CON2bits.PEN = 1; // Initiate Stop condition
}

void I2C_Write(unsigned char data) {
    I2C_Wait();
    SSP1BUF = data; // Write data to SSPBUF
}

unsigned char I2C_Read(unsigned char ack) {
    unsigned char temp;
    I2C_Wait();
    SSP1CON2bits.RCEN = 1; // Enable receive mode
    I2C_Wait();
    temp = SSP1BUF; // Read received data
    I2C_Wait();
    SSP1CON2bits.ACKDT = ack; // Acknowledge sequence
    SSP1CON2bits.ACKEN = 1; // Initiate acknowledge sequence
    return temp;
}


// DAC1CON0 and DAC1CON1 are registers specific to the DAC module.
// Adjust the settings according to your application's requirements.

void DAC1_Init(void) {
    // Configure DAC1CON0 register
    DAC1CON0bits.DAC1EN = 1; // Enable DAC
    DAC1CON0bits.DAC1OE1 = 1; // Enable DAC output on DAC1OUT1 pin
    DAC1CON0bits.DAC1OE2 = 1; // Enable DAC output on DAC1OUT2 pin
    DAC1CON0 = 0b0011; // Select VREF+ as the positive source

    // Configure DAC1CON1 register
    DAC1CON1bits.DAC1R = 0x1F; // Set the DAC voltage output (5-bit value, example: 0x1F for mid-scale)
}

void DAC_SetValue(float voltage) {
    
    uint16_t value = (uint16_t)((voltage / 5.0) * 4095); // Assuming 12-bit DAC and 5V reference
    I2C_Start();
    I2C_Write(DAC_I2C_ADDRESS << 1); // Address + Write
    I2C_Write((value >> 8) & 0xFF); // Upper byte
    I2C_Write(value & 0xFF); // Lower byte
    I2C_Stop();
    UART_Write_Text("dac set .....");
}

void ADC_Init(void) {
    // Configure ADCON0 register
    ADCON0bits.ADON = 1; // Enable ADC
    ADCON0bits.ADCS = 0b110; // ADC conversion clock select bits (FRC)
    ADCON0bits.ADGO = 0; // Ensure ADGO is clear
    
//    // Configure ADCON1 register
//    ADCON1bits.ADFM = 1; // Right justified result
//    ADCON1bits.ADPREF = 0b00; // VREF+ is connected to VDD

    // Configure ADCON2 register
    ADCON2bits.ADPSIS = 0; // Default value
    ADCON2bits.ADCRS = 0b000; // ADC result resolution select bits
    ADCON2bits.ADACLR = 0; // Default value
    ADCON2bits.ADMD = 0b000; // ADC operating mode
    
    // Configure ADCON3 register
    ADCON3bits.ADCALC = 0b000; // Default value
    ADCON3bits.ADSOI = 0; // Default value
    ADCON3bits.ADTMD = 0b000; // Default value

    // Configure ADC Clock source and reference
    ADCLK = 0x3F; // Default value
    ADREFbits.ADPREF = 0b00; // VREF+ is connected to VDD
    ADREFbits.ADNREF = 0; // VREF- is connected to VSS
    
    // Configure ADC Positive Channel
    ADPCH = 0b000000; // Select AN0 for positive channel
    
    // Enable ADC Module
    ADCON0bits.ADON = 1; // Turn on the ADC module
}

uint16_t ADC_ReadValue(void) {
    uint16_t result = 0;

    // Start ADC conversion
    ADCON0bits.ADGO = 1;
    while (ADCON0bits.ADGO); // Wait for conversion to complete

    // Read the result
    result = ((uint16_t)ADRESH << 8) | ADRESL;

    return result;
}


void main(void) {
    I2C_Init();
    UART_Init(115200);
    UART_Write_Text("ok......................... ");
    DAC1_Init();
    DAC_SetValue(3.3); // Set DAC to 3.3V
    ADC_ReadValue();
    while (1) {
        uint16_t adc_value = ADC_ReadValue();
        // Convert ADC value to voltage (assuming 12-bit ADC and 5V reference)
        float voltage = (adc_value / 4095.0) * 5.0;

        UART_Write_Text("ADC Value:  ");
        UART_Write_Float(voltage);

        __delay_ms(1000);
    }
}
