#ifndef DAC_H
#define DAC_H

#include <xc.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif /* __cplusplus */

#define _XTAL_FREQ 32000000  // Define your oscillator frequency

#define I2C_SCL_TRIS TRISCbits.TRISC3
#define I2C_SDA_TRIS TRISCbits.TRISC4

#define DAC_I2C_ADDRESS 0x0C  // Replace with the actual DAC I2C address
#define ADC_I2C_ADDRESS 0x21  // Replace with the actual ADC I2C address
#define I2C_SPEED 400000  // Define I2C speed in Hz

void I2C_Init(void);
void I2C_Wait(void);
void I2C_Start(void);
void I2C_Stop(void);
void I2C_Write(unsigned char data);
unsigned char I2C_Read(unsigned char ack);
void DAC_SetValue(float voltage);
uint16_t ADC_ReadValue(void);
void UART_Init(uint32_t baudrate);
void UART_Write(char data);
void UART_Write_Text(const char* text);
void UART_Write_Float(float value);
uint16_t ADC_ReadValue(void);
#ifdef __cplusplus
}
#endif /* __cplusplus */

#endif /* DAC_H */
