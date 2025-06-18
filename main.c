#include <xc.h>
#include <stdint.h>

#define _XTAL_FREQ 4000000  // 4 MHz crystal oscillator

// CONFIGURATION BITS
#pragma config FOSC = HS     // High-Speed Oscillator
#pragma config WDTE = OFF    // Watchdog Timer Disabled
#pragma config PWRTE = ON    // Power-up Timer Enabled
#pragma config BOREN = ON    // Brown-out Reset Enabled
#pragma config LVP = OFF     // Low Voltage Programming Disabled
#pragma config CPD = OFF     // EEPROM Data Memory Code Protection Disabled
#pragma config WRT = OFF     // Flash Program Memory Write Protection Disabled
#pragma config CP = OFF      // Flash Program Memory Code Protection Disabled

// LCD I2C Settings
#define LCD_ADDR 0x27        // I2C address of LCD (commonly 0x27 or 0x3F)
#define LCD_BACKLIGHT 0x08   // Backlight ON bit

// I2C Function Prototypes
void I2C_Init(void);
void I2C_Start(void);
void I2C_Stop(void);
void I2C_Write(uint8_t data);
uint8_t I2C_Check_Address(uint8_t address);

// LCD Function Prototypes
void LCD_Send_Nibble(uint8_t nibble, uint8_t rs);
void LCD_Send_Byte(uint8_t data, uint8_t rs);
void LCD_Cmd(uint8_t cmd);
void LCD_Write_Char(char c);
void LCD_Write_String(const char *str);
void LCD_Init(void);
void LCD_Set_Cursor(uint8_t row, uint8_t col);
void LCD_Clear(void);

void main(void) {
    // Disable analog functions on PORTA
    ADCON1 = 0x06;

    // Initialize I2C communication
    I2C_Init();

    // Check if LCD is responding at specified I2C address
    if(!I2C_Check_Address(LCD_ADDR)) {
        // If not, blink all PORTD pins continuously
        TRISD = 0x00;
        while(1) {
            PORTD = 0xFF;
            __delay_ms(500);
            PORTD = 0x00;
            __delay_ms(500);
        }
    }

    // Initialize LCD display
    LCD_Init();

    // Display static text on LCD
    LCD_Set_Cursor(1, 1);
    LCD_Write_String("Hello World!");
    LCD_Set_Cursor(2, 1);
    LCD_Write_String("PIC16F877A I2C");

    while(1) {
        // Optionally blink display to show activity
        LCD_Cmd(0x08);  // Turn display off (backlight off)
        __delay_ms(100);
        LCD_Cmd(0x0C);  // Turn display on (backlight on)
        __delay_ms(1000);
    }
}

// I2C initialization in Master mode
void I2C_Init(void) {
    TRISC3 = 1;  // Set SCL (RC3) as input
    TRISC4 = 1;  // Set SDA (RC4) as input
    SSPCON = 0x28;   // Enable MSSP module in I2C Master mode
    SSPCON2 = 0x00;
    SSPADD = 9;      // Set clock frequency to 100kHz (for 4 MHz Fosc)
    SSPSTAT = 0x00;
}

// Check if I2C device responds at given address
uint8_t I2C_Check_Address(uint8_t address) {
    I2C_Start();
    SSPBUF = (address << 1);  // Send address with write bit (R/W = 0)
    while(SSPSTATbits.BF);    // Wait until buffer is empty
    I2C_Stop();
    return !SSPCON2bits.ACKSTAT;  // Return 1 if ACK received, 0 if NACK
}

// Generate I2C Start condition
void I2C_Start(void) {
    SEN = 1;
    while(SEN);  // Wait until Start condition is complete
}

// Generate I2C Stop condition
void I2C_Stop(void) {
    PEN = 1;
    while(PEN);  // Wait until Stop condition is complete
}

// Send one byte over I2C
void I2C_Write(uint8_t data) {
    SSPBUF = data;
    while(SSPSTATbits.BF);       // Wait until data is transferred
    while(SSPCON2bits.ACKSTAT);  // Wait for ACK from slave
}

// Send 4-bit nibble to LCD with RS control
void LCD_Send_Nibble(uint8_t nibble, uint8_t rs) {
    // Data format: [D7 D6 D5 D4 BL EN RW RS]
    uint8_t data = nibble | LCD_BACKLIGHT | (rs ? 0x01 : 0x00);

    // Pulse EN bit to latch data
    I2C_Write(data | 0x04);  // EN = 1
    __delay_us(1);
    I2C_Write(data & ~0x04); // EN = 0
    __delay_us(50);          // Wait for instruction to complete
}

// Send full 8-bit command/data to LCD
void LCD_Send_Byte(uint8_t data, uint8_t rs) {
    LCD_Send_Nibble(data >> 4, rs);  // Send upper nibble
    LCD_Send_Nibble(data & 0x0F, rs); // Send lower nibble
}

// Send command to LCD
void LCD_Cmd(uint8_t cmd) {
    LCD_Send_Byte(cmd, 0);
    if(cmd == 0x01 || cmd == 0x02) __delay_ms(2); // Clear or Home command requires delay
}

// Write single character to LCD
void LCD_Write_Char(char c) {
    LCD_Send_Byte(c, 1);
}

// Write null-terminated string to LCD
void LCD_Write_String(const char *str) {
    while(*str) LCD_Write_Char(*str++);
}

// Set LCD cursor to specified row and column
void LCD_Set_Cursor(uint8_t row, uint8_t col) {
    col--;
    uint8_t address = (row == 1) ? 0x80 : 0xC0;
    LCD_Cmd(address + col);
}

// Clear LCD display
void LCD_Clear(void) {
    LCD_Cmd(0x01);
    __delay_ms(2);
}

// LCD initialization routine
void LCD_Init(void) {
    __delay_ms(50);  // Wait for LCD to power up

    I2C_Start();
    I2C_Write(LCD_ADDR << 1);  // Send LCD address with write flag

    // Initialization sequence as per HD44780 datasheet
    LCD_Send_Nibble(0x03, 0);
    __delay_ms(5);
    LCD_Send_Nibble(0x03, 0);
    __delay_us(150);
    LCD_Send_Nibble(0x03, 0);
    LCD_Send_Nibble(0x02, 0);  // Set 4-bit mode

    LCD_Cmd(0x28);  // Function set: 4-bit, 2 lines, 5x8 dots
    __delay_us(50);
    LCD_Cmd(0x0C);  // Display ON, cursor OFF, blink OFF
    __delay_us(50);
    LCD_Cmd(0x06);  // Entry mode: cursor moves right, no shift
    __delay_us(50);
    LCD_Clear();    // Clear display

    I2C_Stop();
}