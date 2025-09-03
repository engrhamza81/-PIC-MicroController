#include <xc.h>
#include <stdint.h>
#include <stdio.h>
#include "pid.h" // Include your PID implementation

// CONFIGURATION BITS
#pragma config FEXTOSC = OFF         // External Oscillator disabled
#pragma config RSTOSC = HFINTOSC_64MHZ // HF Internal Oscillator, 64MHz
#pragma config MCLRE = EXTMCLR       // MCLR pin enabled
#pragma config LVP = OFF             // Low-Voltage Programming disabled
#define _XTAL_FREQ 64000000

// === I/O DEFINITIONS ===
#define HEATER_PIN LATCbits.LATC0
#define FAN_PIN    LATCbits.LATC1
#define TEMP_SENSOR_PIN PORTBbits.RB0

// === GLOBAL VARIABLES ===
float setpoint = 150.0;         // Desired temp in °C
float current_temp = 0.0;       // Current temp from sensor
float output = 0.0;             // PID result

// === FUNCTION PROTOTYPES ===
void init_system(void);
void init_uart(void);
void init_adc(void);
void init_pwm(void);
void set_pwm_duty_cycle(uint8_t duty_cycle);
void read_temperature(void);
void control_heater_fan(float output);
void send_data_to_uart(float temp);
float read_ds18b20(void);
void ds18b20_init(void);
uint8_t ds18b20_read_byte(void);
void ds18b20_write_byte(uint8_t byte);
void ds18b20_reset(void);

// === MAIN LOOP ===
void main(void) {
    init_system();
    PID_Init(2.0, 1.0, 1.0); // Example PID gains: P=2, I=1, D=1

    while (1) {
        read_temperature(); // Update current_temp
        output = PID_Compute(setpoint, current_temp); // Get PID output

        // Clamp output to -255 to +255
        if (output > 255) output = 255;
        if (output < -255) output = -255;

        control_heater_fan(output);
        send_data_to_uart(current_temp);
        __delay_ms(1000);
    }
}

// === INITIALIZATION ===
void init_system(void) {
    TRISC = 0b11111100;        // RC0/RC1 = output (heater/fan)
    LATC = 0x00;               // Turn off heater/fan initially
    TRISBbits.TRISB0 = 1;      // RB0 = input (sensor)
    ANSELB = 0x00;             // PORTB as digital
    init_uart();
    init_adc();
    init_pwm();
}

// === UART SETUP ===
void init_uart(void) {
    TXSTAbits.SYNC = 0;      // Async mode
    TXSTAbits.BRGH = 1;      // High speed
    SPBRG = 103;             // Baud rate 9600 for 64MHz (SPBRG = (_XTAL_FREQ / (16 * BaudRate)) - 1)
    RCSTAbits.SPEN = 1;      // Enable UART
    TXSTAbits.TXEN = 1;      // Enable transmitter
}

// === ADC (optional - not used for DS18B20) ===
void init_adc(void) {
    ADCON1 = 0b00000100;     // AN0 analog
    ADCON2 = 0b10111111;     // Right justify, 64TAD, FRC
    ADCON0bits.ADON = 1;
}

// === PWM INIT (CCP1) ===
void init_pwm(void) {
    PR2 = 255;               // 8-bit resolution
    CCP1CON = 0b00001100;    // PWM mode
    CCPR1L = 0;              // Initial duty
    T2CON = 0b00000101;      // Timer2 on, prescaler 1:4
}

// === TEMPERATURE SENSOR INTERFACE ===
void read_temperature(void) {
    current_temp = read_ds18b20(); // Read temperature from DS18B20
}

// === HEATER & FAN CONTROL BASED ON PID OUTPUT ===
void control_heater_fan(float output) {
    if (output >= 0) {
        HEATER_PIN = 1; // Turn on heater
        FAN_PIN = 0;    // Turn off fan
        set_pwm_duty_cycle((uint8_t)output);
    } else {
        HEATER_PIN = 0; // Turn off heater
        FAN_PIN = 1;    // Turn on fan
        set_pwm_duty_cycle((uint8_t)(-output));
    }
}

// === SET PWM DUTY CYCLE ===
void set_pwm_duty_cycle(uint8_t duty_cycle) {
    CCPR1L = duty_cycle;          // 0–255
    // For 10-bit PWM: CCP1CONbits.DC1B = (duty_cycle & 0x03); // Optional for higher res
}

// === SEND TEMP VIA UART ===
void send_data_to_uart(float temp) {
    char buffer[20];
    sprintf(buffer, "Temp: %.2f\n", temp);
    for (int i = 0; buffer[i] != '\0'; i++) {
        while (!TXSTAbits.TRMT); // Wait for TX buffer to empty
        TXREG = buffer[i];       // Send char
    }
}

// === DS18B20 READING FUNCTIONS ===
float read_ds18b20(void) {
    uint8_t temp_lsb, temp_msb;
    ds18b20_init(); // Initialize the DS18B20
    ds18b20_write_byte(0xCC); // Skip ROM command
    ds18b20_write_byte(0x44); // Start temperature conversion
    __delay_ms(750); // Wait for conversion (750ms for DS18B20)
    
    ds18b20_init(); // Re-initialize for reading
    ds18b20_write_byte(0xCC); // Skip ROM command
    ds18b20_write_byte(0xBE); // Read Scratchpad command
    
    temp_lsb = ds18b20_read_byte(); // Read LSB
    temp_msb = ds18b20_read_byte(); // Read MSB
    
    // Combine the two bytes to get the temperature
    int16_t raw_temp = (temp_msb << 8) | temp_lsb;
    return (float)raw_temp * 0.0625; // Convert to Celsius
}

// === DS18B20 1-WIRE PROTOCOL ===
void ds18b20_init(void) {
    // Reset the 1-Wire bus
    TRISBbits.TRISB0 = 0; // Set pin as output
    TEMP_SENSOR_PIN = 0; // Pull low
    __delay_us(480); // Wait 480us
    TEMP_SENSOR_PIN = 1; // Release the bus
    __delay_us(60); // Wait 60us
    TRISBbits.TRISB0 = 1; // Set pin as input
    __delay_us(420); // Wait for presence pulse
}

uint8_t ds18b20_read_byte(void) {
    uint8_t byte = 0;
    for (uint8_t i = 0; i < 8; i++) {
        byte >>= 1; // Shift byte to the right
        TRISBbits.TRISB0 = 0; // Set pin as output
        TEMP_SENSOR_PIN = 0; // Pull low
        __delay_us(2); // Wait 2us
        TRISBbits.TRISB0 = 1; // Set pin as input
        __delay_us(10); // Wait 10us
        if (TEMP_SENSOR_PIN) {
            byte |= 0x80; // If pin is high, set the MSB
        }
        __delay_us(50); // Wait for the rest of the bit time
    }
    return byte;
}

void ds18b20_write_byte(uint8_t byte) {
    for (uint8_t i = 0; i < 8; i++) {
        TRISBbits.TRISB0 = 0; // Set pin as output
        TEMP_SENSOR_PIN = 0; // Pull low
        __delay_us(2); // Wait 2us
        TEMP_SENSOR_PIN = (byte & 0x01) ? 1 : 0; // Write bit
        __delay_us(60); // Wait for the rest of the bit time
        TEMP_SENSOR_PIN = 1; // Release the bus
        byte >>= 1; // Shift byte to the right
    }
}
