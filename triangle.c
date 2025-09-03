#include <xc.h>

#define _XTAL_FREQ 8000000  // Define the oscillator frequency (8 MHz)
#define STEPS_PER_REV 200    // Number of steps for a full rotation (depends on your stepper motor)

// Function prototypes
void rotateClockwise(int steps);
void rotateCounterClockwise(int steps);
void delayMs(int milliseconds);

void main(void) {
    // Configure PORTB as output
    TRISB = 0x00;  // Set PORTB as output
    LATB = 0x00;   // Initialize LATB to 0

    // Rotate clockwise for 360 degrees (10 seconds)
    rotateClockwise(STEPS_PER_REV);
    delayMs(10000); // Wait for 10 seconds

    // Rotate counterclockwise for 180 degrees (3 seconds)
    rotateCounterClockwise(STEPS_PER_REV / 2);
    delayMs(3000); // Wait for 3 seconds

    // Rotate clockwise for 90 degrees (1 second)
    rotateClockwise(STEPS_PER_REV / 4);
    delayMs(1000); // Wait for 1 second

    while(1); // Infinite loop to end the program
}

void rotateClockwise(int steps) {
    for (int i = 0; i < steps; i++) {
        // Rotate stepper motor clockwise using assembly instruction
        asm("RRNCF LATB, 10"); // Rotate LATB to the right, affecting bit 10
        __delay_ms(10);         // Delay for step duration
    }
}

void rotateCounterClockwise(int steps) {
    for (int i = 0; i < steps; i++) {
        // Rotate stepper motor counterclockwise using assembly instruction
        asm("RLCF LATB, 10"); // Rotate LATB to the left, affecting bit 10
        __delay_ms(10);       // Delay for step duration
    }
}

void delayMs(int milliseconds) {
    for (int i = 0; i < milliseconds; i++) {
        __delay_ms(1); // Delay for 1 ms
    }
}

