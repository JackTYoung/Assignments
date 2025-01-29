/*** Lab9JY.c
COMP 37 Lab 9 - DC Motor
PIC18F67J60 Microchip MPLAB C18
Jack Young (10226891)
November 11th, 2023
2)	The motor and LEDs will advance one step through the sequence of states of STOPPED, WARN_FWD, FORWARD, WARN_REV, REVERSE, STOPPED with cooresponding LED's 
 ***/

#include <p18cxxx.h>
#include "millis_37.h"
#pragma config FOSC = HSPLL
#pragma config WDT = OFF
#pragma config XINST = OFF

#define DURATION 1000 

// Boolean values
#define FALSE (0)
#define TRUE (!FALSE)

// PWM parameters
#define PWM_PRESCALE 0b01
#define PWM_PERIOD 103
#define PWM_40PCT 42
#define PWM_FRACBITS 0b00

// GPIO port bits
#define BUTTON_PIN PORTBbits.RB0
#define BUILTIN_LED PORTBbits.RB4
#define MOTOR_A PORTBbits.RB3
#define MOTOR_B PORTBbits.RB5
#define PORTB_DIR 0b11101001
#define LED_RED PORTDbits.RD0
#define LED_GRN PORTDbits.RD1
#define PWM_EN_PIN PORTDbits.RD2
#define PORTD_DIR 0b11111000

// State machine definitions
typedef enum {STOPPED, FORWARD, REVERSE, WARN_FWD, WARN_REV} state_t;  // state variable type
state_t state = STOPPED;

// PWM parameters are pre-calculated offline
void init_pwm(unsigned char prescale, unsigned char period,
            unsigned char duty, unsigned char fracbits);
void update_pwm(unsigned char duty, unsigned char fracbits);

void main(void)
{
    unsigned int deadline = 0;  // needed for timing with millis()
    unsigned char pressed = FALSE;  // True if leading edge detected
    unsigned char button_state = 1;
    unsigned char button_previous = 1;

	init_millis(); // init timer1 for millis() function

	// set up the GPIO port pins
    PORTB = 0;
    TRISB = PORTB_DIR;  // Port B data direction
    TRISD = PORTD_DIR;  // PWM and LEDs output

    init_pwm(PWM_PRESCALE, PWM_PERIOD, 0, 0);  // start at 0% duty cycle
    BUILTIN_LED = 0; // turn on, active low

	while(1) // main loop starts here
	{
        // Detect a leading edge of a press of the pushbutton.
        // pressed will be true once, then the user has to 
        // release the button and press again.
        // Reset the variable after use.
        button_state = BUTTON_PIN; // active low input
        if((0 == button_state) && (1 == button_previous)) {
            pressed = TRUE;
        }
        button_previous = button_state;  // save for next iteration

        // This provides a visual indication that the button was pressed.
        // The LED state will toggle every time the button is pressed.
        if(pressed) {
            BUILTIN_LED = ~BUILTIN_LED;
        }

        // This is the state machine implementation.
        // In every state, we ask the question "do we need to do anything?"
        // In this state machine, actions are triggered by the variable pressed.
        // When the button is pressed, the state is advanced to the next.
        // The actions are taken to match the destination state. For example,
        // when going from STOPPED to FORWARD, set the PWM, direction and LEDs
        // to match what they should be in state FORWARD.
        switch(state)
        {
            case STOPPED:
            if (pressed)
            {
                state = WARN_FWD
                update_pwm(0,0);
                MOTOR_A = 0;
                MOTOR_B = 0;
                LED_GRN = 1;
                LED_RED = 1;
                deadline = millis() + DURATION  
            }
            break;

            case WARN_FWD:
             if(millis() >= deadline)  
             {
                state = FORWARD;
				update_pwm(PWM_40PCT, PWM_FRACBITS);
				MOTOR_A = 1;  
                MOTOR_B = 0;
				LED_GRN = 1;
				LED_RED = 0;
             }

            case FORWARD:
            if(pressed)
            {
               state = WARN_REV
                update_pwm(0,0);
                MOTOR_A = 0;
                MOTOR_B = 0;
                LED_GRN = 0;
                LED_RED = 0;
                deadline = millis() + DURATION  
            }
            break;

            case WARN_REV:
             if(millis() >= deadline)  
             {
                state = REVERSE;
				update_pwm(PWM_40PCT, PWM_FRACBITS);
				MOTOR_A = 0;  
                MOTOR_B = 1;
				LED_GRN = 0;
				LED_RED = 1;
             }
             break;
            case REVERSE:
            if(pressed)
            {
                state = STOPPED;
                update_pwm(0, 0);  //  off 0%
                MOTOR_A = 0;  // both off
                MOTOR_B = 0;
                LED_GRN = 0;
                LED_RED = 0;
            }
            break;
           
            default:
                // in case state becomes none-of-the-above
                state = STOPPED;
                update_pwm(0, 0);  //  off 0%
                MOTOR_A = 0;  // both off
                MOTOR_B = 0;
                LED_GRN = 0;
                LED_RED = 0;
        }

        pressed = FALSE;  // reset afer use in the state machine
    }
}

// PWM initialization
void init_pwm(unsigned char prescale,  // Timer 2 prescale bits
                unsigned char period, // Timer 2 period register
                unsigned char duty,  // CCPRL duty cycle value
                unsigned char fracbits)  // CCP fractional duty cycle bits
{
    // CCP4 PWM mode
    CCP4CONbits.CCP4M = 0b1100;

    // Timer 2
    PR2 = period;
    T2CONbits.T2CKPS = prescale;

    // Set DC 8 bits integer and 2 bits fractional
    CCPR4L = duty;
    CCP4CONbits.DC4B = fracbits;

    T2CONbits.TMR2ON = 1; // start the PWM system
}

// PWM update duty cycle only, don't alter period
void update_pwm(unsigned char duty, unsigned char fracbits)
{
    // Set DC 8 bits integer and 2 bits fractional
    CCPR4L = duty;
    CCP4CONbits.DC4B = fracbits;
}
