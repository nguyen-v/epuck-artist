/**
 * @file    main.cpp
 * @brief   Handles BT communication, servomotor and stepper motor of Arduino.
 *          Very simple program to move the stepper motor to the correct 
 *          position and lift the pens if necessary.
 * 
 * @note    SoftwareSerial.h and Servo.h conflict with each other (timer)
 *          The only compatible libraries tested are NeoSWSerial paired
 *          with ServoTimer2
 */

// Hardware                                                                  

// Board:                 Arduino Nano ATmega328
// Bluetooth module:      Whadda Funkmodul HC-05 Bluetooth
// Servomotor:            Amewi Servo Digital Micro ST55MG 5.5g 
// Stepper motor/driver:  Gear Stepper Motor Driver Pack - Seeed Studio

#include <Arduino.h>

// Libraries
#include <NeoSWSerial.h>
#include <ServoTimer2.h>
#include <Stepper.h>

/*===========================================================================*/
/* Module constants.                                                         */
/*===========================================================================*/

// Messages
#define CONFIMATION_MSG           "Ready"

// Intervals
#define SERVO_STEPPER_INTERVAL    500 // ms

// HC-05 bluetooth module
#define BT_SERIAL_PIN_1           2
#define BT_SERIAL_PIN_2           3
#define BT_BAUD_RATE              9600

// Serial
#define SERIAL_BAUD_RATE          9600

// Servo
#define PIN_SERVO                 10
#define DELTA_PULSE               350

// Stepper
#define PIN_STEPPER_1             5
#define PIN_STEPPER_2             11
#define PIN_STEPPER_3             6
#define PIN_STEPPER_4             12
#define STEPPER_SPEED             5
#define STEPS_PER_REV             2048

// 4 pens are placed on a bit more of a quarter of the full revolution
#define STEPPER_MAX_VALUE         690

// because of weight distribution, epuck is tilted depending on the color chosen
// an offset is added to compensate. The positions are from right to left.
#define STEPPER_OFFSET            30
#define STEPPER_POSITION_0        (0 + STEPPER_OFFSET)
#define STEPPER_POSITION_1        (STEPPER_MAX_VALUE/4 + STEPPER_OFFSET/2)
#define STEPPER_POSITION_2        (STEPPER_MAX_VALUE*2/4 - STEPPER_OFFSET/2)
#define STEPPER_POSITION_3        (STEPPER_MAX_VALUE*3/4 - STEPPER_OFFSET)

#define DEFAULT_POSITION          STEPPER_POSITION_0

/*===========================================================================*/
/* Module data structures and types.                                         */
/*===========================================================================*/
enum Color {white, black, red, green, blue};

/*===========================================================================*/
/* Module local variables.                                                   */
/*===========================================================================*/

static NeoSWSerial BTserial(BT_SERIAL_PIN_1, BT_SERIAL_PIN_2);
static ServoTimer2 servo;
static Stepper stepper = Stepper(STEPS_PER_REV, PIN_STEPPER_1, PIN_STEPPER_2, 
                                 PIN_STEPPER_3, PIN_STEPPER_4);
static int16_t stepper_position = DEFAULT_POSITION;


/*===========================================================================*/
/* Module local functions.                                                   */
/*===========================================================================*/

/**
 * @brief             moves stepper motor and servo motor to change colors
 * @param[in]   col   a valid color (enum Color)
 * @return            none
 */
static void change_color(Color col)
{
	// lift servo
	servo.write(DEFAULT_PULSE_WIDTH+DELTA_PULSE);
	delay(SERVO_STEPPER_INTERVAL);
	int16_t goal_step = 0;
	switch (col) {
		case white:
			return; // if white is requested, just lift the pens
			break;
		case black:
			goal_step = STEPPER_POSITION_0;
			break;
		case red:
			goal_step = STEPPER_POSITION_1;
			break;
		case green:
			goal_step = STEPPER_POSITION_2;
			break;
		case blue:
			goal_step = STEPPER_POSITION_3;
			break;
	}

	int16_t delta_step = goal_step - stepper_position;
	stepper.step(delta_step);
	delay(SERVO_STEPPER_INTERVAL);
	stepper_position = goal_step;
	// lower servo
	servo.write(DEFAULT_PULSE_WIDTH);
}

/**
 * @brief             resets motors to their standby state.
 * @return            none
 */
static void reset_motors()
{
	servo.write(DEFAULT_PULSE_WIDTH+DELTA_PULSE);
	delay(SERVO_STEPPER_INTERVAL);
	int16_t goal_step = DEFAULT_POSITION;
	int16_t delta_step = goal_step - stepper_position;
	stepper.step(delta_step);
	stepper_position = goal_step;
}

/*===========================================================================*/
/* Module functions.                                                         */
/*===========================================================================*/

void setup() 
{
	servo.attach(PIN_SERVO);
	stepper.setSpeed(STEPPER_SPEED);

	Serial.begin(SERIAL_BAUD_RATE);
	BTserial.begin(BT_BAUD_RATE);

	delay(SERVO_STEPPER_INTERVAL);
	servo.write(DEFAULT_PULSE_WIDTH+DELTA_PULSE);
}

void loop() 
{
	if (BTserial.available()) {
		char cmd = BTserial.read();
		switch (cmd) {
			case 'W':
				change_color(white);
				break;
			case 'D': // dark
				change_color(black);
				break;
			case 'R':
				change_color(red);
				break;
			case 'G':
				change_color(green);
				break;
			case 'B':
				change_color(blue);
				break;
			case 'X':
				reset_motors();
				break;
		}
	BTserial.println(CONFIMATION_MSG);
	}
}
