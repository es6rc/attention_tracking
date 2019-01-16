/*
 * joystick.h
 *
 *  Created on: Jun 14, 2018
 *      Author: priori
 */

#ifndef JOYSTICK_H_
#define JOYSTICK_H_

// ---------------- joystick -------------------

#define JS_EVENT_BUTTON         0x01    /* button pressed/released */
#define JS_EVENT_AXIS           0x02    /* joystick moved */
#define JS_EVENT_INIT           0x80    /* initial state of device */

struct js_event {

	uint32_t time; /* event timestamp in milliseconds */
	int16_t value; /* value */
	uint8_t type; /* event type */
	uint8_t number; /* axis/button number */
};

static inline int16_t js_to_servo(int js_value) {

	return (int16_t) (js_value * 500 / 32768 + 1500);

}


void *joystick_thread(void *para);


#endif /* JOYSTICK_H_ */
