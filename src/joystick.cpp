/*
 * joystick.c
 *
 *  Created on: Jun 14, 2018
 *      Author: priori
 */

#include "stdio.h"
#include "stdlib.h"
#include "stdint.h"
#include "string.h"
#include "errno.h"

#include <unistd.h>
#include <sys/types.h>
#include <fcntl.h>

#include "head_neck_ctl.h"
#include "joystick.h"

extern int stop_js;
extern struct pan_tilt_ctrl pt_ctrl;

void process_js_event(struct js_event e) {

	uint8_t type = e.type & ~JS_EVENT_INIT;

	// only response when the button is released
	if (type == JS_EVENT_BUTTON && e.value == 1) {

//		printf("Butten event: ts=%u, number=%u, value=%d\n", e.time, e.number,
//				e.value);

		switch (e.number) {

		case 0:

			break;
		case 1:

			break;
		case 2:

			break;
		case 3:

			break;
		case 4:
		case 5:
		case 6:
		case 7:

			break;
		default:

			break;
		}

	} else if (type == JS_EVENT_AXIS) {

//		printf("AXIS event: ts=%u, number=%u, value=%d\n", e.time, e.number,
//				e.value);

		switch (e.number) {

		case 0:
			pt_ctrl.pan_left = js_to_servo(e.value);
			break;
		case 1:
			pt_ctrl.tilt_left = js_to_servo(e.value);
			break;
		case 2:
			pt_ctrl.pan_right = js_to_servo(e.value);
			break;
		case 3:
			pt_ctrl.tilt_right = js_to_servo(e.value);
			break;
		case 4:
			if (e.value > 0 && pt_ctrl.pan_neck < 2000) {
				pt_ctrl.pan_neck += 20;
			} else if (e.value < 0 && pt_ctrl.pan_neck > 1000) {
				pt_ctrl.pan_neck -= 20;
			}
			break;
		case 5:
			if (e.value > 0 && pt_ctrl.tilt_neck < 2000) {
				pt_ctrl.tilt_neck += 20;
			} else if (e.value < 0 && pt_ctrl.tilt_neck > 1000) {
				pt_ctrl.tilt_neck -= 20;
			}
			break;
		default:

			break;
		}
	}

}

void *joystick_thread(void *para) {

	int ret = 0;

	int fd = open("/dev/input/js0", O_RDONLY, O_NONBLOCK);
	if (fd < 0) {
		printf("Cannot find a joystick attached.\n");
		exit(0);
	}

	while (!stop_js) {

		struct js_event e;
		ret = read(fd, &e, sizeof(e));

		if (ret < 0) {

			printf("Unable to read joystick event\n");
			break;
		}

		process_js_event(e);

	}

	return NULL;
}

