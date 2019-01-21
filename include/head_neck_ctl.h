/*
 * head_neck_ctl.h
 *
 *  Created on: Jun 14, 2018
 *      Author: priori
 */

#ifndef HEAD_NECK_CTL_H_
#define HEAD_NECK_CTL_H_

#include <stdint.h>
#include <time.h>


// struct gazepoint {

// 	volatile int left_x;
// 	volatile int left_y;
// 	volatile int right_x;
// 	volatile int right_y;

// };
struct pan_tilt_ctrl {

	volatile int16_t pan_left;
	volatile int16_t tilt_left;
	volatile int16_t pan_right;
	volatile int16_t tilt_right;
	volatile int16_t pan_neck;
	volatile int16_t tilt_neck;

};
extern struct pan_tilt_ctrl pt_ctrl;
extern struct h2l_state_pan_tilt* msg;

static inline uint64_t get_us() {

	struct timespec spec;

	clock_gettime(CLOCK_MONOTONIC, &spec);

	uint64_t s = spec.tv_sec;
	uint64_t us = spec.tv_nsec / 1000 + s * 1000 * 1000;

	return us;
}



#endif /* HEAD_NECK_CTL_H_ */
