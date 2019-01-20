#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>
#include <string.h>
#include <errno.h>
#include <math.h>

#include "head_neck_ctl.h"

#include <iostream>
#define PI 3.14159265359

using namespace std;

double pid_ctrl(double err, double err_sum, double err_diff, double kp, double ki, double kd) {

	return err * kp + err_sum * ki + err * kd;
}

void vergence_servo_ctrl( float *q){
	
	// PID controller

	// double kpneck = 0.005;
	// double kptilt = 0.02;
	double kpvg = 0.02;

    float kpneck = 0.1;
    float kptilt = 0.2;
    float kpverg = 0.2;
	if (!isnan(q[0]) && !isnan(q[1]) && !isnan(q[2])) {

		float out_neck = kpneck * q[0] * 1000 / PI;
		float out_vg = kpverg * q[1] * 1000 / PI;
		float out_tilt = kptilt * q[2] * 1000 / PI;

		// CAUTION: Be careful on the servo direction!!!

		pt_ctrl.pan_neck += out_neck;
        pt_ctrl.pan_left += out_vg;
        pt_ctrl.pan_right += out_vg;
        pt_ctrl.tilt_left += out_tilt;
        pt_ctrl.tilt_right += out_tilt;

	}
}