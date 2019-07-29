/** !!!!!!!!!!!!!!!!!!!!!
 * before cmake this exe file, disable printf lines in sprotocol.cpp
	!!!!!!!!!!!!!!!!!!!!!**/
#include <unistd.h>
#include <sys/types.h>
#include <fcntl.h>
#include <pthread.h>

#include "serial.h"
#include "sprotocol.h"
#include "head_neck_ctl.h"

#include <iostream>
#include <cstdio>
// ----------- configuration -------------------

int period = 100 * 1000; // 100 ms
int stop_serial_tx = 0;
int stop_serial_rx = 0;
int stop_js = 0;

struct pan_tilt_ctrl pt_ctrl = {
	// change servo
	.pan_left = 1500,
	.tilt_left = 1500,
	.pan_right = 1500,
	.tilt_right = 1000,
	.pan_neck = 1500,
	.tilt_neck = 1450
};
extern struct h2l_state_pan_tilt* state_msg;

void calibrate_ctl_pos(){
	// Switch servo
	//pt_ctrl.pan_neck += 1;
	//pt_ctrl.pan_left += 1;
	//pt_ctrl.pan_right += 1;
	//pt_ctrl.tilt_left += 1;
	pt_ctrl.tilt_right += 1;
}
using namespace std;

int main(int argc, char const *argv[])
{
    int ret = 0;

    int sfd = serial_init("/dev/ttyACM0", 115200);
    if (sfd < 0) {
    	printf("Unable to open serial port!!!\n");
        exit(-1);
    }

    // CAUTION: When the serial port is opened, Arduino board is reset.
    // We must wait for a while before sending the first message so that the
    // Arduino board is correctly setup.
    sleep(1);

    // !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!thread js?
    pthread_t tx_serial;
    pthread_t rx_serial;

    ret = pthread_create(&tx_serial, NULL, serial_send_thread, &sfd);
    if (ret != 0) {
        fprintf(stderr, "Error - pthread_create() return code: %d\n", ret);
        exit(EXIT_FAILURE);
    }
    ret = pthread_create(&rx_serial, NULL, serial_recv_thread, &sfd);
    if (ret != 0) {
        fprintf(stderr, "Error - pthread_create() return code: %d\n", ret);
        exit(EXIT_FAILURE);
    }

    // change servo
    freopen("tilt_right_log.txt", "w", stdout);
    usleep(10000000);

    //calibrate_ctl_pos();

    while(pt_ctrl.tilt_right < 2000){ // change servo
        calibrate_ctl_pos();
        cout<<"\n[rx] ts="<<state_msg->timestamp;
        //<<"\n, pan_left_ctl=" <<msg->pan_left
        //<<"\n tilt_left_ctl=" <<msg->tilt_left
        //<<"\n pan_right_ctl=" <<msg->pan_right
        //<<"\n tilt_right_ctl="  <<msg->tilt_right
        //<<"\n, pan_neck_ctl="  <<msg->pan_neck
        //<<"\n, pan_left_pos="<<msg->pan_left_pos;
        //<<"\n tilt_left_pos=" <<msg->tilt_left_pos;
        //<<"\n pan_right_pos=" <<msg->pan_right_pos;
        //<<"\n tilt_right_pos="<<msg->tilt_right_pos;
        //<<", pan_neck_pos="  <<msg->pan_neck_pos<<"\n";
        usleep(100000);
    }

    pthread_join(tx_serial, NULL);
    pthread_join(rx_serial, NULL);
    return 0;
}
