/*
 * sprotocol.c
 *
 *  Created on: Jun 14, 2018
 *      Author: priori
 */

#include <iostream>
#include "stdio.h"
#include "stdlib.h"
#include "stdint.h"
#include "string.h"
#include "errno.h"

#include <unistd.h>
#include <sys/types.h>

#include "serial.h"
#include "head_neck_ctl.h"
#include "sprotocol.h"

extern int period;
extern int stop_serial_tx;
extern int stop_serial_rx;


struct h2l_state_pan_tilt* state_msg = (struct h2l_state_pan_tilt*) malloc(sizeof(struct h2l_state_pan_tilt));
// ---------------- serial -------------------

static int h2l_send_pan_tilt_msg(int sfd, int16_t pan_left, int16_t tilt_left,
		int16_t pan_right, int16_t tilt_right, int pan_neck, int tilt_neck) {

	int ret = 0;
	struct h2l_ctrl_pan_tilt msg;

	h2l_set_header(&msg.header, sizeof(msg) - sizeof(msg.header),
			OPCODE_CTRL_PAN_TILT);

	msg.pan_left = pan_left;
	msg.tilt_left = tilt_left;
	msg.pan_right = pan_right;
	msg.tilt_right = tilt_right;
	msg.pan_neck = pan_neck;
	msg.tilt_neck = tilt_neck;

	ret = serial_send_n_bytes(sfd, (char *) &msg, sizeof(msg));
	
	if (ret < 0) {
		printf("Unable to write to serial port, ret=%d, errno=%s.\n", ret, strerror(errno));
	}
	//serial_flush(sfd);

	return ret;
}

void *serial_send_thread(void *para) {

	// CAUTION: always sleep 1ms as an small interval
	struct timespec idle;
	idle.tv_sec = 0;
	idle.tv_nsec = 1000 * 1000;

	int sfd = *((int *) para);

	printf("serial sending thread started.\n");

	uint64_t init_ts = get_us();

	// serial loop
	for (uint32_t i = 0; !stop_serial_tx; i++) {

		h2l_send_pan_tilt_msg(sfd, pt_ctrl.pan_left, pt_ctrl.tilt_left,
				pt_ctrl.pan_right, pt_ctrl.tilt_right, pt_ctrl.pan_neck, pt_ctrl.tilt_neck);


//		printf(
//				"[tx] pan_left=%d, tilt_left=%d, pan_right=%d, tilt_right=%d\n",
//				pt_ctrl.pan_left, pt_ctrl.tilt_left, pt_ctrl.pan_right,
//				pt_ctrl.tilt_right);

		uint64_t next_ts = init_ts + period * (i + 1);
		uint64_t ts = get_us();

//		printf("[serial] next_ts=%lu, ts=%lu\n", next_ts - init_ts, ts - init_ts);

		while (ts < next_ts) {

			nanosleep(&idle, NULL);
			ts = get_us();
		}

	}

	printf("serial sending thread exited.\n");

	return NULL;
}

static int h2l_recv_process(int sfd, char serial_buff[]) {

	int ret;
	int i;

	int len = (int)serial_buff[2] & 0xFF;
	int opcode = (int)serial_buff[3] & 0xFF;

	// receive payload
	ret = serial_recv_n_bytes(sfd, serial_buff + sizeof(struct h2l_header), len);

//	printf("[rx]");
//	for(i = 0; i < len + sizeof(struct h2l_header); i++) {
//		printf("%02X ", serial_buff[i] & 0xFF);
//	}
//	printf("\n");

	switch (opcode) {

	case OPCODE_STATE_PAN_TILT: {

		state_msg = (struct h2l_state_pan_tilt *) (serial_buff);
		//printf("[rx] ts = %u \n", state_msg->timestamp);
		printf("[rx] ts=%u, pan_left_pos=%d, tilt_left_pos=%d, "
				"pan_right_pos=%d, tilt_right_pos=%d, "
				"pan_neck_pos=%d, tilt_neck_pos=%d\n",

				state_msg->timestamp, state_msg->pan_left_pos, state_msg->tilt_left_pos,
				state_msg->pan_right_pos, state_msg->tilt_right_pos,
				state_msg->pan_neck_pos, state_msg->tilt_neck_pos);

		break;
	}
	default: break;
	}

	return 0;
}

void *serial_recv_thread(void *para) {

	int ret;
	int sfd = *((int *) para);
	char c;
	int comm_state = SERIAL_STATE_INIT;
	char serial_buff[128];

	printf("serial receiving thread started.\n");

	while (!stop_serial_rx) {

		// CAUTION: This is polling every 1 ms!!!
		// receive protocol header
		ret = serial_recv_byte(sfd, &c);
		if(ret < 0) {
			printf("Serial read error!!!\n");
			break;
		} else if(ret == 0) {

			printf("Nothing is received!!!\n");
			usleep(1000 * 1000);
			continue;
		}


		serial_buff[comm_state] = c;

		//printf("%02X ", (c & 0xFF));

		switch (comm_state) {
		case SERIAL_STATE_INIT: {
			if (c == SERIAL_MAGIC_1)
				comm_state = SERIAL_STATE_MAGIC1;
			else
				comm_state = SERIAL_STATE_INIT;
			break;
		}
		case SERIAL_STATE_MAGIC1: {
			if (c == SERIAL_MAGIC_2)
				comm_state = SERIAL_STATE_MAGIC2;
			else
				comm_state = SERIAL_STATE_INIT;
			break;
		}
		case SERIAL_STATE_MAGIC2: {
			comm_state = SERIAL_STATE_PROTO;
			break;
		}
		case SERIAL_STATE_PROTO: {

			h2l_recv_process(sfd, serial_buff);

			comm_state = SERIAL_STATE_INIT;
			break;
		}
		default: {
			comm_state = SERIAL_STATE_INIT;
			break;
		}

		}

	}

	printf("serial receiving thread exited.\n");

	return NULL;
}

