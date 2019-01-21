/*
 * sprotocol.h
 *
 *  Created on: May 10, 2018
 *      Author: priori
 */

#ifndef SPROTOCOL_H_
#define SPROTOCOL_H_

#include <cstdint>

// ---------------- serial protocol -------------------

#define MC_PROTO_HEADER_SIZE 4

#define SERIAL_STATE_INIT      0
#define SERIAL_STATE_MAGIC1    1
#define SERIAL_STATE_MAGIC2    2
#define SERIAL_STATE_PROTO     3

#define SERIAL_MAGIC_1 'G'
#define SERIAL_MAGIC_2 'V'


#define OPCODE_CTRL_PAN_TILT          	0x10

#define OPCODE_STATE_PAN_TILT          	0x80



struct h2l_header {

	uint8_t magic1;
	uint8_t magic2;
	uint8_t len;
	uint8_t opcode;
};

// message from computer to microcontroller

struct h2l_ctrl_pan_tilt {

	struct h2l_header header;

	int16_t pan_left;
	int16_t tilt_left;
	int16_t pan_right;
	int16_t tilt_right;
	int16_t pan_neck;
	int16_t tilt_neck;


};

// message from microcontroller to computer

struct h2l_state_pan_tilt {

	struct h2l_header header;
	uint32_t timestamp;
	int16_t pan_left;
	int16_t tilt_left;
	int16_t pan_right;
	int16_t tilt_right;
	int16_t pan_neck;
	int16_t tilt_neck;
	int16_t pan_left_pos;
	int16_t tilt_left_pos;
	int16_t pan_right_pos;
	int16_t tilt_right_pos;
	int16_t pan_neck_pos;
	int16_t tilt_neck_pos;


};


static inline void h2l_set_header(struct h2l_header *pheader, uint8_t len, uint8_t opcode) {

	pheader->magic1 = SERIAL_MAGIC_1;
	pheader->magic2 = SERIAL_MAGIC_2;
	pheader->len = len;
	pheader->opcode = opcode;

}

void *serial_send_thread(void *para);
void *serial_recv_thread(void *para);


#endif /* SPROTOCOL_H_ */
