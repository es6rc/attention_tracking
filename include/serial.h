
#include <stddef.h>

#ifndef SERIAL_H_
#define SERIAL_H_

int serial_init(const char* serialport, int baud);
int serial_close(int fd);

// CAUTION: these send and receiving are blocking / busy waiting
int serial_send_n_bytes(int fd, const char* buff, size_t n);
int serial_recv_n_bytes(int fd, char *buff, size_t n);

int serial_flush(int fd);

// CAUTION: this is nonblocking receive
int serial_recv_byte(int fd, char *buff);


#endif /* SERIAL_H_ */
