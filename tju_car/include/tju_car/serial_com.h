#include<stdio.h>
#include<stdlib.h>
#include<unistd.h>
#include<sys/types.h>
#include<sys/stat.h>
#include<fcntl.h>
#include<termios.h>
#include<errno.h>
#include<string.h>

#define FALSE  -1
#define TRUE   0

int UART0_Open(int fd, char *port);
void UART0_Close(int fd);
int UART0_Set(int fd, int speed, int flow_ctrl, int databits, int stopbits, int parity);
int UART0_Init(int fd, int speed, int flow_ctrl, int databits, int stopbits, int parity);
int UART0_Recv(int fd, char *rcv_buf, int data_len);
int UART0_Send(int fd, char send_buf[], int data_len);
