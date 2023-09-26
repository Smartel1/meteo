#ifndef SIM800l_H_
#define SIM800l_H_

#include <stdio.h>
#include <stdint.h>

#define uart_num 				UART_NUM_1

void configureUART();

void turnOnSim800l();

void turnOffSim800l();

int sendCommand(char * cmd, char *ok_response);

#endif