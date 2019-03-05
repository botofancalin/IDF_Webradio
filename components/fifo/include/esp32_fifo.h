#ifndef _FIFO_H_
#define _FIFO_H_

#include <inttypes.h>
#include <stddef.h>

bool  FifoInit();
void  FifoRead(char *buff, int len);
void  FifoWrite(const char *buff, int len);
int  FifoFill();
int  FifoFree();
long  GetOverrunCt();
long  GetUnderrunCt();

void FifoReset();
int FifoLen();

#endif
