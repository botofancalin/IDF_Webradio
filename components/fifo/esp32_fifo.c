/******************************************************************************
 * Copyright 2013-2015 Espressif Systems
 *
 * FileName: user_main.c
 *
 * Description: Routines to use a SPI RAM chip as a big FIFO buffer. Multi-
 * thread-aware: the reading and writing can happen in different threads and
 * will block if the fifo is empty and full, respectively.
 *
 * Modification history:
 *     2015/06/02, v1.0 File created.
*******************************************************************************/
#include "esp_system.h"
#include "string.h"
#include <stdio.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
#include "freertos/queue.h"
#include "esp_log.h"

#include "esp32_fifo.h"

#define TAG "FIFO"

//Low watermark where we restart the reader thread.
#define FIFO_LOWMARK (112 * 1024)
#define SPIREADSIZE 64

static int fifoRpos;
static int fifoWpos;
static int fifoFill;
static xSemaphoreHandle semCanRead;
static xSemaphoreHandle semCanWrite;
static xSemaphoreHandle mux;
static long fifoOvfCnt, fifoUdrCnt;
static char *fifoBuffer = NULL;
static size_t buffSize = 128*1024;

#define RamWrite(pos, buf, n) memcpy(&fifoBuffer[pos], buf, n)
#define RamRead(pos, buf, n) memcpy(buf, &fifoBuffer[pos], n)

//Initialize the FIFO
bool FifoInit()
{
	fifoRpos = 0;
	fifoWpos = 0;
	fifoFill = 0;
	fifoOvfCnt = 0;
	fifoUdrCnt = 0;
	vSemaphoreCreateBinary(semCanRead);
	vSemaphoreCreateBinary(semCanWrite);
	mux = xSemaphoreCreateMutex();
	fifoBuffer = (char *)calloc(buffSize, sizeof(char));
	if (!fifoBuffer)
	{
		buffSize = 16384; //allocate enough for about one mp3 frame(min 1850)

		ESP_LOGW(TAG, "SPI RAM not found! Trying to allocate 16k in RAM");
		fifoBuffer = (char *)calloc(buffSize, sizeof(char));

		if (!fifoBuffer)
		{
			ESP_LOGE(TAG, "Fifo buffer allocation failed");
			return false;
		}
	}
	else
	{
		ESP_LOGI(TAG, "Fifo buffer alocated in SPI RAM");
	}
	return true;
}

void FifoReset()
{
	xSemaphoreTake(mux, portMAX_DELAY);
	fifoRpos = 0;
	fifoWpos = 0;
	fifoFill = 0;
	fifoOvfCnt = 0;
	fifoUdrCnt = 0;
	xSemaphoreGive(semCanWrite);
	xSemaphoreGive(mux);
}

//Read bytes from the FIFO
void FifoRead(char *buff, int len)
{
	int n;
	while (len > 0)
	{
		n = len;
		if (n > SPIREADSIZE)
			n = SPIREADSIZE; //don't read more than SPIREADSIZE
		if (n > (buffSize - fifoRpos))
			n = buffSize - fifoRpos; //don't read past end of buffer
		xSemaphoreTake(mux, portMAX_DELAY);
		if (fifoFill < n)
		{
			fifoUdrCnt++;
			xSemaphoreGive(mux);
			if (fifoFill < FIFO_LOWMARK)
			{
				xSemaphoreTake(semCanRead, portMAX_DELAY);
			}
		}
		else
		{
			//Read the data.
			RamRead(fifoRpos, buff, n);
			buff += n;
			len -= n;
			fifoFill -= n;
			fifoRpos += n;
			if (fifoRpos >= buffSize)
			{
				fifoRpos = 0;
			}
			xSemaphoreGive(mux);
			xSemaphoreGive(semCanWrite); //Indicate writer thread there's some free room in the fifo
		}
	}
}

//Write bytes to the FIFO
void FifoWrite(const char *buff, int buffLen)
{
	int n;
	while (buffLen > 0)
	{
		n = buffLen;

		// don't read more than SPIREADSIZE
		if (n > SPIREADSIZE)
		{
			n = SPIREADSIZE;
		}

		// don't read past end of buffer
		if (n > (buffSize - fifoWpos))
		{
			n = buffSize - fifoWpos;
		}

		xSemaphoreTake(mux, portMAX_DELAY);
		if ((buffSize - fifoFill) < n)
		{
			fifoOvfCnt++;
			xSemaphoreGive(mux);
			xSemaphoreTake(semCanWrite, 200 / portTICK_PERIOD_MS);
			taskYIELD();
		}
		else
		{
			// Write the data.
			RamWrite(fifoWpos, buff, n);
			buff += n;
			buffLen -= n;
			fifoFill += n;
			fifoWpos += n;
			if (fifoWpos >= buffSize)
			{
				fifoWpos = 0;
			}
			xSemaphoreGive(mux);
			xSemaphoreGive(semCanRead); // Tell reader thread there's some data in the fifo.
		}
	}
}

//Get amount of bytes in use
int FifoFill()
{
	int ret;
	xSemaphoreTake(mux, portMAX_DELAY);
	ret = fifoFill;
	xSemaphoreGive(mux);
	return ret;
}

int FifoFree()
{
	return (buffSize - FifoFill());
}

int FifoLen()
{
	return buffSize;
}

long GetOverrunCt()
{
	long ret;
	xSemaphoreTake(mux, portMAX_DELAY);
	ret = fifoOvfCnt;
	xSemaphoreGive(mux);
	return ret;
}

long GetUnderrunCt()
{
	long ret;
	xSemaphoreTake(mux, portMAX_DELAY);
	ret = fifoUdrCnt;
	xSemaphoreGive(mux);
	return ret;
}
