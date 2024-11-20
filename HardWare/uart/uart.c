/*
 * This file is part of the Arducam SPI Camera project.
 *
 * Copyright 2021 Arducam Technology co., Ltd. All Rights Reserved.
 *
 * This work is licensed under the MIT license, see the file LICENSE for details.
 *
 */
#include "uart.h"
#include <msp430.h>

uint8_t uartCommBuff[20] = { 0 };
uint8_t uartCommLength = 0;
uint8_t readBuffLength = 0;

extern TaskHandle_t xHandle;

unsigned char rxData = 0;

int fputc(int ch, FILE *f)
{
	EUSCI_A_UART_transmitData(EUSCI_A0_BASE, ch & 0xff);
	return ch;
}

int fputs(const char *_ptr, register FILE *_fp)
{
	unsigned int len = strlen(_ptr);

	while (len > 0)
	{
		EUSCI_A_UART_transmitData(EUSCI_A0_BASE, (*_ptr) & 0xff);
		_ptr++;
		len--;
	}

	return strlen(_ptr);
}

void initUART(void)
{
	EUSCI_A_UART_initParam param;
	param.uartMode = EUSCI_A_UART_MODE;
	param.parity = EUSCI_A_UART_NO_PARITY;
	param.msborLsbFirst = EUSCI_A_UART_LSB_FIRST;
	param.numberofStopBits = EUSCI_A_UART_ONE_STOP_BIT;

	/* https://software-dl.ti.com/msp430/msp430_public_sw/mcu/msp430/MSP430BaudRateConverter/index.html */
	/* 115200 bps, SMCLK = 16MHz */
	param.selectClockSource = EUSCI_A_UART_CLOCKSOURCE_SMCLK;
	param.clockPrescalar = 8;
	param.firstModReg = 10;
	param.secondModReg = 247;
	param.overSampling = 1;

	EUSCI_A_UART_init(EUSCI_A0_BASE, &param);
}

void reportVerInfo(ArducamCamera *camera)
{
	uint8_t headAndtail[] = { 0xff, 0xaa, 0x03, 0xff, 0xbb };

	uint32_t len = 6;
	uartWriteBuffer(&headAndtail[0], 3);
	uartWriteBuffer((uint8_t*) &len, 4);
	uartWriteBuffer(camera->verDateAndNumber, 4);
	printf("\r\n");
	uartWriteBuffer(&headAndtail[3], 2);
}

void reportSdkVerInfo(ArducamCamera *camera)
{
	uint8_t headAndtail[] = { 0xff, 0xaa, 0x05, 0xff, 0xbb };

	uint32_t len = 6;
	uartWriteBuffer(&headAndtail[0], 3);
	uartWriteBuffer((uint8_t*) &len, 4);
	uartWriteBuffer((uint8_t*) &camera->currentSDK->sdkVersion, 4);
	printf("\r\n");
	uartWriteBuffer(&headAndtail[3], 2);
}

void reportCameraInfo(ArducamCamera *camera)
{
	uint8_t headAndtail[] = { 0xff, 0xaa, 0x02, 0xff, 0xbb };

	uint32_t len = 0;
	char buff[400];
	uartWriteBuffer(&headAndtail[0], 3);
	sprintf(buff,
			"ReportCameraInfo\r\nCamera Type:%s\r\nCamera Support Resolution:%d\r\nCamera Support "
			"specialeffects:%d\r\nCamera Support Focus:%d\r\nCamera Exposure Value Max:%d\r\nCamera Exposure Value "
			"Min:%d\r\nCamera Gain Value Max:%d\r\nCamera Gain Value Min:%d\r\nCamera Support Sharpness:%d\r\n",
			camera->myCameraInfo.cameraId,
			camera->myCameraInfo.supportResolution,
			camera->myCameraInfo.supportSpecialEffects,
			camera->myCameraInfo.supportFocus,
			camera->myCameraInfo.exposureValueMax,
			camera->myCameraInfo.exposureValueMin,
			camera->myCameraInfo.gainValueMax,
			camera->myCameraInfo.gainValueMin,
			camera->myCameraInfo.supportSharpness);
	len = strlen(buff);
	uartWriteBuffer((uint8_t*) &len, 4);
	printf(buff);
	uartWriteBuffer(&headAndtail[3], 2);
}

void cameraGetPicture(ArducamCamera *camera)
{
#ifdef USING_ARDUCAM_UI
	uint8_t headAndtail[] = { 0xff, 0xaa, 0x01, 0xff, 0xbb };
	uint32_t len = camera->totalLength;
	uartWriteBuffer(&headAndtail[0], 3);
	uartWriteBuffer((uint8_t*) (&len), 4);
	arducamUartWrite(((camera->currentPictureMode & 0x0f) << 4) | 0x01);
#endif

	uint8_t buff[READ_IMAGE_LENGTH] = { 0 };
	uint8_t rtLength = 0;

	while (camera->receivedLength)
	{
		rtLength = readBuff(camera, buff, READ_IMAGE_LENGTH);
		uartWriteBuffer(buff, rtLength);
	}

#ifdef USING_ARDUCAM_UI
	uartWriteBuffer(&headAndtail[3], 2);
#endif
}

//void send_data_pack(char cmd_type, char* msg)
//{
//    uint8_t headAndtail[] = {0xff, 0xaa, 0x07, 0xff, 0xbb};
//    headAndtail[2] = cmd_type;
//    uint32_t len = strlen(msg) + 2;
//    uartWriteBuffer(&headAndtail[0], 3);
//    uartWriteBuffer((uint8_t*)&len, 4);
//    printf(msg);
//    printf("\r\n");
//    uartWriteBuffer(&headAndtail[3], 2);
//}

/* ----------- for 96*96 YUYV B&W format begin ----------- */
extern uint8_t YUVarr[];

void framWriteBuffer(uint8_t *fram, uint8_t *buff, uint32_t length)
{
	while (length > 0)
	{
		*(fram) = *(buff);
		fram++;
		buff += 2;
		length -= 2;
	}
	return;
}

void uartTestWriteBuffer(uint8_t *buff, uint32_t length)
{
	while (length > 0)
	{
		arducamUartWrite(*(buff));
		buff += 2;
		length -= 2;
	}
	return;
}

void cameraGetPicture96x96YUV(ArducamCamera *camera)
{
	uint8_t *fram_addr = YUVarr;
	uint8_t buff[READ_IMAGE_LENGTH] = { 0 };
	uint8_t rtLength = 0;

	while (camera->receivedLength)
	{
		rtLength = readBuff(camera, buff, READ_IMAGE_LENGTH);
		framWriteBuffer(fram_addr, buff, rtLength);
		// uartTestWriteBuffer(buff, rtLength);
		fram_addr += rtLength / 2;
	}
}
/* ------------ for 96*96 YUYV B&W format end ------------ */

uint8_t commandProcessing(ArducamCamera *camera, uint8_t *buff, uint8_t length)
{
	CamStatus state;
	uint16_t GainValue = 0;
	uint32_t ExposureValue = 0;
	uint32_t ExposureLen1 = 0;
	uint32_t ExposureLen2 = 0;
	uint32_t ExposureLen3 = 0;
	uint8_t CameraResolution = camera->currentPictureMode;
	uint8_t CameraFarmat = camera->currentPixelFormat;
	switch (buff[0])
	{
		case SET_PICTURE_RESOLUTION:                     //Set Camera Resolution
			CameraResolution = buff[1] & 0x0f;
			CameraFarmat = (buff[1] & 0x70) >> 4;
			takePicture(camera, (CAM_IMAGE_MODE) CameraResolution,
						(CAM_IMAGE_PIX_FMT) CameraFarmat);
			break;
		case SET_VIDEO_RESOLUTION:                        //Set Video Resolution
			camera->previewMode = TRUE;
			CameraResolution = buff[1] & 0x0f;
			state = startPreview(camera, (CAM_VIDEO_MODE) CameraResolution);
			if (state == CAM_ERR_NO_CALLBACK)
			{
				printf("callback function is not registered");
			}
			break;
		case SET_BRIGHTNESS:                                    //Set brightness
			setBrightness(camera, (CAM_BRIGHTNESS_LEVEL) buff[1]);
			break;
		case SET_CONTRAST:                                        //Set Contrast
			setContrast(camera, (CAM_CONTRAST_LEVEL) buff[1]);
			break;
		case SET_SATURATION:                                    //Set saturation
			setSaturation(camera, (CAM_STAURATION_LEVEL) buff[1]);
			break;
		case SET_EV:                                                    //Set EV
			setEV(camera, (CAM_EV_LEVEL) buff[1]);
			break;
		case SET_WHITEBALANCE:                               //Set White balance
			setAutoWhiteBalanceMode(camera, (CAM_WHITE_BALANCE) buff[1]);
			break;
		case SET_SPECIAL_EFFECTS:                          //Set Special effects
			setColorEffect(camera, (CAM_COLOR_FX) buff[1]);
			break;
		case SET_FOCUS_CONTROL:                                  //Focus Control
			setAutoFocus(camera, buff[1]);
			if (buff[1] == 0)
			{
				setAutoFocus(camera, 0x02);
			}
			break;
		case SET_EXPOSUREANDGAIN_CONTROL:            //exposure and gain control
			setAutoExposure(camera, buff[1] & 0x01);
			setAutoISOSensitive(camera, buff[1] & 0x01);
			break;
		case SET_WHILEBALANCE_CONTROL:                   //while balance control
			setAutoWhiteBalance(camera, buff[1] & 0x01);
			break;
		case SET_SHARPNESS:
			setSharpness(camera, (CAM_SHARPNESS_LEVEL) buff[1]);
			break;
		case SET_MANUAL_GAIN:                              //manual gain control
			GainValue = (buff[1] << 8) | buff[2];
			setISOSensitivity(camera, GainValue);
			break;
		case SET_MANUAL_EXPOSURE:                      //manual exposure control
			ExposureLen1 = buff[1];
			ExposureLen2 = buff[2];
			ExposureLen3 = buff[3];
			ExposureValue = (ExposureLen1 << 16) | (ExposureLen2 << 8)
					| ExposureLen3;
			setAbsoluteExposure(camera, ExposureValue);
			break;
		case GET_CAMERA_INFO:                                  //Get Camera info
			reportCameraInfo(camera);
			break;
		case TAKE_PICTURE:
			takePicture(camera, (CAM_IMAGE_MODE) CameraResolution,
						(CAM_IMAGE_PIX_FMT) CameraFarmat);
			cameraGetPicture(camera);
			break;
		case DEBUG_WRITE_REGISTER:
			debugWriteRegister(camera, buff + 1);
			break;
		case STOP_STREAM:
			stopPreview(camera);
			break;
		case GET_FRM_VER_INFO: // Get Firmware version info
			reportVerInfo(camera);
			break;
		case GET_SDK_VER_INFO: // Get sdk version info
			reportSdkVerInfo(camera);
			break;
		case RESET_CAMERA:
			reset(camera);
		case SET_IMAGE_QUALITY:
			setImageQuality(camera, (IMAGE_QUALITY) buff[1]);
		default:
			break;
	}
	return buff[0];
}

void arducamUartWrite(uint8_t data)
{
	EUSCI_A_UART_transmitData(EUSCI_A0_BASE, data);
}

void uartWriteBuffer(uint8_t *buff, uint32_t length)
{
	while (length > 0)
	{
		arducamUartWrite(*(buff));
		buff++;
		length--;
	}
}

uint8_t arducamUartRead(void)
{
	uint8_t rt = 0;
	rt = uartCommBuff[readBuffLength];
	readBuffLength++;
	if (readBuffLength == uartCommLength)
	{
		readBuffLength = 0;
		uartCommLength = 0;
	}
	return rt;
}

uint32_t arducamUartAvailable(void)
{
	return uartCommLength;
}

void arducamFlush(void)
{
	while (arducamUartAvailable())
	{
		arducamUartRead();
	}
}

#pragma vector=EUSCI_A0_VECTOR
__interrupt void USCI_A0_ISR(void)
{
	switch (__even_in_range(UCA0IV, USCI_UART_UCTXCPTIFG))
	{
		case USCI_NONE:
			break;
		case USCI_UART_UCRXIFG:
			LPM0_EXIT;

			if (UCA0RXBUF == 0x55)
			{
				BaseType_t xHigherPriorityTaskWoken;
				xHigherPriorityTaskWoken = pdFALSE;
				vTaskNotifyGiveFromISR(xHandle, &xHigherPriorityTaskWoken);
				portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
				break;
			}
			uartCommBuff[uartCommLength] = UCA0RXBUF;
			uartCommLength++;
			break;

		case USCI_UART_UCTXIFG:
			break;
		case USCI_UART_UCSTTIFG:
			break;
		case USCI_UART_UCTXCPTIFG:
			break;
		default:
			break;
	}
}

