/*
 * fft.h
 *
 *  Created on: Jul 9, 2024
 *      Author: smkim
 */

#ifndef INC_FFT_H_
#define INC_FFT_H_

#define FFT_LENGTH 2048

#define FFT_DETECT_CNT 5
#define FFT_DETECT_TIME 30
#define FFT_HOLDING_TIME 50

#define BITS_ERR_LOW_WATER 0x0001
#define BITS_ERR_CAVITATION 0x0002
#define BITS_ERR_FROZEN 0x0004
#define BITS_FREQ_DETECT 0x0008

extern void FFT_Timer_Proc();
extern void Timer_Ms_Ticks_Proc(void);

extern uint16_t usPumpOutBuff[6];
extern uint16_t usErrDectBuff[6];
extern float fPowerRmsBuff[4][4];
extern float fMaxBuff[];
extern uint16_t usMaxFreqBuff[];
extern uint16_t usFullRangeCntBuff[4][4];
extern uint16_t usErrCheckBuff[];
extern uint16_t usMagPowerPeakDetectCnt[4];
extern uint16_t usMagPowerPeakDetectBuff[4][FFT_LENGTH];
extern int32_t nMagPowerPeakDetectBuff[4][FFT_LENGTH];


#endif /* INC_FFT_H_ */
