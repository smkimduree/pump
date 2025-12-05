/*
 * fft.c
 *
 *  Created on: Jul 9, 2024
 *      Author: smkim
 */
#include "arm_math.h"
#include "main.h"
#include "fft.h"

#define POwER_DECT_MAX 2
#define POwER_DECT_MIN 1

volatile float fFftInBuff[FFT_LENGTH];
volatile float fFftOutBuff[4][FFT_LENGTH];
volatile float fMagPowerBuff[4][(FFT_LENGTH/2)];
arm_rfft_fast_instance_f32 fft_handler;
uint16_t usFullRangeCntBuff[4][4];
float fPowerRmsBuff[4][4];
float fMaxBuff[4];
uint16_t usErrCheckBuff[4] ={0,0,0,0};
////////communication///////////////////
uint16_t usPumpOutBuff[6] ={0,0,0,0,0,0};
uint16_t usErrDectBuff[6] ={0,0,0,0,0,0};
////////////////////////////////////////

uint16_t usTimeContinue100msBuff[4] = {0,0,0,0};
uint16_t usTimeChange100msBuff[4] = {0,0,0,0};
uint16_t usTimeDetect100msBuff[4] = {0,0,0,0};
void FFT_Timer_Proc()
{
	if(unTimer1MsTicks>=100)
	{
		unTimer1MsTicks = 0;
		if(usTimeDetect100msBuff[0]>0)
		{
			if(usTimeContinue100msBuff[0] < 6000) usTimeContinue100msBuff[0]++;
		}
		else usTimeContinue100msBuff[0] = 0;
		if(usTimeDetect100msBuff[1]>0)
		{
			if(usTimeContinue100msBuff[1] < 6000) usTimeContinue100msBuff[1]++;
		}
		else usTimeContinue100msBuff[1] = 0;
		if(usTimeDetect100msBuff[2]>0)
		{
			if(usTimeContinue100msBuff[2] < 6000) usTimeContinue100msBuff[2]++;
		}
		else usTimeContinue100msBuff[2] = 0;
		if(usTimeDetect100msBuff[3]>0)
		{
			if(usTimeContinue100msBuff[3] < 6000) usTimeContinue100msBuff[3]++;
		}
		else usTimeContinue100msBuff[3] = 0;

		if(usTimeDetect100msBuff[0] > 0) usTimeDetect100msBuff[0]--;
		if(usTimeDetect100msBuff[1] > 0) usTimeDetect100msBuff[1]--;
		if(usTimeDetect100msBuff[2] > 0) usTimeDetect100msBuff[2]--;
		if(usTimeDetect100msBuff[3] > 0) usTimeDetect100msBuff[3]--;
	}
}


void FFT_Buff_Init()
{
	int i;
	for(i=0;i<FFT_LENGTH;i++)
	{
		fFftOutBuff[0][i] = 0;
		fFftOutBuff[1][i] = 0;
		fFftOutBuff[2][i] = 0;
		fFftOutBuff[3][i] = 0;
		fFftInBuff[i] = 0;
	}
	for(i=0;i<(FFT_LENGTH/2);i++)
	{
		fMagPowerBuff[0][i] = 0;
		fMagPowerBuff[1][i] = 0;
		fMagPowerBuff[2][i] = 0;
		fMagPowerBuff[3][i] = 0;
	}
	arm_rfft_fast_init_f32(&fft_handler, FFT_LENGTH);
}

double dMagPowerAdd[4];
float fMax, fMin, fRms, *fPtr;
float fComp[4];

uint16_t usMaxFreqBuff[4];
uint16_t usDetectBuff[4][5], *usPtr;
void FFT_Check_Proc(uint8_t ch)
{
	uint16_t us_freq = 0;
	us_freq = usFullRangeCntBuff[ch][2] + usFullRangeCntBuff[ch][3];
	if (usPumpOutBuff[ch] > 0)
	{
		if(us_freq > FFT_DETECT_CNT)
		{
			usErrCheckBuff[ch] &= 0xFFF0;
			if((fPowerRmsBuff[ch][2]>10000) || (fPowerRmsBuff[ch][3]>10000))
			{
				usErrCheckBuff[ch] |= (BITS_ERR_LOW_WATER | BITS_FREQ_DETECT);
				usTimeDetect100msBuff[ch] = FFT_DETECT_TIME;
			}
			else
			{
				usErrCheckBuff[ch] |= (BITS_ERR_CAVITATION | BITS_FREQ_DETECT);
				usTimeDetect100msBuff[ch] = FFT_DETECT_TIME;
			}
		}
		else
		{
			if(usTimeDetect100msBuff[ch]==0)
			{
				usErrCheckBuff[ch] = 0x0000;
			}
		}
	}
	else
	{
		if(us_freq > FFT_DETECT_CNT)
		{
			usErrCheckBuff[ch] &= 0xFFF0;
			usErrCheckBuff[ch] |= (BITS_ERR_FROZEN | BITS_FREQ_DETECT);
			usTimeDetect100msBuff[ch] = FFT_DETECT_TIME;
		}
		else
		{
			if(usTimeDetect100msBuff[ch]==0)
			{
				usErrCheckBuff[ch] = 0x0000;
			}
		}
	}

	if(usTimeContinue100msBuff[ch] > FFT_HOLDING_TIME)
	{
		usErrDectBuff[ch] = usErrCheckBuff[ch];
	}
	else usErrDectBuff[ch] = 0;
}

uint8_t ucMagPowerUpFlag;
int32_t nMagPowerSumData=0;
uint16_t usMagPowerPeakDetectCnt[4];
uint16_t usMagPowerPeakDetectBuff[4][FFT_LENGTH];
int32_t nMagPowerPeakDetectBuff[4][FFT_LENGTH];
int32_t nMagPowerSumBuff[5];
uint16_t usMagPowerSumInx;
void FFT_Peak_Detect(uint8_t ch)
{
	int i,j;
	int cnt;
	int32_t val;
	ucMagPowerUpFlag = 1;
	cnt = 0;
	j=0;
	nMagPowerSumData = 0;
	nMagPowerSumBuff[0] = 0;
	nMagPowerSumBuff[1] = 0;
	nMagPowerSumBuff[2] = 0;
	nMagPowerSumBuff[3] = 0;
	nMagPowerSumBuff[4] = 0;
	usMagPowerSumInx = 0;
	for(i=1;i<(FFT_LENGTH/2);i++)
	{
		if(fMagPowerBuff[ch][i] > nMagPowerSumData)
		{
			nMagPowerSumData = fMagPowerBuff[ch][i];
			nMagPowerSumBuff[usMagPowerSumInx] = i*10;
			usMagPowerSumInx++;
			usMagPowerSumInx %= 5;
		}
	}
	i = (usMagPowerSumInx + 4)%5;
	usMagPowerPeakDetectBuff[ch][0] = nMagPowerSumBuff[i];
	i = (usMagPowerSumInx + 3)%5;
	usMagPowerPeakDetectBuff[ch][1] = nMagPowerSumBuff[i];
	i = (usMagPowerSumInx + 2)%5;
	usMagPowerPeakDetectBuff[ch][2] = nMagPowerSumBuff[i];
	i = (usMagPowerSumInx + 1)%5;
	usMagPowerPeakDetectBuff[ch][3] = nMagPowerSumBuff[i];
	i = (usMagPowerSumInx + 0)%5;
	usMagPowerPeakDetectBuff[ch][4] = nMagPowerSumBuff[i];
	cnt = 5;
#if 0
	j=0;
	for(i=1;i<(FFT_LENGTH/2);i++)
	{
		val = fMagPowerBuff[ch][i] - fMagPowerBuff[ch][j];
		if(val > 0)
		{
			ucMagPowerUpFlag = 1;
		}
		else
		{
			if((ucMagPowerUpFlag != 0) && (fMagPowerBuff[ch][j]>=(nMagPowerSumData-100)))
			{
				usMagPowerPeakDetectBuff[ch][cnt] = j;
				nMagPowerPeakDetectBuff[ch][cnt] = fMagPowerBuff[ch][j]*0.1;
				cnt++;
			}
			ucMagPowerUpFlag = 0;
		}
		j++;
	}
#endif
	usMagPowerPeakDetectCnt[ch]=cnt;
}

uint16_t usFftDetectTempBuff[4];
void FFT_Serch(uint8_t ch)
{
	int i,count,j;
	fPtr = &fMagPowerBuff[ch][0];
	usPtr = &usDetectBuff[ch][0];
	dMagPowerAdd[0] = 0;
	dMagPowerAdd[1] = 0;
	dMagPowerAdd[2] = 0;
	dMagPowerAdd[3] = 0;
	fMax = fMin = 0;
	count = 0;
	usPtr[0] = 0;
	usPtr[1] = 0;
	usPtr[2] = 0;
	usPtr[3] = 0;
	usPtr[4] = 0;
	for(i=0;i<(FFT_LENGTH/2);i++)
	{
		if(i<256) dMagPowerAdd[0] += fMagPowerBuff[ch][i];
		else if(i<512) dMagPowerAdd[1] += fMagPowerBuff[ch][i];
		else if(i<768) dMagPowerAdd[2] += fMagPowerBuff[ch][i];
		else dMagPowerAdd[3] += fMagPowerBuff[ch][i];


//		if(fMax<fPtr[i])
		if((fMax<fPtr[i]) && (i>=70))
		{
			fMax = fPtr[i];
			usMaxFreqBuff[ch] = i;
			switch(count)
			{
			case 0:
				usPtr[0] = i+1;
				count = 1;
				break;
			case 1:
				usPtr[1] = usPtr[0];
				usPtr[0] = i+1;
				count = 2;
				break;
			case 2:
				usPtr[2] = usPtr[1];
				usPtr[1] = usPtr[0];
				usPtr[0] = i+1;
				count = 3;
				break;
			case 3:
				usPtr[3] = usPtr[2];
				usPtr[2] = usPtr[1];
				usPtr[1] = usPtr[0];
				usPtr[0] = i+1;
				count = 4;
				break;
			case 4:
				usPtr[4] = usPtr[3];
				usPtr[3] = usPtr[2];
				usPtr[2] = usPtr[1];
				usPtr[1] = usPtr[0];
				usPtr[0] = i+1;
				count = 5;
				break;
			default:
				usPtr[4] = usPtr[3];
				usPtr[3] = usPtr[2];
				usPtr[2] = usPtr[1];
				usPtr[1] = usPtr[0];
				usPtr[0] = i+1;
				break;
			}
		}
		if(fMin>fPtr[i])
		{
			fMin = fPtr[i];
		}
	}

	fPowerRmsBuff[ch][0] = (float)(dMagPowerAdd[0]/(FFT_LENGTH/8));
	fPowerRmsBuff[ch][1] = (float)(dMagPowerAdd[1]/(FFT_LENGTH/8));
	fPowerRmsBuff[ch][2] = (float)(dMagPowerAdd[2]/(FFT_LENGTH/8));
	fPowerRmsBuff[ch][3] = (float)(dMagPowerAdd[3]/(FFT_LENGTH/8));

	fMaxBuff[ch] = fMax;

	fComp[0] = fPowerRmsBuff[ch][0] + 10000;
	fComp[1] = fPowerRmsBuff[ch][1] + 3000;
	fComp[2] = fPowerRmsBuff[ch][2] + 2000;
	fComp[3] = fPowerRmsBuff[ch][3] + 1000;

	usFftDetectTempBuff[0] = 0;
	usFftDetectTempBuff[1] = 0;
	usFftDetectTempBuff[2] = 0;
	usFftDetectTempBuff[3] = 0;

	for(i=0;i<(FFT_LENGTH/2);i++)
	{
		if(i<20)
		{

		}
		else if(i<256)
		{
			if(fComp[0]<fPtr[i]) usFftDetectTempBuff[0]++;
		}
		else if(i<512)
		{
			if(fComp[1]<fPtr[i]) usFftDetectTempBuff[1]++;
		}
		else if(i<768)
		{
			if(fComp[2]<fPtr[i]) usFftDetectTempBuff[2]++;
		}
		else
		{
			if(fComp[3]<fPtr[i]) usFftDetectTempBuff[3]++;
		}
	}

	usFullRangeCntBuff[ch][0] = usFftDetectTempBuff[0];
	usFullRangeCntBuff[ch][1] = usFftDetectTempBuff[1];
	usFullRangeCntBuff[ch][2] = usFftDetectTempBuff[2];
	usFullRangeCntBuff[ch][3] = usFftDetectTempBuff[3];

	FFT_Check_Proc(ch);
}

/*
uint16_t usMaxDectInx[4];
uint16_t usMaxDectBuff[4][100];
uint8_t ucMaxDectSendFlag = 0;
volatile float fPowerMaxBuff[4][100];
void FFT_Detect(uint8_t ch)
{
	int i,j,inx;
	inx = 0;
	for(i=0;i<(FFT_LENGTH/2);i++)
	{
		if(i%10)
		{
			if(i!=0) inx++;
			if(inx == 100) break;
			fPowerMaxBuff[ch][inx] = fMagPowerBuff[ch][i];
			usMaxDectInx = i;
			usMaxDectBuff[ch][inx] = i;
		}
		else
		{
			if(fMagPowerBuff[ch][i]>fMagPowerBuff[ch][usMaxDectInx])
			{
				usMaxDectInx = i;
				usMaxDectBuff[ch][inx] = i;
			}
			fPowerMaxBuff[ch][inx] += fMagPowerBuff[ch][i];
		}
	}
	usMaxDectInx = 0;
	usMaxDectBuff[ch][0] = usMaxDectInx;
	for(i=1;i<100;i++)
	{
		if(fPowerMaxBuff[ch][i]>usMaxDectBuff[ch][usMaxDectInx]) usMaxDectBuff[ch][usMaxDectInx] = fPowerMaxBuff[ch][inx];
	}
}
*/

void FFT_Proc(uint8_t ch,uint16_t *src)
{
	int i;
	for(i=0;i<FFT_LENGTH;i++)	fFftInBuff[i] = (float)src[i];
	/* Perform forward direction 32-bit FFT */
	arm_rfft_fast_f32(&fft_handler, fFftInBuff, (float *)&fFftOutBuff[ch][0], 0);
	/* Calculate magnitude (buffer size is half because real + imag parts are merged) */
	arm_cmplx_mag_f32((float*)&fFftOutBuff[ch][0], &fMagPowerBuff[ch][0], FFT_LENGTH/2);
//	FFT_Serch(ch);
	FFT_Peak_Detect(ch);
}
