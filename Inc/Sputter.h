#ifndef SPUTTER_HANDLE_DEFINE
#define SPUTTER_HANDLE_DEFINE

#include "stdint.h"

void SputterFunc(void);

typedef struct FailureStatus
{
	uint16_t Com_Failure;
	uint8_t PFC_OK;
	uint8_t DC_OK;
	uint16_t FB_OTP;
	uint8_t Hiccup;
	uint16_t tacho2_Disable;
	unsigned char Inrush_OK;
}FailureStatus_t;

typedef struct FlagsStatus
{
	unsigned int flag_DC_OK;
	unsigned int flag_PFC_OK;
	unsigned int flag_PFC_OTP;
	unsigned int flag_FB_OTP;
	unsigned int flag_Hiccup;
	unsigned int flag_Fan;
}FlagsStatus_t;

#endif /*SPUTTER_HANDLE_DEFINE*/
