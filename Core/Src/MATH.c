/*
 * File: MATH.c
 * Driver Name: [[ MATH Functions ]]
 * SW Layer:   ECUAL
 * Created on: Jun 28, 2020
 * Author:     Khaled Magdy
 * -------------------------------------------
 * For More Information, Tutorials, etc.
 * Visit Website: www.DeepBlueMbedded.com
 *
 */

#include "MATH.h"

uint32_t MAP(uint32_t au32_IN, uint32_t au32_INmin, uint32_t au32_INmax, uint32_t au32_OUTmin, uint32_t au32_OUTmax)
{
	return ((((au32_IN - au32_INmin)*(au32_OUTmax - au32_OUTmin))/(au32_INmax - au32_INmin)) + au32_OUTmin);
}

uint32_t Constrain(uint32_t au32_IN, uint32_t au32_MIN, uint32_t au32_MAX)
{
	if(au32_IN < au32_MIN)
	{
		return au32_MIN;
	}
	else if (au32_IN > au32_MAX)
	{
		return au32_MAX;
	}
	else
	{
		return au32_IN;
	}
}

MATH_DataType MIN(MATH_DataType* IN_Arr, uint32_t au32_LEN)
{
	uint32_t i = 0;
	MATH_DataType MIN = 0;

	for(i=0; i<au32_LEN; i++)
	{
		if(IN_Arr[i] < MIN)
		{
			MIN = IN_Arr[i];
		}
	}
	return MIN;
}

MATH_DataType MAX(MATH_DataType* IN_Arr, uint32_t au32_LEN)
{
	uint32_t i = 0;
	MATH_DataType MAX = 0;

	for(i=0; i<au32_LEN; i++)
	{
		if(IN_Arr[i] > MAX)
		{
			MAX = IN_Arr[i];
		}
	}
	return MAX;
}

uint32_t u8_to_u32(const uint8_t *bytes)
{
	//Every uint32_t consist of 4 bytes, therefore we can shift each uint8_t
	//to an appropriate location.
	//u32   ff  ff   ff   ff
	//u8[]   0   1    2    3
	uint32_t u32 = (bytes[0] << 24) + (bytes[1] << 16) + (bytes[2] << 8) + bytes[3];
	return u32;
}

uint8_t* u32_to_u8(const uint32_t u32, uint8_t* u8)
{
	//To extract each byte, we can mask them using bitwise AND (&)
	//then shift them right to the first byte.
	u8[0] = (u32 & 0xff000000) >> 24;
	u8[1] = (u32 & 0x00ff0000) >> 16;
	u8[2] = (u32 & 0x0000ff00) >> 8;
	u8[3] = (u32 & 0x000000ff);
}
