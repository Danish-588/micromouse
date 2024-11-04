/*
 * utils.c
 *
 *  Created on: 24-May-2023
 *      Author: Soham
 */


float constrain(float amt,float low,float high)
{
	if(amt<low)
	{
		amt = low;
	}
	else if(amt>high)
	{
		amt = high;
	}
	return amt;
}

float forceconstrain(float *amt,float low,float high)
{
	if(*amt<low)
	{
		*amt = low;
	}
	else if(*amt>high)
	{
		*amt = high;
	}
	return *amt;
}
