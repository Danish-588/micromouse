/*
 * imu.c
 *
 *  Created on: Jun 27, 2021
 *      Author: Soham
 */

#include <main.h>

struct imu
sparton = {&uart5,115200},
arduimu = {&uart4,38400};

uint_least8_t UartWaitRxByte(USART_TypeDef * UART)
{
    uint32_t sr;

    do {
      sr = UART->SR;
      if (sr & USART_SR_ORE) {
        (void)UART->DR;  // in older USARTs with SR, ORE is cleared by reading SR and then DR
      }
    } while (!(sr & USART_SR_RXNE)) ;
    return UART->DR;
}

void arduimu_isr()
{
    uint8_t temp_data = (int)(arduimu.comm_protocol->uart->DR);

    if(temp_data&0x80)
    {
    	temp_angle = (temp_data&0x7F)<<7;
    	next_byte = 1;
    }
    else if(next_byte)
    {
    	combined_angle = temp_angle|temp_data;
    	next_byte = 0;
    	angle = 360-(combined_angle/10.0);
    }
}

void arduimu_poll()
{
	uint8_t temp_data = UartWaitRxByte(arduimu.comm_protocol->uart);

    if(temp_data&0x80)
    {
    	temp_angle = (temp_data&0x7F)<<7;
    	next_byte = 1;
    }
    else if(next_byte)
    {
    	combined_angle = temp_angle|temp_data;
    	next_byte = 0;
    	raw_angle = 360-(combined_angle/10.0);
    }
    else
    {
    	next_byte = 0;
    }
}

void arduimu_init()
{
	temp_angle = 0;
	next_byte = 0;
	combined_angle = 0;
	uart_init(&uart4, 38400, arduimu_isr);
}

void sparton_init()
{
	while_check = 0, can_isr_check = 0, sparton_isr_check = 0;
	i = 0, k1 = 0;
	entryflag1 = 0, entryflag2 = 0, flag_float = 0, once = 1, flt = 0;
	integer = 0;
	floating = 0.0;
	j = 0;
	next = 0;
	isr_check=0;
	go=0,final_data,m=0;
	angle = 0;
	tare_done = 0;
	checksum_received = 0;
	checksum_calculated = 0x2B;

	uart_init(&uart5,115200,sparton2_isr);
	sparton_tare();
	uart_transmit_ascii(sparton.comm_protocol,"$PSRFS,yaw,get,RPT=0.01*31\r\n");
}

void sparton_tare()
{
	uart_transmit_ascii(sparton.comm_protocol,"\r\nInvokeTare 1 set drop\r\n");
	tare_done = 1;
}

void sparton_request_angle()
{
	if(tare_done)
	{
//		uart_transmit_ascii(sparton.comm_protocol,"yaw di.\r\n");
		uart_transmit_ascii(sparton.comm_protocol,"$PSRFS,yaw,get*5D\r\n");
	}
}

void sparton_isr(void)
{
    data_u5 = (int)(sparton.comm_protocol->uart->DR);

    if (entryflag2)
    {
        ascii = data_u5;
        toint = ascii - 48;
        if (flag_float == 1)
        {
            if (ascii != '\r')
            {
                floating = floating * 10 + toint;
                float_data[flt] = ascii;
                flt++;
            }
            else
            {
                floating = floating / (pow(10, flt));
                sparton_angle = integer + floating;
                if (float_data[flt - 4] == 'e')
                {
                    sparton_angle = (sparton_angle)
                            / (pow(10, (float_data[flt - 1] - 48)));
                }

                integer = floating = flag_float = entryflag1 = entryflag2 =
                        toint = flt = 0;
            }
        }

        if ((ascii != '.') && (flag_float == 0))
        {
            integer = integer * 10 + toint;
        }
        else
        {
            flag_float = 1;
        }
    }

    if ((data_u5 == 32) && (entryflag1 == 1))
    {
        entryflag2 = 1;
    }
    else
    {
        entryflag1 = 0;
    }

    if (data_u5 == 61)
    {
        entryflag1 = 1;
    }
    angle = 360.0 - sparton_angle;
    sparton_angle_int = angle;
    sparton_isr_check++;
}

void sparton2_isr(void)
{
    data_char = (char)(sparton.comm_protocol->uart->DR);

    if (entryflag2)
    {
        if (data_char!= '\r')
        {
            flt_data[flt] = data_char;
            flt++;

            if(flt>13)
            {
                entryflag1 = entryflag2 = flt = 0;
                checksum_calculated = 0x2B;
            }
        }
        else
        {
        	for(int i = 0 ; i < (flt-3) ; i++)
        	{
        		checksum_calculated ^= (int)(flt_data[i]);
        	}
        	checksum_received = (flt_data[flt-1]-48) + 16*((flt_data[flt-2]-48));
        	if((checksum_received==checksum_calculated)&&(flt_data[flt-3]=='*'))
        	{
            	sparton_angle = strtof(flt_data,NULL);
            	isr_check++;
        	}
            entryflag1 = entryflag2 = flt = 0;
            checksum_calculated = 0x2B;

        }
    }

    if ((data_char == ',') && (entryflag1 == 1))
    {
        entryflag2 = 1;
    }
    else
    {
        entryflag1 = 0;
    }

    if (data_char == 'w')
    {
        entryflag1 = 1;
    }


    if(sparton_angle>360)
    {
    	angle = 0;
    }
    else if(sparton_angle<0)
    {
    	angle = 360;
    }
    else
    {
        angle = 360.0 - sparton_angle;
    }

    sparton_angle_int = angle;
    sparton_isr_check++;
}
