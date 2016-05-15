#define F_CPU 1000000UL
#define USART_BAUDRATE 2400
#define BAUD_PRESCALE (((F_CPU / (USART_BAUDRATE * 16UL))) - 1)

#include<avr/io.h>
#include<util/delay.h>
#include<avr/interrupt.h>
#include "nrf24L01.h"

#define W 1
#define R 0
#define t 1
#define r 0
#define low 0
#define high 1



void csn(int value)
{
	if(value)
		PORTB|=(1<<4);
	if(!value)
		PORTB&=~(1<<4);

}

void ce(int value)
{
	if(value)
		PORTB|=(1<<3);
	if(!value)
		PORTB&=~(1<<3);

}


void spi_init(void)
{
	DDRB|=(1<<DDB4)|(1<<DDB5)|(1<<DDB3)|(1<<DDB7);//master mode;
	SPCR|=(1<<SPE)|(1<<MSTR)|(1<<SPR0);//fclk/16
	csn(high);//CSN
	ce(low);//CE
}


char write_byte_spi(unsigned char cdata)
{
	SPDR=cdata;
	while(!(SPSR&(1<<SPIF)));
	return SPDR;
}

uint8_t get_reg(uint8_t reg)
{
	_delay_us(10);
	csn(low);
	_delay_us(10);
	write_byte_spi(R_REGISTER + reg);
	_delay_us(10);
	reg=write_byte_spi(NOP);
	_delay_us(10);
	csn(high);
	return reg;
}

uint8_t write_to_nrf(uint8_t rw,uint8_t reg, uint8_t *val, uint8_t antval)
{
	if(rw==W)
	reg=W_REGISTER+reg;
	
	static uint8_t ret[32];
	_delay_us(10);
	csn(low);
	_delay_us(10);
	write_byte_spi(reg);
	_delay_us(10);
	
	int i;
	for(i=0;i<antval;i++)
	{
		if( (rw==R) && (reg!=W_TX_PAYLOAD))
		{
			ret[i]=write_byte_spi(NOP);
			_delay_us(10);
		}
		else
		{
			write_byte_spi(val[i]);
			_delay_us(10);
		}
	}
	csn(high);
	return ret[0];
}


void rf_setup(void)
{
	uint8_t val[1];
	val[0]=0x27;
	write_to_nrf(W,RF_SETUP,val,1);
	_delay_us(10);
}




void setup_aw(uint8_t address_length)//1
{
	uint8_t val[1];
	val[0]=(address_length-0x02);
	write_to_nrf(W,SETUP_AW,val,1);
	_delay_us(10);

}

void max_retries(uint8_t delay,uint8_t retries )//2
{
	uint8_t val[1];
	val[0]=(delay<<4)|(retries);
	write_to_nrf(W,SETUP_RETR,val,1);
	_delay_us(10);

}

void rf_ch(uint8_t channel)//3
{
	uint8_t val[1];

	val[0]=channel;
	write_to_nrf(W,RF_CH,val,1);
	_delay_us(10);

}


void en_dynamic_payload(uint8_t pipe_no)//4
{

	uint8_t val[1];
	
	val[0]=0x40;
	write_to_nrf(W,FEATURE,val,1);
	
	val[0]=(1<<pipe_no);
	write_to_nrf(W,DYNPD,val,1);
	_delay_us(10);
	
	
}

void en_aa(uint8_t pipe_no)//5
{
	uint8_t val[1];
	
	val[0]=pipe_no;
	write_to_nrf(W,EN_AA,val,1);
	_delay_us(10);
}

void en_rx_address(uint8_t pipe_no)//6
{
	uint8_t val[1];
	
	val[0]=pipe_no;
	write_to_nrf(W,EN_RXADDR,val,1);
	_delay_us(10);
}

void tx_address(uint8_t *address,uint8_t address_length)//7
{
	write_to_nrf(W,TX_ADDR,address,address_length);
}

void rx_address(uint8_t pipe_no,uint8_t *address,uint8_t address_length)//8
{
	uint8_t rx_address_px;
	
	if(pipe_no==0x00)
	rx_address_px=0x0A;
	
	else if(pipe_no==0x01)
	rx_address_px=0x0B;
	
	else if(pipe_no==0x02)
	rx_address_px=0x0C;
	
	else if(pipe_no==0x03)
	rx_address_px=0x0D;
	
	else if(pipe_no==0x04)
	rx_address_px=0x0E;
	
	else
	rx_address_px=0x0F;
	
	
	write_to_nrf(W,rx_address_px,address,address_length);

	_delay_us(10);	
}

void receiver_payload_width(uint8_t pipe_no, uint8_t no_of_bytes)//9
{
	uint8_t rx_address_px;
	
	if(pipe_no==0x00)
	rx_address_px=0x11;
	
	else if(pipe_no==0x01)
	rx_address_px=0x12;
	
	else if(pipe_no==0x02)
	rx_address_px=0x13;
	
	else if(pipe_no==0x03)
	rx_address_px=0x14;
	
	else if(pipe_no==0x04)
	rx_address_px=0x15;
	
	else
	rx_address_px=0x16;
	
	uint8_t val[1];
	val[0]=no_of_bytes;

	write_to_nrf(W,rx_address_px,val,1);

	_delay_us(10);

}


/****************STATUS********************/

int rx_dr(void)//10
{
	if(get_reg(STATUS)&(1<<6))
	return 1;
	return 0;

}

int tx_ds(void)//11
{
	if(get_reg(STATUS)&(1<<5))
	return 1;
	return 0;
}

int max_rt(void)//12
{
	if(get_reg(STATUS)&(1<<4))
	return 1;
	return 0;
}


uint8_t rx_p_no(void)//13
{
	uint8_t data;
	data=get_reg(STATUS);
	data&=0x0E;
	return (data>>1);
	
	
}

void reset_status(void)//14
{
	uint8_t data[1];
	data[0]=get_reg(STATUS);
	data[0]=data[0]|0x70;
	write_to_nrf(W,STATUS,data,1);
}

/****************FIFO*********************/

int tx_full(void)//15
{
	if(get_reg(FIFO_STATUS)&(1<<5))
	return 1;
	return 0;
}

int tx_empty(void)//16
{
	if(get_reg(FIFO_STATUS)&(1<<4))
	return 1;
	return 0;
}

int rx_full(void)//17
{
	if(get_reg(FIFO_STATUS)&(1<<1))
	return 1;
	return 0;
}

int rx_empty(void)//18
{
	if(get_reg(FIFO_STATUS)&(1<<0))
	return 1;
	return 0;
}




void config(int d)//19
{

	
	uint8_t val[5];
	
	if(d==t)
	{
	
	val[0]=0x1E;
	write_to_nrf(W,CONFIG,val,1);
	
	_delay_us(130);
	}
	else
	{
	
	val[0]=0x1F;
	write_to_nrf(W,CONFIG,val,1);
	
	_delay_us(130);
	}
}


void flush_tx(void)//20
{
	uint8_t clear[1];
	clear[0]=0x00;
	write_to_nrf(R,FLUSH_TX,clear,0);
	_delay_us(10);
}
void write_payload(uint8_t *w_buff,uint8_t data_length)
{
	flush_tx();
	write_to_nrf(R,W_TX_PAYLOAD,w_buff,data_length);
	
	_delay_us(10);
	
}

uint8_t read_pl_wid(void)//21
{
	uint8_t val[1];
	val[0]=write_to_nrf(R,R_RX_PL_WID,val,1);
	_delay_us(10);
	return val[0];
}
void flush_rx(void)
{
	uint8_t clear[1];
	
	write_to_nrf(R,FLUSH_RX,clear,0);
	_delay_us(10);

}
uint8_t read_payload(void)
{
	uint8_t val[5];
	uint8_t length;
	length=read_pl_wid();
	return write_to_nrf(R,R_RX_PAYLOAD,val,length);

}


void start_listening(void)//22
{
	
	uint8_t val[5];
	val[0]=0x00;
	write_to_nrf(W,FLUSH_RX,val,0);
	
	PORTB|=(1<<PORTB3);
	_delay_us(130);

}

void stop_listening(void)//23
{
	PORTB&=~(1<<PORTB3);
	
	_delay_us(130);
	
}

void start_sending(void)//24
{
	ce(high);
	_delay_us(130);
}

void stop_sending(void)//25
{
	ce(low);
	_delay_us(10);
}