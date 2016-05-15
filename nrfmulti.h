

#include<avr/io.h>
#include<util/delay.h>
#include<avr/interrupt.h>
#include "nrf24L01.h"
#include"String.h"

#define nrfWrite 1
#define nrfRead 0
#define nrfTransmitter 1
#define nrfReceiver 0
#define low 0
#define high 1
#define RF_250kbps 1
#define RF_1Mbps 2
#define RF_2Mbps 3

void csn(int value);
void ce(int value);
void spi_init(void);
char write_byte_spi(unsigned char cdata);
uint8_t get_reg(uint8_t reg);
void write_to_nrf(uint8_t rw,uint8_t reg, uint8_t *val, int antval);
void rf_setup(int RFDataRate);
void rf_power(int i);
void setup_aw(uint8_t nrf_address_length);
void setup_retries(uint8_t delay,uint8_t retries );
void rf_ch(uint8_t channel);
void en_dynamic_payload(int pipe_no);
void dis_dynamic_payload(int pipe_no);
void en_aa(uint8_t pipe_no);
void dis_aa(int pipe_no);
void en_rx_address(uint8_t pipe_no);
void dis_rx_address(uint8_t pipe_no);
void tx_address(uint8_t *transmitter_address,uint8_t nrf_address_length);
void rx_address(uint8_t pipe_no,uint8_t *address,uint8_t address_length);
void receiver_payload_width(uint8_t pipe_no, uint8_t no_of_bytes);
int rx_dr(void);
int tx_ds(void);
int max_rt(void);
uint8_t rx_p_no(void);
void reset_status(void);
void reset_rx_dr();
void reset_max_rt();
void reset_tx_ds();
int tx_full(void);
int tx_empty(void);
int rx_full(void);
int rx_empty(void);
void power_down();
void power_up();
void config(int d);
void flush_tx(void);
void write_payload(uint8_t *w_buff,int data_length);
uint8_t read_pl_wid(void);
void flush_rx(void);
void read_payload(uint8_t *nrfReadPayload);
void start_listening(void);
void stop_listening(void);
void start_sending(uint8_t *nrfData);
void stop_sending(void);
void unmask_rx_dr(void);
void unmask_tx_ds(void);
void unmask_max_rt(void);
void en_ack_pay();
void dis_ack_pay();
void write_payload_with_ack(uint8_t *nrfAckPayload,int payloadLength);
void reuse_tx_pl(void);
void nrf_reset();
int nrf_check_after_listening();
int nrf_check_before_listening();


volatile uint8_t val[6];

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

void write_to_nrf(uint8_t rw,uint8_t reg, uint8_t *value, int antval)
{
	if(rw==nrfWrite)
		reg=W_REGISTER+reg;
	
	_delay_us(10);
	csn(low);
	_delay_us(10);
	write_byte_spi(reg);
	_delay_us(10);
	
	int i;
	for(i=0;i<antval;i++)
	{
		if( (rw==nrfRead) && (reg!=W_TX_PAYLOAD))
		{
			value[i]=write_byte_spi(NOP);
			_delay_us(10);
		}
		else
		{
			write_byte_spi(value[i]);
			_delay_us(10);
		}
	}
	csn(high);
	return;
}


void rf_setup(int RFDataRate)
{
	val[0]=get_reg(RF_SETUP)&(~(1<<6));
	write_to_nrf(nrfWrite,RF_SETUP,val,1);
	
	if(RFDataRate==RF_250kbps){
		val[0]=get_reg(RF_SETUP)|(1<<5);
		val[0]&=~(1<<3);
	}
	else if(RFDataRate==RF_1Mbps){
		val[0]=get_reg(RF_SETUP);
		val[0]&=~((1<<3)|(1<<5));
	}
	else if(RFDataRate==RF_2Mbps){
		val[0]=get_reg(RF_SETUP)|(1<<3);
		val[0]&=~(1<<5);
	}
	else {
		val[0]=get_reg(RF_SETUP)|(1<<3);
		val[0]&=~(1<<5);
	}
		
	
	write_to_nrf(nrfWrite,RF_SETUP,val,1);
}

void rf_power(int i)
{
	switch(i)
	{
		case 0:
				val[0]=get_reg(RF_SETUP)&(~((1<<2)|(1<<1)));
				break;
		case 1:
				val[0]=get_reg(RF_SETUP)|((1<<1)&(~(1<<2)));
				break;
		case 2:
				val[0]=get_reg(RF_SETUP)|((1<<2)&(~(1<<1)));	
				break;
		case 3:
				val[0]=get_reg(RF_SETUP)|((1<<2)|(1<<1));
				break;
		default:
				val[0]=get_reg(RF_SETUP)|((1<<2)|(1<<1));
				break;
	}
	write_to_nrf(nrfWrite,RF_SETUP,val,1);
}



void setup_aw(uint8_t nrf_address_length)//1
{
	if(nrf_address_length>5)
		nrf_address_length=5;
	else if(nrf_address_length<3)
		nrf_address_length=3;
	else
		val[0]=(nrf_address_length-2);
	
	write_to_nrf(nrfWrite,SETUP_AW,val,1);

}

void setup_retries(uint8_t delay,uint8_t retries )//2
{
	if(delay>15)
		delay=15;
	else if(retries>15)
		retries=15;
	
	val[0]=(delay<<4)|(retries);
	write_to_nrf(nrfWrite,SETUP_RETR,val,1);
}

void rf_ch(uint8_t channel)//3
{
	if(channel>125)
		channel=125;
	val[0]=channel;
	write_to_nrf(nrfWrite,RF_CH,val,1);
}


void en_dynamic_payload(int pipe_no)//4
{
	if(pipe_no<=5)
	{
		val[0]=0x04;
		write_to_nrf(nrfWrite,FEATURE,val,1);
		
		val[0]=get_reg(DYNPD)|(1<<pipe_no);
		write_to_nrf(nrfWrite,DYNPD,val,1);
	}
}

// void dis_dynamic_payload(int pipe_no)
// {
	// if(pipe_no<=5)
	// {
		// val[0]=0x04;
		// write_to_nrf(nrfWrite,FEATURE,val,1);
		
		// val[0]=get_reg(DYNPD)&(~(1<<pipe_no));
		// write_to_nrf(nrfWrite,DYNPD,val,1);
	// }
// }

void en_aa(uint8_t pipe_no)//5
{
	if(pipe_no<=5)
	{
		val[0]=get_reg(EN_AA)|(1<<pipe_no);
		write_to_nrf(nrfWrite,EN_AA,val,1);
	}
}

// void dis_aa(int pipe_no)
// {
	// if(pipe_no<=5)
	// {
		// val[0]=get_reg(EN_AA)&(~(1<<pipe_no));
		// write_to_nrf(nrfWrite,EN_AA,val,1);
	// }
// }

void en_rx_address(uint8_t pipe_no)//6
{
	if(pipe_no<=5)
	{
		val[0]=get_reg(EN_RXADDR)|(1<<pipe_no);
		write_to_nrf(nrfWrite,EN_RXADDR,val,1);
		_delay_us(10);
	}
}

// void dis_rx_address(uint8_t pipe_no)
// {
	// if(pipe_no<=5)
	// {
		// val[0]=get_reg(EN_RXADDR)&(~(1<<pipe_no));
		// write_to_nrf(nrfWrite,EN_RXADDR,val,1);
	// }
// }

void tx_address(uint8_t *transmitter_address,uint8_t nrf_address_length)//7
{
	write_to_nrf(nrfWrite,TX_ADDR,transmitter_address,nrf_address_length);
}

void rx_address(uint8_t pipe_no,uint8_t *address,uint8_t address_length)//8
{
	uint8_t rx_address_px;
	
	if(pipe_no<=5)
	{
		rx_address_px=0x0A+pipe_no;	
		write_to_nrf(nrfWrite,rx_address_px,address,address_length);
	}
}

// void receiver_payload_width(uint8_t pipe_no, uint8_t no_of_bytes)//9
// {
	// uint8_t rx_address_px;
	
	// if(pipe_no<=5)
	// {
		// rx_address_px=0x11+pipe_no;	
		// val[0]=no_of_bytes;
		// write_to_nrf(nrfWrite,rx_address_px,val,1);
	// }
// }


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


// uint8_t rx_p_no(void)//13
// {
	// val[0]=get_reg(STATUS)&0x0E;
	// return (val[0]>>1);
// }

void reset_status(void)//14
{
	val[0]=get_reg(STATUS)|0x70;
	write_to_nrf(nrfWrite,STATUS,val,1);
}

void reset_rx_dr()
{
	val[0]=get_reg(STATUS)|(1<<6);
	write_to_nrf(nrfWrite,STATUS,val,1);
}

void reset_tx_ds()
{
	val[0]=get_reg(STATUS)|(1<<5);
	write_to_nrf(nrfWrite,STATUS,val,1);
}

void reset_max_rt()
{
	val[0]=get_reg(STATUS)|(1<<4);
	write_to_nrf(nrfWrite,STATUS,val,1);
}

/****************FIFO*********************/

// int tx_full(void)//15
// {
	// if(get_reg(FIFO_STATUS)&(1<<5))
		// return 1;
	// return 0;
// }

// int tx_empty(void)//16
// {
	// if(get_reg(FIFO_STATUS)&(1<<4))
		// return 1;
	// return 0;
// }

// int rx_full(void)//17
// {
	// if(get_reg(FIFO_STATUS)&(1<<1))
		// return 1;
	// return 0;
// }

// int rx_empty(void)//18
// {
	// if(get_reg(FIFO_STATUS)&(1<<0))
		// return 1;
	// return 0;
// }


void power_up()
{
	val[0]=get_reg(CONFIG)|(1<<1);
	write_to_nrf(nrfWrite,CONFIG,val,1);
	_delay_us(1500);
}

void power_down()
{
	val[0]=get_reg(CONFIG)&(~(1<<1));
	write_to_nrf(nrfWrite,CONFIG,val,1);
	_delay_us(1500);
}
	
	

void config(int d)//19
{
	if(d==nrfTransmitter)
	{
		val[0]=get_reg(CONFIG)&(~(1<<0));
		write_to_nrf(nrfWrite,CONFIG,val,1);
	}
	else
	{
		val[0]=get_reg(CONFIG)|(1<<0);
		write_to_nrf(nrfWrite,CONFIG,val,1);
	}
}


void flush_tx(void)//20
{
	val[0]=0x00;
	write_to_nrf(nrfRead,FLUSH_TX,val,0);
}

void write_payload(uint8_t *w_buff,int data_length)
{
	if(data_length>31)
		data_length=31;
	write_to_nrf(nrfRead,W_TX_PAYLOAD,w_buff,data_length);
}

uint8_t read_pl_wid(void)//21
{
	write_to_nrf(nrfRead,R_RX_PL_WID,val,1);
	return val[0];
}
void flush_rx(void)
{
	write_to_nrf(nrfRead,FLUSH_RX,val,0);
}

void read_payload(uint8_t *nrfReadPayload)
{
	write_to_nrf(nrfRead,R_RX_PAYLOAD,nrfReadPayload,read_pl_wid());
}


void start_listening(void)//22
{
	stop_sending();
	flush_rx();
	flush_tx();
	reset_status();
	config(nrfReceiver);
	// power_up();
	ce(high);
	_delay_us(130);
}

void stop_listening(void)//23
{
	ce(low);
	// power_down();
	_delay_us(10);
}

void start_sending(uint8_t *nrfData)//24
{
	stop_listening();
	flush_tx();
	flush_rx();
	reset_status();
	config(nrfTransmitter);
	write_payload(nrfData,StringLength(nrfData));
	// power_up();
	ce(high);
	_delay_us(130);
}

void stop_sending(void)//25
{
	ce(low);
	// power_down();
	_delay_us(10);
}

void unmask_rx_dr(void)
{
	val[0]=get_reg(CONFIG)&(~(1<<6));
	write_to_nrf(nrfWrite,CONFIG,val,1);
}

void unmask_tx_ds(void)
{
	val[0]=get_reg(CONFIG)&(~(1<<5));
	write_to_nrf(nrfWrite,CONFIG,val,1);
}

void unmask_max_rt(void)
{
	val[0]=get_reg(CONFIG)&(~(1<<4));
	write_to_nrf(nrfWrite,CONFIG,val,1);
}

// void en_ack_pay()
// {
	// val[0]=get_reg(FEATURE)|(1<<1);
	// write_to_nrf(nrfWrite,FEATURE,val,1);
// }

// void dis_ack_pay()
// {
	// val[0]=get_reg(FEATURE)&(~(1<<1));
	// write_to_nrf(nrfWrite,FEATURE,val,1);
// }

// void write_payload_with_ack(uint8_t *nrfAckPayload,int payloadLength)
// {
	// en_dynamic_payload(1);
	// en_ack_pay();
	// write_to_nrf(nrfWrite,W_ACK_PAYLOAD,nrfAckPayload,payloadLength);
// }

// void reuse_tx_pl(void)
// {
	// val[0]=0;
	// write_to_nrf(nrfRead,REUSE_TX_PL,val,0);
// }


void nrf_init()
{
	spi_init();
	
	flush_rx();
	flush_tx();
	val[0]=0x0D;
	write_to_nrf(nrfWrite,CONFIG,val,1);
	
	val[0]=0x3F;
	write_to_nrf(nrfWrite,EN_AA,val,1);
	
	val[0]=0x03;
	write_to_nrf(nrfWrite,EN_RXADDR,val,1);
	
	val[0]=0x03;
	write_to_nrf(nrfWrite,SETUP_AW,val,1);
	
	val[0]=0x3F;
	write_to_nrf(nrfWrite,SETUP_RETR,val,1);
	
	val[0]=0x02;
	write_to_nrf(nrfWrite,RF_CH,val,1);
	
	val[0]=0x0E;
	write_to_nrf(nrfWrite,RF_SETUP,val,1);
	
	val[0]=0x0E;
	write_to_nrf(nrfWrite,STATUS,val,1);
	
	int nrfI;
	for(nrfI=0;nrfI<5;nrfI++)
		val[nrfI]=0xE7;
	write_to_nrf(nrfWrite,RX_ADDR_P0,val,5);
	write_to_nrf(nrfWrite,TX_ADDR,val,5);
	
	for(nrfI=0;nrfI<5;nrfI++)
		val[nrfI]=0xC2;
	write_to_nrf(nrfWrite,RX_ADDR_P1,val,5);
	
	val[0]=0x00;
	write_to_nrf(nrfWrite,DYNPD,val,1);
	
	val[0]=0x00;
	write_to_nrf(nrfWrite,FEATURE,val,1);
	
	rf_setup(RF_250kbps);
}

void nrf_reset()
{
	spi_init();
	
	flush_rx();
	flush_tx();
	unmask_max_rt();
	unmask_rx_dr();
	unmask_tx_ds();
	// power_down();
	power_up();
	config(nrfReceiver);
	en_aa(0);
	en_aa(1);
	en_rx_address(0);
	en_rx_address(1);
	setup_aw(5);
	setup_retries(3,15);
	rf_ch(2);
	rf_setup(RF_250kbps);
	rf_power(3);
	reset_status();
	val[0]=0xE7;
	val[1]=0xE7;
	val[2]=0xE7;
	val[3]=0xE7;
	val[4]=0xE7;
	write_to_nrf(nrfWrite,RX_ADDR_P0,val,5);
	write_to_nrf(nrfWrite,TX_ADDR,val,5);
	val[0]=0xC2;
	val[1]=0xC2;
	val[2]=0xC2;
	val[3]=0xC2;
	val[4]=0xC2;
	write_to_nrf(nrfWrite,RX_ADDR_P1,val,5);
	en_dynamic_payload(0);
	en_dynamic_payload(1);
}

void nrf_print_status()
{
		usart_puts("\nCONFIG:");
		usart_putbyte(get_reg(CONFIG));
		usart_puts("\nEN_AA:");
		usart_putbyte(get_reg(EN_AA));
		usart_puts("\nEN_RXADDR:");
		usart_putbyte(get_reg(EN_RXADDR));
		usart_puts("\nSETUP_AW:");
		usart_putbyte(get_reg(SETUP_AW));
		usart_puts("\nSETUP_RETR:");
		usart_putbyte(get_reg(SETUP_RETR));
		usart_puts("\nRF_CH:");
		usart_putbyte(get_reg(RF_CH));
		usart_puts("\nRF_SETUP:");
		usart_putbyte(get_reg(RF_SETUP));
		usart_puts("\nSTATUS:");
		usart_putbyte(get_reg(STATUS));
		usart_puts("\nRX_ADDR_P0:");
		usart_putbyte(get_reg(RX_ADDR_P0));
		usart_puts("\nRX_ADDR_P1:");
		usart_putbyte(get_reg(RX_ADDR_P1));
		usart_puts("\nTX_ADDR:");
		usart_putbyte(get_reg(TX_ADDR));
		usart_puts("\nDYNPD:");
		usart_putbyte(get_reg(DYNPD));
		usart_puts("\nFEATURE:");
		usart_putbyte(get_reg(FEATURE));
}

int nrf_check_before_listening(){
	if(get_reg(CONFIG)!=0x0F){
		usart_puts("\nnrf pre-listening check failed");
		usart_puts("\nCONFIG:");
		usart_putbyte(get_reg(CONFIG));
		usart_putch('\n');
		return 0;
	}
	else if(get_reg(EN_AA)!=0x3F){
		usart_puts("\nnrf pre-listening check failed");
		usart_puts("\nEN_AA:");
		usart_putbyte(get_reg(EN_AA));
		usart_putch('\n');
		return 0;
	}
	else if(get_reg(EN_RXADDR)!=0x03){
		usart_puts("\nnrf pre-listening check failed");
		usart_puts("\nEN_RXADDR:");
		usart_putbyte(get_reg(EN_RXADDR));
		usart_putch('\n');
		return 0;
	}
	else if(get_reg(SETUP_AW)!=0x03){
		usart_puts("\nnrf pre-listening check failed");
		usart_puts("\nSETUP_AW:");
		usart_putbyte(get_reg(SETUP_AW));
		usart_putch('\n');
		return 0;
	}
	else if(get_reg(SETUP_RETR)!=0x3F){
		usart_puts("\nnrf pre-listening check failed");
		usart_puts("\nSETUP_RETR:");
		usart_putbyte(get_reg(SETUP_RETR));
		usart_putch('\n');
		return 0;
	}
	else if(get_reg(RF_CH)!=0x02){
		usart_puts("\nnrf pre-listening check failed");
		usart_puts("\nRF_CH:");
		usart_putbyte(get_reg(RF_CH));
		usart_putch('\n');
		return 0;
	}
	else if(get_reg(RF_SETUP)!=0x26){
		usart_puts("\nnrf pre-listening check failed");
		usart_puts("\nRF_SETUP:");
		usart_putbyte(get_reg(RF_SETUP));
		usart_putch('\n');
		return 0;
	}
	else if(get_reg(STATUS)!=0X0E){
		usart_puts("\nnrf pre-listening check failed");
		usart_puts("\nSTATUS:");
		usart_putbyte(get_reg(STATUS));
		usart_putch('\n');
		return 0;
	}
	else if(get_reg(RX_ADDR_P0)!=0XE7){
		usart_puts("\nnrf pre-listening check failed");
		usart_puts("\nRX_ADDR_P0:");
		usart_putbyte(get_reg(RX_ADDR_P0));
		usart_putch('\n');
		return 0;
	}
	else if(get_reg(RX_ADDR_P1)!=0XC2){
		usart_puts("\nnrf pre-listening check failed");
		usart_puts("\nRX_ADDR_P1:");
		usart_putbyte(get_reg(RX_ADDR_P1));
		usart_putch('\n');
		return 0;
	}
	
	else if(get_reg(TX_ADDR)!=0XE7){
		usart_puts("\nnrf pre-listening check failed");
		usart_puts("\nTX_ADDR:");
		usart_putbyte(get_reg(TX_ADDR));
		usart_putch('\n');
		return 0;
	}
	else if(get_reg(DYNPD)!=0X03){
		usart_puts("\nnrf pre-listening check failed");
		usart_puts("\nDYNPD:");
		usart_putbyte(get_reg(DYNPD));
		usart_putch('\n');
		return 0;
	}
	else if(get_reg(FEATURE)!=0X04){
		usart_puts("\nnrf pre-listening check failed");
		usart_puts("\nFEATURE:");
		usart_putbyte(get_reg(FEATURE));
		usart_putch('\n');
		return 0;
	}
	else{
		usart_puts("\nnrf pre-listening check completed successfully\n");
		return 1;
	}
}

int nrf_check_after_listening(){
	if(get_reg(CONFIG)!=0x0F){
		usart_puts("\nnrf post-listening check failed");
		usart_puts("\nCONFIG:");
		usart_putbyte(get_reg(CONFIG));
		usart_putch('\n');
		return 0;
	}
	else if(get_reg(EN_AA)!=0x3F){
		usart_puts("\nnrf post-listening check failed");
		usart_puts("\nEN_AA:");
		usart_putbyte(get_reg(EN_AA));
		usart_putch('\n');
		return 0;
	}
	else if(get_reg(EN_RXADDR)!=0x03){
		usart_puts("\nnrf post-listening check failed");
		usart_puts("\nEN_RXADDR:");
		usart_putbyte(get_reg(EN_RXADDR));
		usart_putch('\n');
		return 0;
	}
	else if(get_reg(SETUP_AW)!=0x03){
		usart_puts("\nnrf post-listening check failed");
		usart_puts("\nSETUP_AW:");
		usart_putbyte(get_reg(SETUP_AW));
		usart_putch('\n');
		return 0;
	}
	else if(get_reg(SETUP_RETR)!=0x3F){
		usart_puts("\nnrf post-listening check failed\n");
		usart_puts("\nSETUP_RETR:");
		usart_putbyte(get_reg(SETUP_RETR));
		usart_putch('\n');
		return 0;
	}
	else if(get_reg(RF_CH)!=0x02){
		usart_puts("\nnrf post-listening check failed");
		usart_puts("\nRF_CH:");
		usart_putbyte(get_reg(RF_CH));
		usart_putch('\n');
		return 0;
	}
	else if(get_reg(RF_SETUP)!=0x26){
		usart_puts("\nnrf post-listening check failed");
		usart_puts("\nRF_SETUP:");
		usart_putbyte(get_reg(RF_SETUP));
		usart_putch('\n');
		return 0;
	}
	else if(get_reg(STATUS)!=0X0E){
		usart_puts("\nnrf post-listening check failed");
		usart_puts("\nSTATUS:");
		usart_putbyte(get_reg(STATUS));
		usart_putch('\n');
		return 0;
	}
	else if(get_reg(RX_ADDR_P0)!=0XE7){
		usart_puts("\nnrf post-listening check failed");
		usart_puts("\nRX_ADDR_P0:");
		usart_putbyte(get_reg(RX_ADDR_P0));
		usart_putch('\n');
		return 0;
	}
	else if(get_reg(RX_ADDR_P1)!=0XC2){
		usart_puts("\nnrf post-listening check failed");
		usart_puts("\nRX_ADDR_P1:");
		usart_putbyte(get_reg(RX_ADDR_P1));
		usart_putch('\n');
		return 0;
	}	
	else if(get_reg(TX_ADDR)!=0XE7){
		usart_puts("\nnrf post-listening check failed");
		usart_puts("\nTX_ADDR:");
		usart_putbyte(get_reg(TX_ADDR));
		usart_putch('\n');
		return 0;
	}
	else if(get_reg(DYNPD)!=0X03){
		usart_puts("\nnrf post-listening check failed");
		usart_puts("\nDYNPD:");
		usart_putbyte(get_reg(DYNPD));
		usart_putch('\n');
		return 0;
	}
	else if(get_reg(FEATURE)!=0X04){
		usart_puts("\nnrf post-listening check failed");
		usart_puts("\nFEATURE:");
		usart_putbyte(get_reg(FEATURE));
		usart_putch('\n');
		return 0;
	}
	else{
		usart_puts("\nnrf post-listening check completed successfully\n");
		return 1;
	}
}



