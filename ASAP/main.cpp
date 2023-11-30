#define __DELAY_BACKWARD_COMPATIBLE__

#define F_CPU 16000000UL
#define FS_SEL 131

#include <avr/io.h>
#include <avr/interrupt.h>
#include <util/delay.h>
#include <math.h>
#include <stdlib.h>
#include <string.h>

#define UBRR 103

#define delta_t 0.05

 #define DO 261.63
 #define RE 293.67
 #define MI 329.63
 #define FA 349.23
 #define SOL 392.11
 #define RA 440.11
 #define SI 493.88
 #define HDO 523.25
 #define HRE 587.33
 #define HMI 659.26
 #define HFA 698.46
 #define HSOL 783.99
 #define HRA 880.11
 #define HSI 987.77
	
#define MAF_SIZE 5
double MAF_buff[MAF_SIZE] = {0};
int MAF_count = 0;

struct Mpu6050{
	volatile double dt;
	volatile int temp;
	volatile unsigned char a_x_l,a_x_h,a_y_l,a_y_h,a_z_l,a_z_h;
	volatile unsigned char g_x_l,g_x_h,g_y_l,g_y_h,g_z_l,g_z_h;
	volatile double bas_a_x,bas_a_y,bas_a_z;
	volatile double bas_g_x,bas_g_y,bas_g_z;
	volatile double a_x,a_y,a_z;
	volatile double g_x,g_y,g_z;
	volatile double las_angle_gx,las_angle_gy,las_angle_gz;
	volatile double angle_ax,angle_ay,angle_az;
	volatile double angle_gx,angle_gy,angle_gz;
	volatile double roll,pitch,yaw;
	volatile double alpha;
}IMU;

typedef struct FIR
{
	int n_fir;
	int Fc_fir;
	
	double *b_fir;
	double *FIR_ADC_buff;
}FIR;

typedef struct IIR
{
	int n_iir;
	int Fc_iir;
	
	double *a_iir;
	double *b_iir;
	double *IIR_ADC_buff;
	double *IIR_data;
}IIR;

FIR* Potentiometer = (FIR*)malloc(sizeof(FIR));
FIR* Cds = (FIR*)malloc(sizeof(FIR));
FIR* LM35 = (FIR*)malloc(sizeof(FIR));
IIR* Psd_L = (IIR*)malloc(sizeof(IIR));
IIR* Ir = (IIR*)malloc(sizeof(IIR));

double P[2][2] = {{0,0}, {0,0}};
double Q_angle = 0.01;
double Q_gyroBias = 0.01;
double R_measure = 0.001;

double KFbias = 0;
double KFangle = 0;
double KFangle_output = 0;

double K[2] = {0, 0};

double Potentiometer_b_fir[6] = {0.0000, 0.0952, 0.4048, 0.4048, 0.0952, 0.0000}; //차수 5 차단 3
double Cds_b_fir[11] = {-0.0052, -0.0080, 0.0134, 0.1057, 0.2405, 0.3072, 0.2405, 0.1057, 0.0134, -0.0080, -0.0052}; //차수 10 차단 3
double LM35_b_fir[6] = {0.0000, 0.0952, 0.4048, 0.4048, 0.0952, 0.0000}; //차수 5 차단 3
double Psd_L_a_iir[3] = {1, -0.7478, 0.2722}; //차수 2 차단 3
double Psd_L_b_iir[3] = {0.1311, 0.2622, 0.1311}; //차수 2 차단 3
double Ir_a_iir[4] = {1, -1.1619, 0.6959, -0.1378}; //차수 3 차단 3
double Ir_b_iir[4] = {0.0495, 0.1486, 0.1486, 0.0495}; //차수 3 차단 3

double adc_Potentiometer;
double adc_Cds;
double adc_LM35;
double adc_Psd_L;
double adc_Ir;

double filtered_Potentiometer;
double filtered_Cds;
double filtered_LM35;
double filtered_Psd_L;
double filtered_Ir;
		
double Conversion_Potentiometer = 0;
double Conversion_Cds = 0;
double Conversion_LM35 = 0;
double Conversion_Psd_L = 0;
double Conversion_Ir = 0;
		
int g_cnt = 0;
int initcnt = 0;
int inited = 0;
int startflag = 0;
int Ircnt = 0;
int buzzeron = 0;
double angle = 90.0;

unsigned char twi_read(char address)
{
	unsigned char data;

	TWCR = (1<<TWINT)|(1<<TWSTA)|(1<<TWEN);	//START

	while(!(TWCR & (1<<TWINT))); //TWINT flag 기다림
	while((TWSR&0xF8) != 0x08);  //START 상태(08) 기다림

	TWDR = 0b11010000;			 //AD(1101000)+W(0)
	TWCR = (1<<TWINT)|(1<<TWEN); //전송

	while(!(TWCR & (1<<TWINT)));
	while((TWSR&0xF8) != 0x18);  //SLA+W ACK 상태(18) 기다림

	TWDR = address; 			 //register address
	TWCR = (1<<TWINT)|(1<<TWEN); //전송

	while(!(TWCR & (1<<TWINT)));
	while((TWSR&0xF8) != 0x28);  //Data ACK 상태(28) 기다림

	TWCR = (1<<TWINT)|(1<<TWSTA)|(1<<TWEN);	//Repeat START

	while(!(TWCR & (1<<TWINT)));
	while((TWSR&0xF8) != 0x10);  //Repeat START 상태(08) 기다림

	TWDR = 0b11010001;			 //AD(1101000)+R(1)
	TWCR = (1<<TWINT)|(1<<TWEN); //전송

	while(!(TWCR & (1<<TWINT)));
	while((TWSR&0xF8) != 0x40);  //SLA+R ACK 상태(40) 기다림

	TWCR = (1<<TWINT)|(1<<TWEN); //전송

	while(!(TWCR & (1<<TWINT)));
	while((TWSR&0xF8) != 0x58);  //ACK 상태(58) 기다림

	data = TWDR;

	TWCR = (1<<TWINT)|(1<<TWSTO)|(1<<TWEN); //STOP

	return data;
}

void twi_write(unsigned char address,unsigned char data)
{
	TWCR = (1<<TWINT)|(1<<TWSTA)|(1<<TWEN);	//START

	while(!(TWCR & (1<<TWINT))); //TWINT flag 기다림
	while((TWSR&0xF8) != 0x08);  //START 상태(08) 기다림

	TWDR = 0b11010000;			 //AD(1101000)+W(0)
	TWCR = (1<<TWINT)|(1<<TWEN); //전송

	while(!(TWCR & (1<<TWINT)));
	while((TWSR&0xF8) != 0x18);  //SLA+W ACK 상태(18) 기다림

	TWDR = address; 			 //register address
	TWCR = (1<<TWINT)|(1<<TWEN); //전송

	while(!(TWCR & (1<<TWINT)));
	while((TWSR&0xF8) != 0x28);  //Data ACK 상태(28) 기다림

	TWDR = data; 				 //data
	TWCR = (1<<TWINT)|(1<<TWEN); //전송

	while(!(TWCR & (1<<TWINT)));
	while((TWSR&0xF8) != 0x28);

	TWCR = (1<<TWINT)|(1<<TWSTO)|(1<<TWEN); //STOP
}

void get_raw_data(struct Mpu6050* xMpu6050)
{
	xMpu6050->a_x_h = twi_read(0x3B);		//x축 가속도
	_delay_us(10);
	xMpu6050->a_x_l = twi_read(0x3C);
	_delay_us(10);
	xMpu6050->a_y_h = twi_read(0x3D);		//y축 가속도
	_delay_us(10);
	xMpu6050->a_y_l = twi_read(0x3E);
	_delay_us(10);
	xMpu6050->a_z_h = twi_read(0x3F);		//z축 가속도
	_delay_us(10);
	xMpu6050->a_z_l = twi_read(0x40);
	_delay_us(10);
	xMpu6050->g_x_h = twi_read(0x43);		//x축 각속도
	_delay_us(10);
	xMpu6050->g_x_l = twi_read(0x44);
	_delay_us(10);
	xMpu6050->g_y_h = twi_read(0x45);		//y축 각속도
	_delay_us(10);
	xMpu6050->g_y_l = twi_read(0x46);
	_delay_us(10);
	xMpu6050->g_z_h = twi_read(0x47);		//z축 각속도
	_delay_us(10);
	xMpu6050->g_z_l = twi_read(0x48);
	_delay_us(10);
}

void calibrate(struct Mpu6050* xMpu6050)
{
	int cal = 10;

	for(int i=0; i<cal; i++)
	{
		get_raw_data(xMpu6050);
		
		xMpu6050->temp = (xMpu6050->a_x_h<<8) | xMpu6050->a_x_l;
		xMpu6050->a_x += - xMpu6050->temp - 16383;
		xMpu6050->temp = (xMpu6050->a_y_h<<8) | xMpu6050->a_y_l;
		xMpu6050->a_y += - xMpu6050->temp;
		xMpu6050->temp = (xMpu6050->a_z_h<<8) | xMpu6050->a_z_l;
		xMpu6050->a_z += xMpu6050->temp;
		xMpu6050->temp = (xMpu6050->g_x_h<<8) | xMpu6050->g_x_l;
		xMpu6050->g_x += xMpu6050->temp;
		xMpu6050->temp = (xMpu6050->g_y_h<<8) | xMpu6050->g_y_l;
		xMpu6050->g_y += xMpu6050->temp;
		xMpu6050->temp = (xMpu6050->g_z_h<<8) | xMpu6050->g_z_l;
		xMpu6050->g_z += xMpu6050->temp;

		_delay_ms(100);
	}
	
	xMpu6050->a_x /= cal;
	xMpu6050->a_y /= cal;
	xMpu6050->a_z /= cal;
	xMpu6050->g_x /= cal;
	xMpu6050->g_y /= cal;
	xMpu6050->g_z /= cal;

	xMpu6050->bas_a_x = xMpu6050->a_x;
	xMpu6050->bas_a_y = xMpu6050->a_y;
	xMpu6050->bas_a_z = xMpu6050->a_z;
	xMpu6050->bas_g_x = xMpu6050->g_x;
	xMpu6050->bas_g_y = xMpu6050->g_y;
	xMpu6050->bas_g_z = xMpu6050->g_z;
}

void calc_mpu6050_data(struct Mpu6050* xMpu6050)
{
	xMpu6050->las_angle_gx = xMpu6050->roll;
	xMpu6050->las_angle_gy = xMpu6050->pitch;
	xMpu6050->las_angle_gz = xMpu6050->yaw;

	xMpu6050->temp = (xMpu6050->a_x_h<<8) | xMpu6050->a_x_l;
	xMpu6050->a_x = - xMpu6050->temp - 16383;
	xMpu6050->temp = (xMpu6050->a_y_h<<8) | xMpu6050->a_y_l;
	xMpu6050->a_y = - xMpu6050->temp;
	xMpu6050->temp = (xMpu6050->a_z_h<<8) | xMpu6050->a_z_l;
	xMpu6050->a_z = xMpu6050->temp;
	xMpu6050->temp = (xMpu6050->g_x_h<<8) | xMpu6050->g_x_l;
	xMpu6050->g_x = xMpu6050->temp;
	xMpu6050->temp = (xMpu6050->g_y_h<<8) | xMpu6050->g_y_l;
	xMpu6050->g_y = xMpu6050->temp;
	xMpu6050->temp = (xMpu6050->g_z_h<<8) | xMpu6050->g_z_l;
	xMpu6050->g_z = xMpu6050->temp;

	xMpu6050->g_x = (xMpu6050->g_x - xMpu6050->bas_g_x)/FS_SEL;
	xMpu6050->g_y = (xMpu6050->g_y - xMpu6050->bas_g_y)/FS_SEL;
	xMpu6050->g_z = (xMpu6050->g_z - xMpu6050->bas_g_z)/FS_SEL;
	
	xMpu6050->angle_ax = atan(-1.000*xMpu6050->a_y/sqrt(pow(xMpu6050->a_x,2) + pow(xMpu6050->a_z,2)))*180/3.141592;
	xMpu6050->angle_ay = atan(xMpu6050->a_x/sqrt(pow(xMpu6050->a_y,2) + pow(xMpu6050->a_z,2)))*180/3.141592;

	xMpu6050->angle_gx = xMpu6050->g_x*xMpu6050->dt + xMpu6050->las_angle_gx;
	xMpu6050->angle_gy = xMpu6050->g_y*xMpu6050->dt + xMpu6050->las_angle_gy;
	xMpu6050->angle_gz = xMpu6050->g_z*xMpu6050->dt + xMpu6050->las_angle_gz;

	xMpu6050->dt = 0.000;

	xMpu6050->alpha = 0.96;
	xMpu6050->roll = xMpu6050->alpha*xMpu6050->angle_gx + (1.000 - xMpu6050->alpha)*xMpu6050->angle_ax;
	xMpu6050->pitch = xMpu6050->alpha*xMpu6050->angle_gy + (1.000 - xMpu6050->alpha)*xMpu6050->angle_ay;
	xMpu6050->yaw = xMpu6050->angle_gz;
}

void USART0_TX(unsigned char data)
{
	while(!(UCSR0A & (1<<UDRE0)));
	
	UDR0 = data;
}

void USART0_TX_int4(int data)
{
	if(data < 0)
	{
		data = -data;
		USART0_TX('-');
	}
	else
	USART0_TX(' ');

	int temp = 0;
	temp = data/10000;
	USART0_TX(temp+48);
	temp = (data%10000)/1000;
	USART0_TX(temp+48);
	temp = (data%1000)/100;
	USART0_TX(temp+48);
	temp = (data%100)/10;
	USART0_TX(temp+48);
	temp = data%10;
	USART0_TX(temp+48);
}

unsigned char USART0_RX(void)
{
	while(!(UCSR0A & (1<<RXC0)));
	
	return UDR0;
}

void USART0_NUM(int nNum)
{
	USART0_TX((nNum % 10000)/1000 + 48);
	USART0_TX((nNum % 1000)/100 + 48);
	USART0_TX((nNum % 100)/10 + 48);
	USART0_TX((nNum % 10) + 48);
}

void buzzer(double hz, int count)
{
	for(int i = 0; i < count; i++)
	{
		PORTC = 0b00000011;
		_delay_ms(((double)1000 / hz) / 2);
		
		PORTC = 0b00000001;
		_delay_ms(((double)1000 / hz) / 2);
	}
}

void FIR_init(FIR* fir, int n_fir, int Fc_fir, double* b_fir)
{
	fir->n_fir = n_fir;
	
	fir->b_fir = (double*)malloc(sizeof(double) * (n_fir+1));
	fir->FIR_ADC_buff = (double*)malloc(sizeof(double) * (n_fir+1));
	
	fir->b_fir = b_fir;
}

double FIR_filter(FIR* fir, double rawdata)
{
	double FIR_data = 0;
	
	for(int i = 0; i < fir->n_fir; i++)
	{
		fir->FIR_ADC_buff[i] = fir->FIR_ADC_buff[i+1];
	}
	fir->FIR_ADC_buff[fir->n_fir] = rawdata;
	
	for(int k = 0; k < fir->n_fir + 1; k++)
	{
		FIR_data += fir->b_fir[k] * fir->FIR_ADC_buff[fir->n_fir - k];
	}
	
	return FIR_data;
}

void IIR_init(IIR* iir, int n_iir, int Fc_iir, double* a_iir, double* b_iir)
{
	iir->n_iir = n_iir;
	
	iir->a_iir = (double*)malloc(sizeof(double) * (n_iir+1));
	iir->b_iir = (double*)malloc(sizeof(double) * (n_iir+1));
	iir->IIR_ADC_buff = (double*)malloc(sizeof(double) * (n_iir+1));
	iir->IIR_data = (double*)malloc(sizeof(double) * (n_iir+1));
	
	iir->a_iir = a_iir;
	iir->b_iir = b_iir;
}

double IIR_filter(IIR* iir, double rawdata)
{
	for(int i = 0; i < iir->n_iir; i++)
	{
		iir->IIR_ADC_buff[i] = iir->IIR_ADC_buff[i+1];
		iir->IIR_data[i] = iir->IIR_data[i+1];
	}
	iir->IIR_ADC_buff[iir->n_iir] = rawdata;
	
	for (int k = 0; k < iir->n_iir + 1; k++)
	{
		iir->IIR_data[iir->n_iir] += iir->b_iir[k] * iir->IIR_ADC_buff[iir->n_iir - k] - iir->a_iir[k] * iir->IIR_data[iir->n_iir - k];
	}
	
	return iir->IIR_data[iir->n_iir];
}

double get_Potentiometer()
{
	ADMUX = 0x00;
	
	ADCSRA |= (1<<ADSC); //USART start
	while(!(ADCSRA & (1 << ADIF))); //flag check
	
	int adc = ADC;
	
	return adc;
}

double get_cds()
{
	ADMUX = 0x01; //cds활성화
	
	ADCSRA |= (1<<ADSC); //USART start
	while(!(ADCSRA & (1 << ADIF))); //flag check
	
	int adc = ADC;
	
	return adc;
}

double get_LM35()
{
	ADMUX = 0x05; //LM35활성화
	
	ADCSRA |= (1<<ADSC); //USART start
	while(!(ADCSRA & (1 << ADIF))); //flag check
	
	int adc = ADC;
	
	return adc;
}

double get_Psd_L()
{
	ADMUX = 0x04;
	
	ADCSRA |= (1<<ADSC); //USART start
	while(!(ADCSRA & (1 << ADIF))); //flag check
	
	int adc = ADC;
	
	return adc;
}

double get_Psd_R()
{
	ADMUX = 0x06;
	
	ADCSRA |= (1<<ADSC); //USART start
	while(!(ADCSRA & (1 << ADIF))); //flag check
	
	int adc = ADC;
	
	return adc;
}

double get_Ir()
{
	ADMUX = 0x07;
	
	ADCSRA |= (1<<ADSC); //USART start
	while(!(ADCSRA & (1 << ADIF))); //flag check
	
	int adc = ADC;
	
	return adc;
}

double convert_Potentiometer(double filtereddata)
{
	double Vadc = filtereddata * 5.0 / 1023.0; //ADC값(0~1023)을 전압값(0~5V)로 수치 변환
	
	double Angle = Vadc * 360.0 / 5.0; //각도값
	
	return Angle;
}

double convert_Cds(double filtereddata)
{
	double Vadc = filtereddata * 5.0 / 1023.0;
	
	double R9 = 4.7 * 1000;
	double Rcds = R9 * 5 / Vadc - R9;
	double r = 0.7;
	
	double lux = pow(10, 1-((log(Rcds)-log(40 * 100)/r))); //LUX변환식
	
	return lux;
}

double convert_LM35(double filtereddata)
{
	double Vadc = filtereddata * 5.0 / 1023.0;
	 
	Vadc /= 4.0;
	
	double T = 100 * Vadc; //섭씨온도
	
	return T;
}

double convert_Psd(double filtereddata)
{
	double Vadc = filtereddata * 5.0 / 1023.0;
	
	double Distance = 29.988 * pow(Vadc, -1.173); //섭씨온도
	
	return Distance;
}

double convert_Ir(double filtereddata)
{	
	double NomalizeData = (filtereddata - 0.0) / (1023.0 - 0.0); //정규화
	
	if(NomalizeData < 0.17)
	{
		return 0;
	}
	else
	{
		return 1;
	}
}

void Algorithm_Potentiometer(double Conversion)
{
	if(Conversion > 0 && Conversion <= 72)
	{
		OCR1B = 0.00 * ICR1;
	}
	else if(Conversion > 72 && Conversion <= 72 * 2)
	{
		OCR1B = 0.02 * ICR1;
	}
	else if(Conversion > 72 * 2 && Conversion <= 72 * 3)
	{
		OCR1B = 0.04 * ICR1;
	}
	else if(Conversion > 72 * 3 && Conversion <= 72 * 4)
	{
		OCR1B = 0.08 * ICR1;
	}
	else
	{
		OCR1B = 0.10 * ICR1;
	}
}

void Algorithm_Cds(double Conversion)
{
	if(Conversion > 0 && Conversion <= 5000)
	{
		OCR3A = 0.8 * ICR3;
	}
	else if(Conversion > 5000 * 1 && Conversion <= 5000 * 2)
	{
		OCR3A = 0.6 * ICR3;
	}
	else if(Conversion > 5000 * 2 && Conversion <= 5000 * 3)
	{
		OCR3A = 0.4 * ICR3;
	}
	else if(Conversion > 5000 * 3 && Conversion <= 5000 * 4)
	{
		OCR3A = 0.2 * ICR3;
	}
	else
	{
		OCR3A = 0.0 * ICR3;
	}	
}

void Algorithm_LM35(double Conversion)
{
	if(Conversion > 0 && Conversion <= 14)
	{
		OCR3C = 0.0 * ICR3;
	}
	else if(Conversion > 14 && Conversion <= 14 + 1)
	{
		OCR3C = 0.3 * ICR3;
	}
	else if(Conversion > 14 + 1 && Conversion <= 14 + 2)
	{
		OCR3C = 0.6 * ICR3;
	}
	else if(Conversion > 14 + 2 && Conversion <= 14 + 3)
	{
		OCR3C = 0.9 * ICR3;
	}
	else
	{
		OCR3C = 1.0 * ICR3;
	}
}

void Algorithm_Psd(double Conversion_L)
{
	int MIN = 20;
	int MAX = 30;
	int test_angle;
	
	if(Conversion_L > MIN && Conversion_L < MAX)
	{
		angle = 90.0;
	}
	else if(Conversion_L < MIN)
	{
		test_angle = 90 - (MIN - Conversion_L) * 8;
		if(test_angle < 50) angle = 50;
		else angle = test_angle;
	}
	else if(Conversion_L > MAX)
	{
		test_angle = 90 + (Conversion_L - MAX) * 8;
		if(test_angle > 130) angle = 130;
		else angle = test_angle;
	}
	/*
	else if(Conversion_L < MIN)
	{
		angle = 50.0;
	}
	else if(Conversion_L > MAX)
	{
		angle = 120.0;
	}
	*/
}

void Algorithm_Ir(double Conversion)
{
	if(Conversion == 0)
	{		
		Ircnt++;
		
		if(Ircnt > 20)
		{
			OCR1A = 0;
			OCR1B = 0;
			OCR1C = 0;
			OCR3A = 0;
			
			buzzer(HMI, 100);
			_delay_ms(50);
			buzzer(HMI, 100);
			_delay_ms(200);
			buzzer(HMI, 100);
			_delay_ms(300);
			buzzer(HDO, 100);
			_delay_ms(100);
			buzzer(HMI, 100);
			_delay_ms(150);
			buzzer(HSOL, 100);
			_delay_ms(400);
			buzzer(SOL, 100);
			_delay_ms(50);
			
			startflag = 0;
		}
	}
}

unsigned int set_servo(double angle)
{
	double width;
	double duty;
	
	width = (angle / 90) + 0.5;
	
	duty = (width / 20.0) * 100;
	
	OCR1A = int(duty/100 * ICR1);
	return OCR1A;
}  

ISR(TIMER2_OVF_vect)
{	
	set_servo(angle);
	
	if(inited == 0)
	{	
		FIR_init(Potentiometer, 5, 3, Potentiometer_b_fir);
		FIR_init(Cds, 10, 3, Cds_b_fir);
		FIR_init(LM35, 5, 3, LM35_b_fir);
		IIR_init(Psd_L, 2, 3, Psd_L_a_iir, Psd_L_b_iir);
		IIR_init(Ir, 3, 2, Ir_a_iir, Ir_b_iir);
		
		inited = 1;
	}
	else
	{
		initcnt++;
		
		if(initcnt == 100)
		{
			startflag = 1;
		}
		
		//IMU.dt += 0.05;
	    //get_raw_data(&IMU);
		//calc_mpu6050_data(&IMU);
				
		adc_Potentiometer = get_Potentiometer();
		adc_Cds = get_cds();
		adc_LM35 = get_LM35();
		adc_Psd_L = get_Psd_L();
		adc_Ir = get_Ir();
		
		filtered_Potentiometer = FIR_filter(Potentiometer, adc_Potentiometer);
		filtered_Cds = FIR_filter(Cds, adc_Cds);
		filtered_LM35 = FIR_filter(LM35, adc_LM35);
		filtered_Psd_L = IIR_filter(Psd_L, adc_Psd_L);
		filtered_Ir = IIR_filter(Ir, adc_Ir);
		
		Conversion_Potentiometer = convert_Potentiometer(filtered_Potentiometer);
		Conversion_Cds = convert_Cds(filtered_Cds);
		Conversion_LM35 = convert_LM35(filtered_LM35);
		Conversion_Psd_L = convert_Psd(filtered_Psd_L);
		Conversion_Ir = convert_Ir(filtered_Ir);
		
		
		//START
		
		/*if(startflag == 1)
		{
			Algorithm_Potentiometer(Conversion_Potentiometer);
			Algorithm_Cds(Conversion_Cds);
			
			Algorithm_LM35(Conversion_LM35);
			Algorithm_Psd(Conversion_Psd_L);
			Algorithm_Ir(Conversion_Ir);
		}*/
	}
	
	//TEST
	
	g_cnt++;		
	if(g_cnt == delta_t*100) //0.05초
	{
		g_cnt = 0;
		
		USART0_NUM(Conversion_Potentiometer);
		USART0_TX(32);
		USART0_NUM(Conversion_Cds);
		USART0_TX(32);
		USART0_NUM(Conversion_LM35);
		USART0_TX(32);
		USART0_NUM(Conversion_Psd_L);
		USART0_TX(32);
		USART0_NUM(Conversion_Ir);
		
		USART0_TX(13);
		
		
		/*USART0_TX_int4(IMU.angle_gx);
		USART0_TX(',');
		USART0_TX_int4(IMU.angle_gy);
		USART0_TX(',');
		USART0_TX_int4(IMU.angle_gz);
		USART0_TX('\n');
		USART0_TX('\r');*/
		
	}
	
	TCNT2 = 255 - 156;
}

int main(void)
{
	DDRB = 0b11100000; //PWM 출력
	DDRC = 0b00000011; //DIR 출력
	DDRD = 0b00000000; //스위치 입력
	DDRE = 0b00111000; //PWM 출력
	DDRF = 0b00000000; //ADC 입력
	
	PORTC = 0b00000001; //앞으로
	
   	UCSR0A = 0x00;
   	UCSR0B = (1 << TXEN0);
   	UCSR0C = (3 << UCSZ00);
   	UBRR0H = 0;
   	UBRR0L = 103;
	   
    TWCR = (1 << TWEN);
    TWBR = 12;
	
	twi_write(0x6B, 0x00); 	//sleep 끔
	_delay_ms(1);
	twi_write(0x1A, 0x05); 	//DLPF 10Hz
	calibrate(&IMU);
		
	TCCR2 = (1 << FOC2) | (1 << WGM20) | (1 << WGM21) | (1 << COM21) | (1 << CS22) | (1 << CS20); //WGM조합 : 11 : Fast PWM mode, COM조합 : 10 : non-inverting mode, CS조합 : 101 : 분주비 1024
	TCNT2 = 255 - 156; //10ms 주기로 설정
	TIMSK = (1 << TOIE2);
	
	TCCR1A = (1 << COM1A1) | (0 << COM1A0) | (1 << COM1B1) | (0 << COM1B0) | (1 << COM1C1) | (0 << COM1C0) |(1 << WGM11) | (0 << WGM10); //WGM조합 : 1110 : Fast PWM mode, COM조합 : 10 : non-inverting mode, CS조합 : 011 : 분주비 64
	TCCR1B = (1 << WGM13) | (1 << WGM12) | (0 << CS12) | (1 << CS11) | (1 << CS10);
	TCCR1C = 0x00;
	ICR1 = 4999; //20ms 주기로 설정
	TCNT1 = 0;
	
	TCCR3A = (1 << COM3A1) | (0 << COM3A0) | (1 << COM3B1) | (0 << COM3B0) | (1 << COM3C1) | (0 << COM3C0)| (1 << WGM31) | (0 << WGM30); //WGM조합 : 1110 : Fast PWM mode, COM조합 : 10 : non-inverting mode, CS조합 : 011 : 분주비 64
	TCCR3B = (1 << WGM33) | (1 << WGM32) | (0 << CS32) | (1 << CS31) | (1 << CS30);
	TCCR3C = 0x00;
	ICR3 = 4999; //20ms 주기로 설정
	TCNT3 = 0;	

	ADMUX = 0x00;
	ADCSRA = 0x87;
		
	sei(); //모든 인터럽트 활성화
	
	//OCR1A = 0;
	//OCR1B = 0;
	//OCR1C = 0;
	//OCR3A = 0;
		
	//angle = 90.0;
	
    while (1)
	{

	}
}

