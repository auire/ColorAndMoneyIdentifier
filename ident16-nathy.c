#include <avr/io.h>
#include "lcd.h"
#include <avr/interrupt.h>
//#include <avr/signal.h>
#include <inttypes.h>
#include <avr/iom16.h>

#define F_OSC 4000000		           /* frequencia do oscilador em Hz */
#define UART_BAUD_RATE 9600
#define UART_BAUD_CALC(UART_BAUD_RATE,F_OSC) ((F_OSC)/((UART_BAUD_RATE)*16l)-1)

#define inp(port) (port)
#define outp(port, val) (port) = (val)
#define sbi(port, bit) (port) |= (1 << (bit))
#define cbi(port, bit) (port) &= ~(1 << (bit))
#define bit_is_clear(sfr, bit) (!(_SFR_BYTE(sfr) & _BV(bit)))
#define bit_is_set(sfr, bit) (_SFR_BYTE(sfr) & _BV(bit))

#define R 0
#define G 1
#define B 2

int fromSerial = 0;

/***************************************************
*  Funções usart
***************************************************/
void usart_putc(unsigned char c) {
   //espera até que o UDR esteja pronto
	while(!(UCSRA & (1 << UDRE)));
	UDR = c;    //manda um caracter
}

void usart_puts(char *s) {
	//  loop até  *s != NULL
	while (*s) {
		usart_putc(*s);
		s++;
	}
}

/****************************************************
*  Display integer atraves do USART
****************************************************/
void usart_printint(unsigned int x)
{
  	char interger[10];
	itoa(x,interger,10);
	usart_puts(interger);
}

void usart_putint(unsigned int x)
{
	if(x<100)
		usart_putc(0x30);
	if(x<10)
		usart_putc(0x30);
	usart_printint(x);
}

void init(void) {
	// set baud rate
	UBRRH = (uint8_t)(UART_BAUD_CALC(UART_BAUD_RATE,F_OSC)>>8);
	UBRRL = (uint8_t)UART_BAUD_CALC(UART_BAUD_RATE,F_OSC);

	// Enable receiver and transmitter; enable RX interrupt
	UCSRB = (1 << RXEN) | (1 << TXEN) | (1 << RXCIE);

	//asynchronous 8N1
	UCSRC = (1 << URSEL) | (3 << UCSZ0);
}


/***************************************************
*  Display initialization
***************************************************/
void lcd_init(void)
{
  	LCD_DDR = 0xff;	//Make the LCD port an output

	_delay_ms(20);	//Allow LCD to power up > 15mS

  	//LCD Starts up in 8-Bit mode, busy cannot be checked at moment
  	//we send these bytes according to the datasheet.
  	LCD_PORT = 0x30; lcd_enable();  _delay_ms(1);
  	LCD_PORT = 0x30; lcd_enable();  _delay_ms(1);
  	LCD_PORT = 0x30; lcd_enable();

  	//Now we can check the Busy
  	lcd_busy();

  	LCD_PORT = 0x20; lcd_enable();	//This places the LCD in 4-bit mode.

  	lcd_busy();
  	LCD_PORT = 0x20; lcd_enable();	//FUNCTION SET
  	LCD_PORT = 0x80; lcd_enable();
  	lcd_busy();
  	LCD_PORT = 0x00; lcd_enable();	//DISPLAY/CURSOR
  	LCD_PORT = 0xc0; lcd_enable();
	lcd_busy();
  	LCD_PORT = 0x00; lcd_enable();	//ENTRY SET - Increment, No Disp Shift
  	LCD_PORT = 0x60; lcd_enable();

  	lcd_cls();	//Clear Screen
	lcd_home();	//Home Cursor
}

/***************************************************
*  Check if the LCD is Busy
***************************************************/
void lcd_busy(void)
{
	//We should check the busy flag properly, but
	//were going to cheat and just use a delay to
	//allow the LCD to not be busy
	_delay_ms(8);
}

/***************************************************
*  Toggle the Enable Pin on the LCD
***************************************************/
void lcd_enable()
{
	SET_E;		//E=1
	CLEAR_E;	//E=0
}


/***************************************************
*  Clear Display
***************************************************/
void lcd_cls()
{
  	lcd_busy();
  	LCD_PORT = 0x00; lcd_enable();	//CLEAR SCREEN
  	LCD_PORT = 0x10; lcd_enable();
}

/***************************************************
*  Go to Home Position (Top Left)
***************************************************/
void lcd_home()
{
  	lcd_busy();
  	LCD_PORT = 0x00; lcd_enable();
  	LCD_PORT = 0x20; lcd_enable();
}

/****************************************************
*  Goto position   (0x00 - 1.row, 1.col)
*                  (0x40 - 1.row, 2.col)
****************************************************/
void lcd_goto(byte pos)
{
	lcd_busy();
	byte temp;

  	temp=0x80;
  	temp|=pos;
  	LCD_PORT = (temp & 0xf0);
  	lcd_enable();
  	LCD_PORT = (pos<<4 & 0xF0);
  	lcd_enable();
}

/****************************************************
*  Put character on LCD
****************************************************/
void lcd_putch(byte data)
{
	lcd_busy();

	//Send the UPPER nibble first
	LCD_PORT = (data & 0xf0);
	SET_RS;
	CLEAR_RW;
  	lcd_enable();

  	//Now send the LOWER nibble
  	LCD_PORT = ((data << 4)  & 0xf0);
  	SET_RS;
	CLEAR_RW;
  	lcd_enable();

  	CLEAR_RS;
}

/***************************************************
*  Display null terminated string
***************************************************/
void lcd_putstr(byte *data)
{
    while(*data)
    {
        lcd_putch(*data);
        data++;
    }
}

/****************************************************
*  Display integer
****************************************************/
void lcd_printint(unsigned int x)
{
  	char interger[10];
	itoa(x,interger,10);
	lcd_putstr(interger);
}

void lcd_putint(unsigned int x)
{
	if(x<100)
		lcd_putstr("0");
	if(x<10)
		lcd_putstr("0");
	lcd_printint(x);
}

double max(double i, double j)
{
	if (i >= j)
		return i;
	else
		return j;
}

double min(double i, double j)
{
	if (i <= j)
		return i;
	else
		return j;
}

double max3(double a, double b, double c)
{
	return max(a,max(b,c));
}

double min3(double a, double b, double c)
{
	return min(a,min(b,c));
}

//função de leitura dos sensores
int leitura(int sensor)
{
	int i;
	float j, Vmax = 0, Vmin = 0;
	
	switch(sensor)
	{
		case R:
			cbi(ADMUX, MUX0);
			cbi(ADMUX, MUX1);
			cbi(ADMUX, MUX2);
			cbi(ADMUX, MUX3);
			cbi(ADMUX, MUX4);
			Vmax=255.0;
			Vmin=66.0;
			break;
			
		case G:
			cbi(ADMUX, MUX0);
			sbi(ADMUX, MUX1);
			cbi(ADMUX, MUX2);
			cbi(ADMUX, MUX3);
			cbi(ADMUX, MUX4);
			Vmax=290.0;
			Vmin=41.0;
			break;
			
		case B:
			sbi(ADMUX, MUX0);
			cbi(ADMUX, MUX1);
			cbi(ADMUX, MUX2);
			cbi(ADMUX, MUX3);
			cbi(ADMUX, MUX4);
			Vmax=272.0;
			Vmin=63.0;
			break;
	}
	
	sbi(ADCSRA, ADSC);
	while (bit_is_set(ADCSRA, ADSC)){}
	i=ADCH;
	
	if (i<0)
		i=i+256; /* leitura completa de 0 a 255 da tensao */
		
	j=(i/255.0)*527.0;   /* converte a leitura para valor de centesimo de volts */
	j=((j-Vmin)/(Vmax-Vmin))*255.0; /* converte a tensão em RGB */
	i=j;
	
	if (i>255) 
		i=255;
		
	if (i<0) 
		i=0;
	
	return i;
}

// declara a matriz/tabela dos valores RGB das 16 cores básicas
void declara(int TAB[16][3], char *cores[16][20])
{

	/* tabela padrão */
	TAB[0][0]=0;    TAB[0][1]=0;    TAB[0][2]=0; 	*cores[0]  = "Preto";
	TAB[1][0]=0;    TAB[1][1]=0;    TAB[1][2]=168;  *cores[1]  = "Azul";
	TAB[2][0]=0;    TAB[2][1]=168;  TAB[2][2]=0;    *cores[2]  = "Verde";
	TAB[3][0]=0;    TAB[3][1]=168;  TAB[3][2]=168;  *cores[3]  = "Ciano";
	TAB[4][0]=168;  TAB[4][1]=0;    TAB[4][2]=0;    *cores[4]  = "Vermelho";
	TAB[5][0]=168;  TAB[5][1]=0;    TAB[5][2]=168;  *cores[5]  = "Magenta";
	TAB[6][0]=168;  TAB[6][1]=84;   TAB[6][2]=0;    *cores[6]  = "Marrom";
	TAB[7][0]=168;  TAB[7][1]=168;  TAB[7][2]=168;  *cores[7]  = "Cinza Claro";
	TAB[8][0]=84;   TAB[8][1]=84;   TAB[8][2]=84;   *cores[8]  = "Cinza Escuro";
	TAB[9][0]=84;   TAB[9][1]=84;   TAB[9][2]=255;  *cores[9]  = "Azul Claro";
	TAB[10][0]=84;  TAB[10][1]=255; TAB[10][2]=84;  *cores[10] = "Verde Claro";
	TAB[11][0]=84;  TAB[11][1]=255; TAB[11][2]=255; *cores[11] = "Ciano Claro";
	TAB[12][0]=255; TAB[12][1]=84;  TAB[12][2]=84;  *cores[12] = "Vermelho Claro";
	TAB[13][0]=255; TAB[13][1]=84;  TAB[13][2]=255; *cores[13] = "Magenta Claro";
	TAB[14][0]=255; TAB[14][1]=255; TAB[14][2]=84;  *cores[14] = "Amarelo";
	TAB[15][0]=255; TAB[15][1]=255; TAB[15][2]=255; *cores[15] = "Branco";

}

void calculateHSLfromRGB(int var_R, int var_G, int var_B, double *h, double *s, double *l)
{
	double delta;
	double r, g, b;

	//RGB values = 0 ÷ 255
	r = ((double)var_R / 255.0 ); 
	g = ((double)var_G / 255.0 );
	b = ((double)var_B / 255.0 );

	double cmax = max3(r,g,b);
	double cmin = min3(r,g,b);
	l=(cmax+cmin)/2.0;
	if(cmax==cmin) 
		*s = *h = 0; // it's really undefined
	else {
		if(*l < 0.5)	
			*s = (cmax-cmin)/(cmax+cmin);
		else			
			*s = (cmax-cmin)/(2.0-cmax-cmin);
		delta = cmax - cmin;
		if (r==cmax) 
			*h = (g-b)/delta;
		else 
			if(g==cmax) 
				*h = 2.0 +(b-r)/delta;
	  		else        
				*h = 4.0+(r-g)/delta;
	  	*h /= 6.0;
	  	if (*h < 0.0) 
			*h += 1;

		*h = *h * 360;
	}
}

// encontra na tabela a cor que mais se aproxima da cor detectada e devolve o número correspondente
int aproxima(int r, int g, int b, int TAB[16][3])
{
    int i, cor;
    double dista;
    double distb;
    double h, s, l, htab, stab, ltab;

    dista = 255.0*255.0;
    cor   = 0;
    for(i=0; i<=15; i++)
	{     

		calculateHSLfromRGB(r,g,b,&h,&s,&l);
		calculateHSLfromRGB(TAB[i][0],TAB[i][1],TAB[i][2],&htab,&stab,&ltab);
		dist = ( (h-htab*h-htab) + (s-stab*s-stab) + (l-ltab*l-ltab) );

		//distb = ( (r-TAB[i][0])*(r-TAB[i][0]) )+ 
		//		( (g-TAB[i][1])*(g-TAB[i][1]) )+
		//		( (b-TAB[i][2])*(b-TAB[i][2]) );
		
		if (distb<dista)
		{
			dista = distb;
			cor   = i;
		}
    }
    return cor;
}

/* le sensores */
void lerSensores(int btnCor, int TAB[16][3], char *cores[16][20])
{
	int r, g, b;
	int cor;
	
	r=leitura(R);
	_delay_ms(100);
	g=leitura(G);
	_delay_ms(100);
	b=leitura(B);
	_delay_ms(100);
	
	/* imprime valores */
	lcd_cls();
	lcd_home();
	
	lcd_putstr("R");
	lcd_putint(r);
	
	lcd_putstr(" G");
	lcd_putint(g);
	
	lcd_putstr(" B");
	lcd_putint(b);
	
	/* imprime o nome da cor - tabelado */
	lcd_goto(0x40); /* pula linha */
	
	if(btnCor == 1) // reconhece cor
	{
		cor = aproxima(r, g, b, TAB);
		lcd_putstr(*cores[cor]);
		usart_putc(0x43); //manda um "C" indicando que está no modo Cor
	}
	else       //reconhece dinheiro
	{
		lcd_putstr("    Dinheiro");
		usart_putc(0x44); //manda um "D" indicando que está no modo Dinheiro
	}
	
	//envia ao computador os dados
	usart_putint(r);
	usart_putint(g);
	usart_putint(b);
}

// INTERRUPT pode ser interrompido
// SIGNAL não pode ser interrompido
// Função que é disparada quando receber um caracter pela serial
SIGNAL (SIG_UART_RECV)
{
	unsigned char c;
	c = UDR;
	unsigned int i;
	
	switch(c)
	{
		case 0x43: //recebe um C do computador (modo Cor)
			fromSerial = 1; //cor
			break;
			
		case 0x44: //recebe um D do computador(modo Dinheiro)
			fromSerial = 2; //dinheiro
			break;
			
		case 0x50: //recebe um P (pc está pingando)
			usart_putc(0x4D);
			break;
		case 0x30:
			usart_putc(0x30);
			for(i=0;i<256;i++)
				usart_putint(i);
			usart_putc(0x30);
			break;
	}
}

int main ()
{
	int TAB[16][3];
	char *cores[16][20]; // 16 cores de no maximo 20 letras
	int btnCor, btnDin, btnCorOld, btnDinOld;
	
	btnCor    = 0;
	btnDin    = 0;
	btnCorOld = 0;
	btnDinOld = 0;
	
	declara(TAB, cores);
	_delay_ms(100);
	
	init(); //inicia USART
	sei();  // habilita a interrupção
	
	sbi(ADCSRA, ADEN); /* ativa conversor AD */
	sbi(ADMUX, ADLAR); /* ajuste a esquerda da leitura AD */
	
	lcd_init();         /* inicializa LCD */
	lcd_putstr(" Identificador  ");   /* apresentação */
	lcd_goto(0x40);
	lcd_putstr("    de Cores    ");

	// manda um caracter inicial para informar que está ligado ao PC
	while(!(UCSRA & (1 << UDRE)));
	UDR = 0x4D; // manda um "M" 
	
	while (1)
	{
		if( (btnCor == 1 && btnCorOld == 0) ||
			(btnDin == 1 && btnDinOld == 0) )
		{
			lerSensores(btnCor, TAB, cores);
			fromSerial = 0;
		}
		else if(fromSerial > 0) //leitura iniciada pelo PC
		{
			lerSensores(fromSerial, TAB, cores);
			fromSerial = 0;
		}
		
		/* leitura dos botoes */
		btnCorOld = btnCor;
		btnCor = bit_is_set(PINB, PINB0);
		
		btnDinOld = btnDin;
		btnDin = bit_is_set(PINB, PINB1);
	}
	
	return 0;
}
