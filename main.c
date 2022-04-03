/*
 * Sprint10 - AFMN.c
 *
 * Created: 02/04/2022 22:37:29
 * Author : Felipe
 */ 

#define F_CPU 16000000UL
#define  BAUD 9600
#define MYUBRR F_CPU/16/BAUD-1
#include <avr/io.h>
#include <avr/interrupt.h>
#include <stdio.h>
#include <avr/delay.h>
#include <avr/eeprom.h>
#include "SSD1306.h"
#include "Font5x8.h"


// Novos tipos
typedef struct stc_veiculo //struct para armazenar parâmetros do veículo
{
	uint16_t velocidade_carro_kmH; // velocidade do veiculo em km/h
	uint16_t RPM_motor;            //RPM do motor (obs transimssão 1:1)
	uint16_t Diametro_pneu_cm;     //diametro do peneu em cm
	uint16_t Curso_pedal;          // posição do pedal de aceleração (0 - 1023)
	uint16_t distancia_veiculo;    // indica a distância percorrida pelo carro
	uint16_t carga_bateria;       // indica a porcentagem de bateria do carro
	uint16_t temperatura_bateria; // indica a temperatura da bateria
	float Distancia_hodometro_km;  // Distância total percorrida em km
	char Marcha;                   // Drive ("D"), reverse('R') e park('P')
	
}stc_veiculo; //define o nome do novo tipo criado


// variáveis globais
stc_veiculo Veiculo={.carga_bateria=0,.temperatura_bateria=0,.Curso_pedal=0,.velocidade_carro_kmH=123, .RPM_motor=0, .Diametro_pneu_cm=65, .Distancia_hodometro_km = 10, .Marcha='D', .distancia_veiculo=0};// incializa struct do veículo
uint8_t Flag_5ms=0, Flag_500ms=0, Flag_aux = 0;
uint32_t Tempo_ms=0;
int aux1 = 0, aux2 = 10, aux_bat = 0;
float valor_ant = 0;
uint32_t tempo_borda_subida = 0, temp_delta = 0;

//protótipos para o eeprom
void EEPROM_write(unsigned int uiAddress, unsigned char ucData);
unsigned char EEPROM_read(unsigned int uiAddress);
void potenciomentro(int i);
void USART_Transmit(unsigned char data);

// modo de captura TIMER1 (indicativo de distância entre o carro e o da frente)

ISR(USART_RX_vect) // INTERRUPÇÃO PARA USO DA USART
{
	char recebido;
	recebido = UDR0;
	if(recebido == 'l')
	{
		// sabendo que a entrada do serio foi l, limpamos o valor guardado.
		aux_bat = 0 ;
		eeprom_write_byte(0,aux_bat);
		valor_ant = aux_bat;
	}
	if(recebido == 'd')
	{
		USART_Transmit(aux_bat);
	}

}

// função para envio da frame de 5 a 8 bits

void USART_Transmit(unsigned char data)
{
	while(!(UCSR0A & (1<<UDRE0)));
	UDR0 = data;
}

ISR(TIMER1_CAPT_vect) // interrupção por captura do valor TC1
{
	
	if(TCCR1B & (1<<ICES1)) // lê o valor do TC1 na borda de subida do sinal
	{
		tempo_borda_subida=ICR1; // salva a primeira contagem para determinar a largura do pulso
	}
	else // lê o valor da contagem de TC1 na borda de descida do sinal
	{
		temp_delta = (ICR1 - tempo_borda_subida)*12.8; // cada incremento de TC1 corresponde a 1us
	}
	TCCR1B ^= (1<<ICES1); // inverte a borda de captura
	Veiculo.distancia_veiculo = temp_delta/58;
	

}

// tratamento da interrupções (utilização para contagem de tempo)
ISR(TIMER0_COMPA_vect) //interrupção do TC0 a cada 1ms = (64*(249+1))
{
	Tempo_ms++;
	if((Tempo_ms % 1)==0) // true a cada 5ms
	{
		Flag_aux = Flag_5ms = 1;
	}
	
	if((Tempo_ms % 50)==0) // true a cada 500ms
	{
		Flag_500ms=1;
	}
}

ISR(INT0_vect) // interrupção externa 0, PIN D2, tacômetro (distância percorrida pelo carro)
{
	static uint8_t cont_5voltas = 0;
	static uint32_t tempo_ms_anterior = 0;
	uint16_t delta_t_ms = 0;
	
	if(cont_5voltas == 5)
	{
		delta_t_ms = Tempo_ms - tempo_ms_anterior;
		Veiculo.RPM_motor = 300000/(delta_t_ms); //(5voltas*60min*100ms)/delta_t_ms
		Veiculo.velocidade_carro_kmH = ((uint32_t)Veiculo.Diametro_pneu_cm*565)/(delta_t_ms) + 60;
		tempo_ms_anterior = Tempo_ms;
		cont_5voltas = 0;
	}
	cont_5voltas++;
	Veiculo.Distancia_hodometro_km += ((float)Veiculo.Diametro_pneu_cm*3.1415)/100000; //distancia percorrida pelo veículo
	if(aux2 != Veiculo.Distancia_hodometro_km)
	{
		aux2 = Veiculo.Distancia_hodometro_km;
		eeprom_write_byte(0,aux2);
	}
	
}

ISR(PCINT2_vect) // interrupção por mudança de pino na porta D (PD4 e PD5) BOTÃO "+" E "-"
{
	if((PIND&0b00010000)==0) //botão + pressionado
	{
		if(Veiculo.Diametro_pneu_cm < 200)
		{
			Veiculo.Diametro_pneu_cm++;
			aux1 = Veiculo.Diametro_pneu_cm;
			eeprom_write_byte(1, aux1);
		}

	}
	
	if((PIND&0b00100000)==0) //botão - pressionado
	{
		if(Veiculo.Diametro_pneu_cm > 1)
		{
			Veiculo.Diametro_pneu_cm--;
			aux1 = Veiculo.Diametro_pneu_cm;
			eeprom_write_byte(1, aux1);
		}
	}
	
	
	if((PIND&0b01000000)==0) // marcha DRIVE 'D' (P06) (indicativo da marcha do carro)
	{
		Veiculo.Marcha = 'D';
	}
	
	else // marcha REVERSE - R(P06)
	{
		Veiculo.Marcha = 'R';
	}
	
	if((PIND&0b10000000)==0)
	{
		Veiculo.Marcha = 'P';
	}
}

ISR(ADC_vect)
{
	static uint8_t auxiliar = 0;
	switch(auxiliar)
	{
		_delay_ms(100);
		
		case 0: // para o pino do potenciometro da velocidade do motor
		ADMUX = 0b01000000;
		Veiculo.Curso_pedal = ADC;
		if(((Veiculo.distancia_veiculo<300)&(Veiculo.velocidade_carro_kmH>20))==1)
		{
			OCR2B = Veiculo.Curso_pedal/20;
		}
		else
		{
			OCR2B = Veiculo.Curso_pedal/2;
		}
		break;
		
		
		case 1:
		ADMUX = 0b01000001; // para medição de porcentagem da bateria
		Veiculo.carga_bateria = (ADC * 0.00489)*(100/5);
		break;
		
		
		case 2:
		ADMUX = 0b01000010; // para a medição da temperatura da bateria
		Veiculo.temperatura_bateria = (uint32_t)ADC * 2597/(1023-ADC) - 259; 
		if((Veiculo.temperatura_bateria) > valor_ant) // testagem do maior valor a ser guardado na eeprom
		{
			aux_bat = Veiculo.temperatura_bateria;
			eeprom_write_byte(2,aux_bat);
			valor_ant = aux_bat;
		}
		break;
		
		
		case 3: 
		ADMUX =  0b01000011; // para medição de lux dos raios luminozos
		if(ADC > 950)
		{
			PORTB = 0b00001010;
		}
		
		
		break;
		auxiliar = - 1;
	}
	auxiliar++;
	
	
}



int main(void)
{
	// configuração da interrupção para a temperatura da bateria
	//DDRC = 0b01110000;
	// Definição do GPIO
	//Direção dos pinos
	DDRC &= 0b11110000;
	DDRB |= 0b01111110; // PB0-PB6 como saídas para o display de velocidade
	PORTB |= 0b10000101;
	DDRD &= 0b00001011; // PD2(entrada tacômetro),PD3(saída PWM), PD4(entrada +), PD5(entrada -), PD6(entrada D/R), PD7(liga/desliga motor)
	//pull-ups de entrada
	PORTD = 0b00110000; // habilita os pull-ups de entrada
	// configurações das interrupções externas
	EICRA = 0b00000010;
	EIMSK = 0b00000001;
	PCICR = 0b00000100;
	PCMSK2 = 0b11110000;
	
	// CONFIGURAÇÃO DA USART //
	UBRR0H = (unsigned char)(MYUBRR>>8);
	UBRR0L = (unsigned char)MYUBRR;
	UCSR0B = (1<<RXCIE0)|(1<<RXEN0)|(1<<TXEN0);
	UCSR0C = (3<<UCSZ00);
	
	// configuração do timer 0
	
	TCCR0A = 0b00000010; // habilita modo CTC do TC0
	TCCR0B = 0b00000011; //liga o TC0 com prescaler = 64
	OCR0A = 249; // ajusta o comparardor para o TC0 contar até 249
	TIMSK0 = 0b00000010; // habilita a interrupção na igualdade de comparação com OCR0A
	
	// configuração do timer 2 no modo fast PWM
	TCCR2A = 0b00100011; // PWM não invertido no pino Oc0B - modo fast PWM
	TCCR2B = 0b00000100;  //liga TC2, prescaler = 64, fpwm =f0sc/(256*prescaler) = 16Hz/(256*64) = 976Hz
	OCR2B = 128;
	
	// configuração do ADC
	ADCSRA = 0b11101111; // habilita o AD, modo de conversão contínua, prescalar = 128, interrupção habilitada
	ADCSRB = 0b00000000; // modo de conversão contínua
	DIDR0 = 0b00100000; // desabilita o pino PC5 como entrada principal
	
	//configuração para a interrupção timer1
	TCCR1B = (1<<ICES1)|(1<<CS12); // captura na borda de subida, TC1 com prescale = 256
	TIMSK1 = 1<<ICIE1; // habilita a interrupção por captura
	
	
	// habilita o flag de interrupções globais
	sei();
	
	Veiculo.distancia_veiculo = temp_delta/58;
	Veiculo.Diametro_pneu_cm = eeprom_read_byte(1);
	Veiculo.Distancia_hodometro_km = eeprom_read_byte(0);
	
	while (1)
	{	
		PORTB = 0b00000100;
		
		if (Veiculo.Marcha == 'R')
		{
			PORTB = 0b00000010;
		}
		else
		{
			PORTB = 0b00000000;
		}
		
		anima_LCD(Veiculo, &Flag_500ms);
		
	}
	
	
}


void anima_LCD(stc_veiculo veiculo, uint8_t *flag_disparo_, int A)
{
	if(*flag_disparo_)
	{
		char diametro_pneu_cm_string[4];
		char rpm_motor_string[6];
		char distancia_hodometro_km_string[8];
		char distancia_sensor_string[5];
		char veiculo_marcha_string[1];
		char veiculo_carga_bateria_string[4];
		char veiculo_temperatura_bateria_string[4];
		char veiculo_velocidade_string[8];
		
		sprintf(diametro_pneu_cm_string, "%u", veiculo.Diametro_pneu_cm);
		sprintf(rpm_motor_string, "%u", veiculo.RPM_motor);
		sprintf(distancia_hodometro_km_string, "%u", (uint16_t)veiculo.Distancia_hodometro_km);
		sprintf(distancia_sensor_string ,"%u", Veiculo.distancia_veiculo);
		sprintf(veiculo_marcha_string ,"%u", veiculo.Marcha);
		sprintf(veiculo_carga_bateria_string, "%u",Veiculo.carga_bateria );
		sprintf(veiculo_temperatura_bateria_string, "%u", veiculo.temperatura_bateria);
		sprintf(veiculo_velocidade_string, "%u", veiculo.velocidade_carro_kmH);
		
		GLCD_Setup();
		GLCD_SetFont(Font5x8, 5, 8, GLCD_Overwrite);
		GLCD_InvertScreen();
		GLCD_Clear();
		
		GLCD_GotoXY(10, 10);
		GLCD_PrintString(veiculo_velocidade_string);
		GLCD_GotoXY(30, 10);
		GLCD_PrintString("Km/h");
		
		GLCD_GotoXY(95, 10);
		GLCD_PrintString(veiculo_carga_bateria_string);
		GLCD_GotoXY(110, 10);
		GLCD_PrintString("%");
		GLCD_GotoXY(95, 25);
		GLCD_PrintString(veiculo_temperatura_bateria_string);
		GLCD_GotoXY(115, 25);
		GLCD_PrintString("C");
		
		GLCD_GotoXY(10, 25);
		GLCD_PrintString("Sonar:");
		GLCD_GotoXY(50, 25);
		GLCD_PrintString(distancia_sensor_string);
		
		GLCD_GotoXY(10, 35);
		GLCD_PrintString("D.Pneu:");
		GLCD_GotoXY(55, 35);
		GLCD_PrintString(diametro_pneu_cm_string);
		
		GLCD_GotoXY(20, 50);
		GLCD_PrintString(distancia_hodometro_km_string);
		GLCD_GotoXY(40, 50);
		GLCD_PrintString("km");
		GLCD_GotoXY(95, 50);
		GLCD_PrintString(veiculo_marcha_string);
		GLCD_GotoXY(110, 50);
		GLCD_PrintString("ASC");
		
		GLCD_Render();
		
		*flag_disparo_ = 0;
	}
}




