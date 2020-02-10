/*
main.c
Program robota minisumo o nazwie RoDAP
Projekt p�ytki i schemat dost�pne na githube:
github.com/krol8lo
Mikrokontroler to ATmega328

Uwaga!
Projekt jest udost�pniany jako pomoc naukowa.
Ca�o�� powstawa�a dwa tygodnie, w zwi�zku z tym ani hardware ani software nie s� najpi�kniejsze.
Nie wszystkie rozwi�zania s� m�dre i warte powt�rzenia - nale�y czyta� komentarze.
KR�L 2019
*/

#include <avr/io.h>
#include <util/delay.h>
#include <avr/interrupt.h>

//Definicje rejestr�w timer�w - �atwiejsze w u�yciu
#define L_PWM OCR1AL
#define R_PWM OCR1BL

//Mo�na te� zastosowa� const
#define REVERSE_TIME 60
#define DEFAULT_PWM 200

//Struktura przechowuj�ca dane z czujnik�w przeciwnika - czytelno�� kodu
volatile struct sensors_data
{
	uint8_t front[2];
	uint8_t rear[2];
	uint8_t front_last[2];
	uint8_t enemy;
} enemy;							//Niezbyt szcz�liwie dobrana nazwa - enemy.enemy wygl�da kiepsko

/*
Bardzo brzydka obs�uga startera z u�yciem dw�ch zmiennych
Zosta�a zrobiona w ten spos�b ze wzgl�du na trudne do naprawy b��dy w poprzednich pr�bach implementacji startera
*/
volatile uint8_t starta, startb;

uint8_t line[2];					//Stan czujnik�w linii
uint8_t pit;						//Tryb serwisowy(pitstop)

//Zmenne steruj�ce skr�caniem i zawracaniem
volatile char turn;
volatile uint8_t turn_time;

void set_leds(uint8_t a, uint8_t b);
void enable_sensors(uint8_t a, uint8_t b, uint8_t c, uint8_t d);
void line_read();
void batt_read();
void go(int8_t dir_l, int8_t dir_r);	//Ustawia kierunek obrotu k�(1 - prz�d, -1 - ty�, 0 - stop) - czytelno�� kodu
void stop();							//Zatrzyanie

int main(void)
{
	DDRB = 0b00111111;
	DDRC = 0b00010010;
	DDRD = 0b10101010;
	
	EICRA = _BV(ISC00);
	EIMSK = _BV(INT0);
	
	TCCR1A = _BV(COM1A1) | _BV(COM1B1) |_BV(WGM10);
	TCCR1B = _BV(WGM12) | _BV(CS12);
	
	ADMUX = _BV(ADLAR) | _BV(REFS0) | _BV(MUX2) | _BV(MUX1);
	ADCSRA = _BV(ADEN);
	
	/*
	Nie by�o tego na zaj�ciach - pin change interrupt
	Pozwala na wyprowadzenie przerwania na prawie ka�dym pinie GPIO,
	ale wymaga sprawdzenia, kt�ry pin wygenerowa� przerwanie - patrz ni�ej
	*/
	PCICR = _BV(PCIE2) | _BV(PCIE1);
	PCMSK2 = _BV(PCINT22) | _BV(PCINT16);
	PCMSK1 = _BV(PCINT10) | _BV(PCINT11);
	
	TCCR0A = _BV(WGM01);
	TCCR0B = _BV(CS02) | _BV(CS00);
	OCR0A = 255;
	
	starta = startb = 0;
	turn_time = 0;
	turn = 0;
	line[0] = line[1] = 0;
	enemy.front[0] = enemy.front[1] = 0;
	enemy.rear[0] = enemy.rear[1] = 0;
	enemy.front_last[0] = enemy.front_last[1] = 0;
	enemy.enemy = 0;
	pit = 0;
	
	//Mruganie ledami na pocz�tku kodu pozwala znajdowa� przypadkowe resety
	set_leds(1,0);
	_delay_ms(200);
	set_leds(0,1);
	_delay_ms(200);
	set_leds(0,0);
	
	batt_read();
	
	sei();
	
	while (1)
	{
		if(!starta && !startb)
		{
			if(PORTD & _BV(PD4))
			{
				pit = ~pit;
				if(pit)						//Tryb pitstopu - kr�cenie ko�ami u�atwia czyszczenie k�
				{
					go(1,1);
					L_PWM = R_PWM = DEFAULT_PWM;
				}
				else
				{
					stop();
				}
				_delay_ms(500);
			}
		}
		
		if(starta && startb)				//Po sygnale startera
		{
			set_leds(0,0);
			/*
			Ma�e oszustwo - kierunek obrotu na pocz�tku wybierany jest prze��cznikiem.
			Pozwala to na szybsze znalenienie przeciwnika - kr�tszy obr�t.
			Technika cz�sto stosowana na zawodach czasem implementowana te� za pomoc� czujnik�w -
			przed startem pokazujesz robotowi kierunek zas�aniaj�c jeden z czujnik�w.
			*/
			if(PINC & _BV(PC5))
			{
				go(1,-1);
			}
			else
			{
				go(-1,1);
			}
			turn = 'r';
			turn_time = REVERSE_TIME;		//Cofanie przez 60 cykli timera
			L_PWM = R_PWM = DEFAULT_PWM;
			enable_sensors(1,1,1,1);
			TIMSK0 = _BV(OCIE0A);
			startb = 0;
		}
		
		/*
		G��wna logika robota
		Bardzo agresywne zatrzymywanie i zmiana kierunku obrotu k� ze wzgl�du na problemy z przyczepno�ci� i s�abe silniki
		Z punktu widzenia estetyki kodu troch� za du�o zagnie�d�onych if'�w
		*/
		if(starta && !startb)
		{
			if(enemy.enemy)
			{
				if(enemy.front[1] || enemy.front[0])
				{
					TIMSK0 = 0;
					turn = 0;
					turn_time = 0;
					if(enemy.front[1] && enemy.front[0])
					{
						go(1,1);
						L_PWM = R_PWM = 255;
						continue;
					}
					else if(enemy.front[1])
					{
						go(1,0);
						L_PWM = 255;
						turn = 'r';
						TIMSK0 = _BV(OCIE0A);
						turn_time = REVERSE_TIME*3;
					}
					else if(enemy.front[0])
					{
						go(0,1);
						R_PWM = 255;
						turn = 'r';
						TIMSK0 = _BV(OCIE0A);
						turn_time = REVERSE_TIME*3;
					}
					else
					{
						/*
						Taka sytuacja mo�e mie� miejsce po przerwaniu
						*/
						TIMSK0 = 0;
						turn = 0;
						turn_time = 0;
						go(1,1);
						L_PWM = R_PWM = DEFAULT_PWM;
					}
				}
				/*
				Tylko, gdy robot nie omija akurat kraw�dzi
				Brak tego if'a powodowa� b��dy
				*/
				else if(turn == 0 || turn == 'r')
				{
					if(enemy.rear[1])
					{
						go(1,0);
						turn = 'r';
						TIMSK0 = _BV(OCIE0A);
						turn_time = REVERSE_TIME;
						L_PWM = DEFAULT_PWM;
					}
					if(enemy.rear[0])
					{
						go(0,1);
						turn = 'r';
						TIMSK0 = _BV(OCIE0A);
						turn_time = REVERSE_TIME;
						R_PWM = DEFAULT_PWM;
					}
				}
			}
			/*
			Gdy robot widzi przeciwnika ignoruje lini�
			*/
			line_read();
			if(line[0])
			{
				turn = 'p';
				TIMSK0 = _BV(OCIE0A);
				turn_time = REVERSE_TIME;
				go(-1,0);
				L_PWM = DEFAULT_PWM;
			}
			if(line[1])
			{
				turn = 'l';
				TIMSK0 = _BV(OCIE0A);
				turn_time = REVERSE_TIME;
				go(0,-1);
				R_PWM = DEFAULT_PWM;
			}
		}
		else
		{
			stop();
		}
	}
}

void set_leds(uint8_t a, uint8_t b)
{
	if(a)
	{
		PORTD &= ~_BV(PD3);
	}
	else
	{
		PORTD |= _BV(PD3);
	}
	if(b)
	{
		PORTB &= ~_BV(PB5);
	}
	else
	{
		PORTB |= _BV(PB5);
	}
}


void enable_sensors(uint8_t a, uint8_t b, uint8_t c, uint8_t d)
{
	if(a)
	{
		PORTD |= _BV(PD5);
	}
	else
	{
		PORTD &= ~_BV(PD5);
	}
	if(b)
	{
		PORTD |= _BV(PD1);
	}
	else
	{
		PORTD &= ~_BV(PD1);
	}
	if(c)
	{
		PORTC |= _BV(PC1);
	}
	else
	{
		PORTC &= ~_BV(PC1);
	}
	if(d)
	{
		PORTC |= _BV(PC4);
	}
	else
	{
		PORTC &= ~_BV(PC4);
	}
}

void line_read()	//Odczyt czujnik�w linii
{
	//ADC6 - prawy
	//ADC7 - lewy
	ADMUX &= ~_BV(MUX0);	//ADC6
	ADCSRA |= _BV(ADSC);
	while(ADCSRA & _BV(ADSC));
	if(ADCH > 128)
	line[1] = 0;
	else
	line[1] = 1;
	
	ADMUX |= _BV(MUX0);	//ADC7
	ADCSRA |= _BV(ADSC);
	while(ADCSRA & _BV(ADSC));
	if(ADCH > 128)
	line[0] = 0;
	else
	line[0] = 1;
}

void go(int8_t dir_l, int8_t dir_r)		//Sterowanie mostkami
{
	if(dir_l == 1)
	{
		PORTD |= _BV(PD7);
		PORTB &= ~_BV(PB0);
	}
	else
	{
		if(dir_l == -1)
		{
			PORTB |= _BV(PB0);
			PORTD &= ~_BV(PD7);
		}
		else
		{
			if(dir_l == 0)
			{
				PORTB |= _BV(PB0);
				PORTD |= _BV(PD7);
			}
		}
	}
	if(dir_r == 1)
	{
		PORTB |= _BV(PB4);
		PORTB &= ~_BV(PB3);
	}
	else
	{
		if(dir_r == -1)
		{
			PORTB |= _BV(PB3);
			PORTB &= ~_BV(PB4);
		}
		else
		{
			if(dir_r == 0)
			{
				PORTB |= _BV(PB3);
				PORTB |= _BV(PB4);
			}
		}
	}
}

void stop()
{
	OCR1AL = 0;
	OCR1BL = 0;
	enable_sensors(0,0,0,0);
	go(0,0);
}

void batt_read()		//Pomiar stanu akumulatora
{
	ADMUX = _BV(ADLAR) | _BV(REFS0);
	ADCSRA |= _BV(ADSC);
	while(ADCSRA & _BV(ADSC));
	if(ADCH < 191)
	{
		for(uint8_t i = 0; i < 10; i++)		//Sygnalizacja niskiego napi�cia
		{
			set_leds(1,0);
			_delay_ms(50);
			set_leds(0,0);
			_delay_ms(50);
		}
	}
	/*
	Przywr�cenie rejestru do stanu pocz�tkowego dla funkcji line_read()
	Niezbyt eleganckie rozwi�zanie, ale za to szybsze - batt_read() wywo�ywane jest tylko po resecie
	*/
	ADMUX = _BV(ADLAR) | _BV(REFS0) | _BV(MUX2) | _BV(MUX1);
}

/*
Obs�uga przerwa� pin change
Wype�nianie struktury enemy danymi z czujnik�w w zale�no�ci od stan�w pin�w podczas przewania
*/
ISR(PCINT1_vect)
{
	enemy.front_last[1] = enemy.front[1];
	enemy.rear[1] = !((PINC & _BV(PC2)) >> PC2);
	enemy.front[1] = !((PINC & _BV(PC3)) >> PC3);
	enemy.enemy |= !(enemy.front[1] | enemy.rear[1] | enemy.front[0] | enemy.rear[0]);
}

ISR(PCINT2_vect)
{
	enemy.front_last[0] = enemy.front[0];
	enemy.rear[0] = !((PIND & _BV(PD0)) >> PD0);
	enemy.front[0] = !((PIND & _BV(PD6)) >> PD6);
	enemy.enemy |= !(enemy.front[0] | enemy.rear[0] | enemy.front[1] | enemy.rear[1]);
}

ISR(TIMER0_COMPA_vect)	//Przerwanie timera s�u��ce do skr�cania i zawracania przez okre�lony czas (zmienna turn_time)
{
	if(turn_time-- == 0)
	{
		TIMSK0 = 0;
		turn = 0;
		go(1,1);
		L_PWM = R_PWM = DEFAULT_PWM;
	}
}


ISR(INT0_vect)			//Starter
{
	if(PIND & _BV(PD2))
	{
		starta = startb = 1;
	}
	else
	{
		stop();
		starta = 0;
	}
}
