#include <avr\io.h>
#include <avr\interrupt.h>
#include <util\delay.h>
#include <stdbool.h>

// makro na funcje ustawiajace/zerujace okreslony bit y w slowie bitowym x
#define set_bit(x,y) x |= _BV(y) 			//set bit
#define clear_bit(x,y) x &= ~(_BV(y)) 

// deklaracje zmiennych 
unsigned int c0, c1, c2, c3, c4;
float previous_error=0;
float error = 0;
char last_known_line_side;
float P, I, D, correction;
unsigned short int basespeed = 50;
unsigned short Kp = 25;
unsigned short Ki = 1;
unsigned short Kd = 2;
unsigned int leftpulse;
unsigned int rightpulse;
unsigned int max_speed = 250;
uint16_t max_value = 1024;
uint16_t curr_value;
bool allow = 0;

void init_ref_voltage()
{
	// Ustawienie napiecia referencyjnego wewnetrznego
	set_bit(ADMUX, REFS0);
	clear_bit(ADMUX, REFS1);
}

void init_pins()
{	
	// Ustawienie pinów C0-C4 jako wejście sygnału z czujników
	clear_bit(DDRC, PC0);
	clear_bit(DDRC, PC1);
	clear_bit(DDRC, PC2);
	clear_bit(DDRC, PC3);
	clear_bit(DDRC, PC4);
	
	// Zapewnienie 'czystego', tzn. cyfrowego sygnału z czujników
	set_bit(PORTC, PC0);
	set_bit(PORTC, PC1);
	set_bit(PORTC, PC2);
	set_bit(PORTC, PC3);
	set_bit(PORTC, PC4);

	// Ustawienie pinów PD jako wyjścia do sterowania kierunkiem obracania się koła
	set_bit(DDRD, PD6);
	set_bit(DDRD, PD7);	
	set_bit(DDRD, PD0);
	set_bit(DDRD, PD1);

	
	// Ustawienie wartości początkowych podawanych na wejścia sterujących kierunkiem obracania się koła na 0
	clear_bit(PORTD, PD6);
	clear_bit(PORTD, PD7);	
	clear_bit(PORTD, PD0);
	clear_bit(PORTD, PD1);

	// Ustawienie portów B1 i B2 jako wyjścia dla PWM
	set_bit(DDRB, PB1); // Wyjście PWM lewego silnika - OC1A
	set_bit(DDRB, PB2);  // Wyjście PWM prawego silnka - OC1B
	
	// Ustawienie wartości początkowych dla PWM na 0
	clear_bit(PORTB, PB1);  
	clear_bit(PORTB, PB2);	
}



void init_pwm()
{
	// Ustawienie trybu: 8 bitowy szybki PWM 
	set_bit(TCCR1A, WGM10);
	set_bit(TCCR1B, WGM12);

	// Czyszczenie OC1A/OC1B gdy osiągnie wartość porównywaną
	// Ustawiam OC1A/OC1B gdy osiągnie wartość dolną ( czyli 0 ) 
	set_bit(TCCR1A, COM1A1);
	set_bit(TCCR1A, COM1B1);

	// Preskaler = 1,  fpwm (częstotliwość) = 31250Hz 
	set_bit(TCCR1B, CS10);

	// Ustawienie prędkości początkowej na maksimum 
	OCR1A = 250;         
	OCR1B = 250; 
}


void init_timer()
{
	// Tryb CTC - Timer zlicza w górę, a po przekroczeniu ustalonej wartości wywołuje przerwanie i się zeruje
	setbit(TCCR0A,WGM01);
	
	// Ustawienie taktowania na 30,5Hz
	setbit(TCCR0B,CS02);
	setbit(TCCR0B,CS00); 

	// Wartość po której licznik będzie się przepełniał 
	OCR0A = 78;

	// Włączenie przerwania po przepełnieniu licznika
	setbit(TIMSK0,OCIE0A);

	// Globalne włączenie przerwań
	sei();

}

ISR(TIMER0_COMPA_vect) 
{ 
    allow = 1; // Zezwolenie na wejście do algorytmu liczącego błąd
}

void initialize()
{
	init_ref_voltage();
	init_pins();
	init_pwm();
	init_timer();
}

void lm_move_forward(void)
{
	set_bit(PORTB, PB1);
	set_bit(PORTD, PD0); // Włącz jazde do przodu
	clear_bit(PORTD, PD1); // Wyłącz jazde do tylu
}

void lm_move_backward(void)
{
	set_bit(PORTB, PB1);
	clear_bit(PORTD, PD0); // Wyłącz jazde do przodu
	set_bit(PORTD, PD1); // Włącz jazde do tylu
}

void lm_stop(void)
{
	set_bit(PORTB, PB1);
	set_bit(PORTD, PD0); // Włącz jazde do przodu
	set_bit(PORTD, PD1); // Włącz jazde do tylu 
}

void rm_move_forward(void)
{
	set_bit(PORTB, PB2);
	set_bit(PORTD, PD6); // Włącz jazde do przodu
	clear_bit(PORTD, PD7); // Wyłącz jazde do tylu
}

void rm_move_backward(void)
{
	set_bit(PORTB, PB2);
	clear_bit(PORTD, PD6); // Wyłącz jazde do przodu
	set_bit(PORTD, PD7); // Włącz jazde do tylu
}

void rm_stop(void)
{
	set_bit(PORTB, PB2);
	set_bit(PORTD, PD6); // Włącz jazde do przodu
	set_bit(PORTD, PD7); // Włącz jazde do tylu
}

int calculate_error()
{
	c0 = c1 = c2 = c3 = c4 = 0; // Wyzerowanie pomiarów
	
	// Sprawdzenie każdego czujnika - jeśli odczytał czerń to wartość 1
	if (bit_is_clear(PINC, PC4))
	{
		last_known_line_side = 'r';
		c4 = 1;
	}
	if (bit_is_clear(PINC, PC3)) 
	{
		c3 = 1;
	}
	if (bit_is_clear(PINC, PC2))
	{
		c2 = 1;
	}
	if (bit_is_clear(PINC, PC1))
	{
		c1 = 1;
	}
	if (bit_is_clear(PINC, PC0))
	{
		last_known_line_side = 'l';
		c0 = 1;
	}

	// Sprawdzenie czy robot nie wyjechał poza czarną linię
	if (!(c0 || c1 || c2 || c3 || c4))		
	{	
		// Sprawdza czy miejsce ostatniej znanej linii bylo na prawo
		if(last_known_line_side == 'r')				
		{
			rm_move_backward();						
			lm_move_forward();
			OCR1A = 255;
			OCR1B = 255;
		}
		// Sprawdza czy miejsce ostatniej znanej linii bylo na lewo
		else if (last_known_line_side == 'l')		
		{
			rm_move_forward();						
			lm_move_backward();
			OCR1A = 255;
			OCR1B = 255;
		}
		previous_error = 0;
		return 0;
	}

	// Przypisanie do zmiennej perror wartość poprzedniego błędu
	previous_error = error;

	// Obliczenie błędu
	error = (c0 * 1) + (c1 * 2) + (c2 * 3) + (c3 * 4) + (c4 * 5);
	error = (error) / (c0 + c1 + c2 + c3 + c4);
	error = error - 3;

	return 1;
}

void move()
{
	if(calculate_error())
	{
		P = error * Kp;

		I += error;
		I = I * Ki;

		D = error - previous_error;

		correction = P + I + D;

		rightpulse = basespeed + correction;
		leftpulse = basespeed - correction;

		lm_move_forward();
		rm_move_forward();


		if (leftpulse > 255)
			leftpulse = 255;

		if (rightpulse > 255)
			rightpulse = 255;

		OCR1A = leftpulse;
		OCR1B = rightpulse;
	}
}


int main(void)
{
	initialize();

	while(true)
	{
		if(allow)
		{
			move();
			allow = 0;
		}

	}
}