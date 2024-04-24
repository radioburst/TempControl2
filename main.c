#include <avr/io.h>
#include "lcd-routines.h"
#include "main.h"

#define DEBOUNCE	256L		// debounce clock (256Hz = 4msec)

//EEPROM addresses
#define EEPROM_TEMPVALUE 1
#define EEPROM_TEMPSIGN 2
#define EEPROM_TEMPFLOAT 3
#define EEPROM_ID1START 10
#define EEPROM_ID2START 30
#define EEPROM_RESETFORCED 99
#define EEPROM_MODE 105

#define PHASE_A		(PIND & 1<<PD3)
#define PHASE_B		(PIND & 1<<PD4)

#define TEMPTIMEOUT 90 // min
#define TEMPSENSORTIMEOUT 2 // s

uchar prescaler;
volatile uint8_t uiMainSensorLastReading = 0, uiBeerSensorLastReading = 0;			// count seconds
volatile float Tsoll=8.0;
volatile uint8_t sleep = 1, iTempMes = 1, iTempSet = 0;
volatile uint8_t encoder_state=0;
volatile uint8_t encoder_buffer[]={'N','N','N','N'};
volatile int8_t enc_delta = 0, iInactiveSec = 0;
volatile uint16_t iTimerWait = 0;
volatile uint8_t iTimerSpeed = 0;
uint8_t uiRodaryPressActive = 0, uiRodaryPush = 0, uiMenuActive = 0;
int8_t iMode = 1;

volatile uchar id1[9] = "00000000", id2[9] = "00000000";
volatile float fTist = 0, fTist2 = 0;

void encode_init();
void init_timer();
int8_t encode_read4();
void saveSetTemp();
void saveTempSensorID(volatile uchar *id, uint8_t iStartAddress);
void readTempSensorID(volatile uchar *id, uint8_t iStartAddress);
void saveMode();
int8_t readMode();

enum TEMPMODE
{
	COOLING = 0,
	HEATING = 1
};

char *ftoa(char *a, double f, int precision)
{
	long p[] = {0,10,100,1000,10000,100000,1000000,10000000,100000000};
	
	char *ret = a;
	long heiltal = (long)f;
	itoa(heiltal, a, 10);
	while (*a != '\0') a++;
	*a++ = '.';
	long desimal = abs((long)((f - heiltal) * p[precision]));
	itoa(desimal, a, 10);
	return ret;
}

void startBeep()
{
	PORTD |= (1<<PD6);
	TIMSK |= 1<<TOIE0; 
	iTimerSpeed = 0;
}

void stopBeep()
{
	TIMSK &= ~(1<<TOIE0); 
	PORTD &= ~(1<<PD6);
}

int main()
{
	char clcd_TistBeer[15], clcd_Tist[9], clcd_Tsoll[9], clcd_Temp[5];
	int8_t enc_new_delta = 0;
	uint16_t uiOnTime = 0;
	int8_t iErrorState = NOERROR;
	
	for(uint8_t i = 0; i < 8; i++)
	{
		id1[i] = '0';
		id2[i] = '0';
	}
	
	id1[8] = '\0';
	id2[8] = '\0';

	MCUCR |= (1 << ISC11) | (1 << ISC10);   //Steigende Flanke von INT1 als ausl�ser
	GIFR &= ~(1<<INTF1);
	GICR |= (1 << INT1);                  //Global Interrupt Flag f�r INT1

	MCUCR |= (1<<ISC00);   //jede Flanke von INT0 als ausloeser
	//MCUCR |= (1<<ISC01) | (1<<ISC00);   //Steigende Flanke von INT0 als ausl�ser
	GIFR &= ~(1<<INTF0);
	GICR |= (1<<INT0);		//Global Interrupt Flag f�r INT0
	sei();								//Interrupt aktivieren

	// Ext interrupts pullup on
	PORTD |= (1<<DDD2);					// Int Pull UP
	PORTD |= (1<<DDD3);
	
	// relais output
	DDRB |= (1<<PB0);
	PORTB &= ~(1<<PB0);
	DDRB |= (1<<PB1);
	PORTB &= ~(1<<PB1);
	
	// lcd light
	DDRD |= (1<<PD1);
	PORTD |= (1<<PD1);
	
	// buzzer output
	DDRD |= (1<<PD6);
	PORTD &= ~(1<<PD6);
	// buzzer timer
	TCCR0 |= 1<<CS01; 
		
	DDRD &= ~(1<<DDD4);	// Rodary A
	DDRD &= ~(1<<DDD3); // Rodary B
	PORTD |= (1<<PD4);	// int. pull up
	PORTD |= (1<<PD3);	// int. pull up
	
	w1_reset();
	lcd_init();
	lcd_setcursor(0,1);
	lcd_string("Pflanzbeet Braeu");
	lcd_setcursor(0,2);
	lcd_string("TempControl v1.3");

	_delay_ms(1000);
	
	if(!(PIND & (1<<PD2)))	// setupmode
	{
		lcd_clear();
		lcd_string("Temp Sensor");
		lcd_setcursor(0, 2);
		lcd_string("Setup Modus");
		_delay_ms(2000);
		lcd_clear();
		lcd_string("Haupt Sensor");
		lcd_setcursor(0, 2);
		lcd_string("verbinden!");
		fTist = -99;
		while(fTist < -97)
		{
			read_meas();
			start_meas();
		}
		saveTempSensorID(id1, EEPROM_ID1START);
		
		lcd_clear();
		lcd_string("Haupt Sensor");
		lcd_setcursor(0, 2);
		lcd_string("gespeichert!");
		_delay_ms(2000);
		lcd_clear();
		lcd_string("Bier Sensor");
		lcd_setcursor(0, 2);
		lcd_string("verbinden!");
		fTist2 = -99;
		while(fTist2 < -97)
		{
			read_meas();
			start_meas();
		}
		saveTempSensorID(id2, EEPROM_ID2START);
		
		lcd_clear();
		lcd_string("Bier Sensor");
		lcd_setcursor(0, 2);
		lcd_string("gespeichert!");
		_delay_ms(2000);
		lcd_clear();
		lcd_string("Fertig!");
		_delay_ms(1500);
		lcd_clear();
		eeprom_write_byte((uint8_t*)EEPROM_RESETFORCED, (uint8_t)0);
	}	
	else
	{
		// get ids from eeprom
		readTempSensorID(id1, EEPROM_ID1START);
		readTempSensorID(id2, EEPROM_ID2START);
	}
	
	Tsoll = eeprom_read_byte((uint8_t*)EEPROM_TEMPVALUE);
	if(eeprom_read_byte((uint8_t*)EEPROM_TEMPSIGN) == 1)
		Tsoll*=(-1);
	if(eeprom_read_byte((uint8_t*)EEPROM_TEMPFLOAT) == 1)
		Tsoll+=0.5;	
	
	iMode = readMode();

	// initialize temp sensor (first value is not valid)
	fTist = -99;
	while(fTist < -97)
	{
		read_meas();
		start_meas();
	}

	init_timer();
	encode_init();
	wdt_enable(WDTO_1S);
	lcd_clear();
	
	while(1)
	{	
		set_sleep_mode(SLEEP_MODE_IDLE );
		sleep_mode();
					
		if(sleep == 1)
		{
			sleep = 0;	
			if(iTempMes == 1)
			{
				iTempMes = 0;
				read_meas();
				start_meas();
			}

			if(iMode == COOLING)
				PORTB &= ~(1<<PB1);		// HEATING OFF
			else
				PORTB &= ~(1<<PB0);		// COLLING OFF

			//// Regler /////////////////////////////////////////
			if(iErrorState > NOERROR)
			{
				PORTB &= ~(1<<PB0);		// COLLING OFF
				PORTB &= ~(1<<PB1);		// HEATING OFF
			}
			else
			{
				if(fTist < Tsoll - 0.4)
				{
					if(iMode == COOLING)
					{
						PORTB &= ~(1<<PB0);		// COLLING OFF
						uiOnTime = 0;
					}
					else if(uiOnTime == 0)
					{
						PORTB |= (1<<PB1);		// HEATING ON
						uiOnTime++;
					}
				}
				else if(fTist >= (Tsoll + 0.5))
				{
					if(iMode == COOLING)
					{
						if(uiOnTime == 0)
							PORTB |= (1<<PB0);		// COLLING ON
						uiOnTime++;
					}
					else
					{
						PORTB &= ~(1<<PB1);		    // HEATING OFF
						uiOnTime = 0;
					}
				}
			}
			/////////////////////////////////////////////////////
				
			if(uiOnTime > TEMPTIMEOUT * 60 && iErrorState != MAINSENSORERROR) // timeout reached!!
			{	
				uint8_t iRecentError = eeprom_read_byte((uint8_t*)EEPROM_RESETFORCED);
				if(iRecentError > 0)
				{
					iErrorState = RUNTIMEERROR;
					eeprom_write_byte((uint8_t*)EEPROM_RESETFORCED, (uint8_t)0);	// reset error state flag for next start up since now it will run in idle for ever disabling the cooler
				}
				else
				{	
					eeprom_write_byte((uint8_t*)EEPROM_RESETFORCED, (uint8_t)1);	// set error flag for the next restart
					while(1){} // force watchdog reset
				}
			}

			if(uiMainSensorLastReading > TEMPSENSORTIMEOUT)
				iErrorState = MAINSENSORERROR;
			else if(iErrorState == MAINSENSORERROR && uiMainSensorLastReading < TEMPSENSORTIMEOUT) // Reset error when sensor is working again
			{
				iErrorState = NOERROR;
				lcd_setcursor(12,2);
				lcd_string("    ");
			}
				
			//// Ausgabe vorbereiten ///////////////////////////////////		

			if(uiMenuActive < 1)
			{
				if(iErrorState == MAINSENSORERROR)
					sprintf(clcd_Tist, "  ---%cC ", (char) 223);
				else
				{
					if(fTist < 0)
						sprintf(clcd_Tist, " %s%cC ", ftoa(clcd_Temp, fTist, 1), (char) 223);
					else if(fTist < 10)
						sprintf(clcd_Tist, "  %s%cC ", ftoa(clcd_Temp, fTist, 1), (char) 223);
					else if(fTist < 100)
						sprintf(clcd_Tist, " %s%cC ", ftoa(clcd_Temp, fTist, 1), (char) 223);
					else
						sprintf(clcd_Tist, "%s%cC ", ftoa(clcd_Tsoll, fTist, 1), (char) 223);
				}

				if(Tsoll < 100)
					sprintf(clcd_Tsoll, " %s%cC ", ftoa(clcd_Temp, Tsoll, 1), (char) 223);
				else
					sprintf(clcd_Tsoll, "%s%cC ", ftoa(clcd_Tsoll, Tsoll, 1), (char) 223);

				if(uiBeerSensorLastReading > TEMPSENSORTIMEOUT)
					sprintf(clcd_TistBeer, "Bier: ---%cC  ", (char) 223);
				else
					sprintf(clcd_TistBeer, "Bier: %s%cC  ", ftoa(clcd_Temp, fTist2, 1), (char) 223);

				//// Ausgabe ///////////////////////////////////
				lcd_clear();
				lcd_setcursor(0,1);
				lcd_string(clcd_Tist);

				lcd_setcursor(9,1);
				lcd_string(clcd_Tsoll);

				lcd_setcursor(0,2);
				lcd_string(clcd_TistBeer);

				if(iErrorState == RUNTIMEERROR)
				{
					lcd_setcursor(12,2);
					lcd_string("LErr");
				}
				else if(iErrorState == MAINSENSORERROR)
				{
					lcd_setcursor(12,2);
					lcd_string("SErr");
				}
				else
				{
					lcd_setcursor(14,2);
					if(iMode == COOLING)
						lcd_string("K");
					else
						lcd_string("H");
				}
			}
			else
			{
				lcd_clear();
				lcd_setcursor(0,1);
				if(iMode == COOLING)
					lcd_string("Modus: Kuehlen");
				else
					lcd_string("Modus: Heitzen");
			}
		}
		
		if(iTempSet == 1 || uiMenuActive)
		{
			enc_new_delta = encode_read4();
			if(enc_new_delta != 0)
			{
				if(uiMenuActive)
				{
					iMode += enc_new_delta;

					if(iMode > HEATING)
						iMode = HEATING;
					else if(iMode < COOLING)
						iMode = COOLING;
				}
				else
					Tsoll += ((float)enc_new_delta / 2);
				sleep = 1;
				iInactiveSec = 0;
			}
		}
		
		if(iInactiveSec > 9 && iTempSet == 1)
		{
			saveSetTemp();
			TIMSK &= ~(1<<OCIE2);
			iTempSet = 0;
			PORTD &= ~(1<<PD1);
			GICR |= (1<<INT1);
		}
		else if(iInactiveSec > 9 && !uiMenuActive)
			PORTD &= ~(1<<PD1);
		
		if(uiRodaryPush == 1)
		{
			uiRodaryPush = 0;
			uiRodaryPressActive = 0;
			if(iTempSet)
			{
				saveSetTemp();
				TIMSK &= ~(1<<OCIE2);
				iTempSet = 0;
				GICR |= (1<<INT1);
			}
			else
			{
				TIMSK |= 1<<OCIE2;
				iTempSet = 1;
				PORTD |= (1<<PD1);
				GICR &= ~(1<<INT1);
			}

			if(uiMenuActive)
			{
				saveMode();
				TIMSK &= ~(1<<OCIE2);
				GICR |= (1<<INT1);
				uiMenuActive = 0;
			}
		}
		else if(uiRodaryPush == 2 || (uiRodaryPressActive && iInactiveSec > 1))
		{
			uiRodaryPush = 0;
			uiRodaryPressActive = 0;
			uiMenuActive = 1;
			TIMSK |= 1<<OCIE2;
			PORTD |= (1<<PD1);
			GICR &= ~(1<<INT1);
		}

		wdt_reset();		
	}
	
	return 0;
}

void saveSetTemp()
{	
	if(Tsoll < 0.0) // minus
	{
		eeprom_write_byte ((uint8_t*) EEPROM_TEMPSIGN, (uint8_t)1);
		eeprom_write_byte ((uint8_t*) EEPROM_TEMPVALUE, (uint8_t)(Tsoll*(-1)));
	}			
	else
	{
		eeprom_write_byte ((uint8_t*) EEPROM_TEMPSIGN, (uint8_t)0);
		eeprom_write_byte ((uint8_t*) EEPROM_TEMPVALUE, (uint8_t)Tsoll);
	}
					
	if(Tsoll == (int)Tsoll) // nachkommerstelle
		eeprom_write_byte ((uint8_t*) EEPROM_TEMPFLOAT, (uint8_t)0);			
	else
		eeprom_write_byte ((uint8_t*) EEPROM_TEMPFLOAT, (uint8_t)1);
}

///////////////////////////////////////////////////////////
/*ISR(INT0_vect) // rodary push
{
	iInactiveSec = 0;
	if(iTempSet)
	{
		saveSetTemp();
		TIMSK &= ~(1<<OCIE2); 
		iTempSet = 0;
		GICR |= (1<<INT1);
	}
	else
	{
		TIMSK |= 1<<OCIE2; 
		iTempSet = 1;
		PORTD |= (1<<PD1);
		GICR &= ~(1<<INT1);
	}
} */


ISR(INT0_vect)
{
	if(!(PIND & (1<<PC2)))
	{
		iInactiveSec = 0;
		uiRodaryPressActive = 1;
	}
	else if(uiRodaryPressActive)
	{
		if(iInactiveSec > 1)
			uiRodaryPush = 2;
		else
			uiRodaryPush = 1;
	}
}


ISR(INT1_vect) // rodary turn
{
	PORTD |= (1<<PD1);
	iInactiveSec = 0;
}

ISR (TIMER1_COMPA_vect)
{
	uchar tcnt1h = TCNT1H;

	OCR1A += F_CPU / DEBOUNCE;		// new compare value

	if( ++prescaler == (uchar)DEBOUNCE ){
		prescaler = 0;
		uiMainSensorLastReading++;
		uiBeerSensorLastReading++;
		iInactiveSec++;
		sleep = 1;	
		iTempMes = 1;
		#if F_CPU % DEBOUNCE			// handle remainder
		OCR1A += F_CPU % DEBOUNCE; 		// compare once per second
		#endif
	}
	TCNT1H = tcnt1h;			// restore for delay() !
}


void init_timer( void )
{
	TCCR1B = 1<<CS10;			// divide by 1
	OCR1A = 0;
	TCNT1 = -1;
	prescaler = 0;
	TIMSK |= 1<<OCIE1A;
}

void encode_init()
{
	TCCR2 = 1<<WGM21 | 1<<CS22;		// CTC, XTAL / 64
	OCR2 = (uint8_t)(F_CPU / 64 * 1e-3 - 0.5);	// 2ms
	TIMSK &= ~(1<<OCIE2); 
}

int8_t encode_read4()			// read four step encoders
{
	int8_t val;

	cli();
	val = enc_delta;
	enc_delta &= 3;
	sei();
	return val >> 2;
}

ISR(TIMER2_COMP_vect)				// 2ms for manual movement
{
	static int8_t last;
	int8_t new, diff;

	new = 0;
	if( PHASE_A )
	new = 3;
	if( PHASE_B )
	new ^= 1;					// convert gray to binary
	diff = last - new;				// difference last - new
	if( diff & 1 ){				// bit 0 = value (1)
		last = new;					// store new as next last
		enc_delta += (diff & 2) - 1;		// bit 1 = direction (+/-)
	}
}

ISR(TIMER0_OVF_vect)
{
	if(iTimerSpeed > 3)
	{
		if(iTimerWait < 250)
			DDRD ^= (1<<DDD6);
		
		if(iTimerWait > 750)
			iTimerWait = 0;
		
		iTimerWait++;
		iTimerSpeed = 0;
	}
	iTimerSpeed++;
}

void saveTempSensorID(volatile uchar *id, uint8_t iStartAddress)
{
	for(int i = 0; i < 9; i++)
		eeprom_write_byte ((uint8_t*) (iStartAddress + i), (uint8_t) id[i]);
}

void readTempSensorID(volatile uchar *id, uint8_t iStartAddress)
{
	for(int i = 0; i < 9; i++)
		id[i] = eeprom_read_byte((uint8_t*)(iStartAddress + i));
}

void saveMode()
{
	eeprom_write_byte ((uint8_t*) EEPROM_MODE, (uint8_t) iMode);
}

int8_t readMode()
{
	return eeprom_read_byte((uint8_t*)(EEPROM_MODE));
}
