/*
 * flamingo.c
 *
 * Created: 10/19/2014 11:22:42 AM
 *  Author: Aaron Klingaman, Limitedslip Engineering LLC
 */ 

#define F_CPU	8000000UL

#include <avr/io.h>
#include <avr/interrupt.h>
#include <avr/sleep.h>
#include <avr/eeprom.h>
#include <util/delay.h>
#include <stdlib.h>


// double buffered 4 channel software PWM.

// internal state variables below must always be sorted.
// use the functions start_pwm to add a new channel
// or update an existing one. stop_pwm to remove one

// num_pwm_channels has the used channel count, 
// pwm_duty[0..N] for each channel (timer compare
// value for off portion), and set which PORTB pin
// that channel corresponds to in pwm_pin_map.
// changes to the PWM duty should be set in pending_pwm_duty
// and mark new_pwm_duty_pending to 1, which will reload
// the new values at the next timer overflow.

#define MAX_PWM_CHANNELS 4 

uint8_t num_pwm_channels= 0;
uint8_t current_pwm_index= 0;

uint8_t pwm_duty[MAX_PWM_CHANNELS];
uint8_t pending_pwm_duty[MAX_PWM_CHANNELS];

uint8_t pwm_pin_map[MAX_PWM_CHANNELS];
uint8_t pending_pwm_pin_map[MAX_PWM_CHANNELS];

uint8_t new_pwm_duty_pending= 0;


// where the next random seed is in eeprom (4 bytes)
#define SEED_ADDR		0



// start or update one of the PWM channels. specify the PORTB pin
// number and timer duty is 0 to 255 for the next TIMER0 compare.
// return 1 if updated/added, 0 if failure (too many PWM channels)
uint8_t start_pwm( uint8_t portb_pin, uint8_t timer_duty )
{
	// load into buffer for next timer overload interrupt.
	if( num_pwm_channels == 0 )
	{
		pending_pwm_duty[0]= timer_duty;
		pending_pwm_pin_map[0]= portb_pin;
		num_pwm_channels= 1;
	}
	else
	{
		// if portb_pin is already specified, update it, otherwise,
		// append if we haven't hit the max channel count
		
		uint8_t updated= 0;
		for( int i= 0; i < num_pwm_channels; ++i )
		{
			if( pending_pwm_pin_map[i] == portb_pin )
			{
				pending_pwm_duty[i]= timer_duty;
				updated= 1;
				break;
			}
		}
		
		if( !updated && num_pwm_channels == MAX_PWM_CHANNELS )
			return 0;
			
		if( !updated )
		{
			pending_pwm_duty[num_pwm_channels]= timer_duty;
			pending_pwm_pin_map[num_pwm_channels]= portb_pin;
			num_pwm_channels++;
		}
		
		// insertion sort by time_duty.  might be able to avoid a bit
		// of time if we did the insertion sort during the update check
		for( uint8_t i= 1; i < num_pwm_channels; ++i )
		{
			uint8_t j= i;
			while( j > 0 && pending_pwm_duty[j-1] > pending_pwm_duty[j] )
			{
				uint8_t temp;
				
				temp= pending_pwm_duty[j-1];
				pending_pwm_duty[j-1]= pending_pwm_duty[j];
				pending_pwm_duty[j]= temp;
				
				temp= pending_pwm_pin_map[j-1];
				pending_pwm_pin_map[j-1]= pending_pwm_pin_map[j];
				pending_pwm_pin_map[j]= temp;
				
				j--;
			}
		}
		
	}
	
	new_pwm_duty_pending= 1;
	
	return 1;
}


// return 1 if stopped and removed or 0 if it wasn't present
uint8_t stop_pwm( uint8_t portb_pin )
{
	uint8_t i= 0;
	while( i < num_pwm_channels )
	{
		if( pending_pwm_pin_map[i] == portb_pin )
			break;
			
		++i;
	}
	
	if( i == num_pwm_channels )
	{
		// wasn't present
		return 0;
	}
	
	// remove, moving the entries after it up
	for( ; i < num_pwm_channels-1; ++i )
	{
		pending_pwm_duty[i]= pending_pwm_duty[i+1];
		pending_pwm_pin_map[i]= pending_pwm_pin_map[i+1];
	}
	
	num_pwm_channels--;
	
	PORTB &= ~_BV(portb_pin);
	
	new_pwm_duty_pending= 1;
	
	return 1;
}


ISR(TIMER0_OVF_vect)
{
	if( new_pwm_duty_pending )
	{
		// reload PWM values and the port bit mapping (co-sorted)
		for( uint8_t i= 0; i < num_pwm_channels; ++i )
		{
			pwm_duty[i]= pending_pwm_duty[i];
			pwm_pin_map[i]= pending_pwm_pin_map[i];
		}
		
		new_pwm_duty_pending= 0;
	}
	
	// all on at the start of the timer overflow
	for( uint8_t i= 0; i < num_pwm_channels; ++i )
		PORTB |= _BV( pwm_pin_map[i] );
	
	// start at the first PWM duty to switch (closest in time)
	current_pwm_index= 0;
	
	if( num_pwm_channels > 0 )
	{
		OCR0A= pwm_duty[current_pwm_index];
	}
}


ISR(TIMER0_COMPA_vect)
{
	// turn off the one the compare value was set up for
	// and others if they are the same
	for( uint8_t i= current_pwm_index; i < num_pwm_channels; ++i )
	{
		if( pwm_duty[i] == pwm_duty[current_pwm_index] )
		{
			PORTB &= ~_BV(pwm_pin_map[i]);
		}		
		else if( pwm_duty[i] > pwm_duty[current_pwm_index] )
		{
			current_pwm_index= i;
			break;
		}
	}
	
	// set up for the next one in the list
	OCR0A= pwm_duty[current_pwm_index];
}


uint32_t range_rand( uint32_t upper )
{
	return (uint32_t)random() / (RANDOM_MAX / upper + 1);
}


void seed_rand()
{
	uint32_t next_seed = eeprom_read_dword( (uint32_t*)SEED_ADDR );
	
	srandom( next_seed );
	
	eeprom_write_dword( (uint32_t*)SEED_ADDR, random() );
}



void setup_mcu()
{
	// shutdown some peripherals for some
	// power saving (USART, USI, and TIMER1)
	PRR |= ( _BV(PRTIM1) | _BV(PRUSI) | _BV(PRUSART) );
}


void pwm_timer_init()
{
	// timer 0 with 256 prescaling ( 1/8e6 * 256 = 32us, 31.25kHz
	TCCR0B |= ( _BV(CS02) );
	TCNT0 = 0;
	
	// overflow and match A interrupt on
	TIMSK |= ( _BV(TOIE0) | _BV(OCIE0A) );
}


void led_driver_init()
{
	// all output and initially off
	DDRB |= ( _BV(PB1) | _BV(PB2) | _BV(PB3) | _BV(PB4) );
	PORTB &= ~( _BV(PB1) | _BV(PB2) | _BV(PB3) | _BV(PB4) );
}


void pir_init()
{
	// input, but no pull up needed
	DDRD &= ~( _BV(PD2) );
}


int main()
{
	seed_rand();
	setup_mcu();
	led_driver_init();
	pwm_timer_init();
	pir_init();
	
	// the PIR sensor takes a few seconds
	_delay_ms(5000);
	
	sei();
	
	uint8_t lower_pwm= 1;
	uint8_t upper_pwm= 70;
	uint8_t base_delay_interval= 10;
	
	while(1)
	{
		while( !(PIND && PD2) );
		
		uint8_t pins[4]= { PB1, PB2, PB3, PB4 };
		uint8_t pin_starts[4];
		uint8_t pin_pwm[4];
		uint16_t delay_count= 0;
		uint8_t still_fading= 1;
			
		for( uint8_t i= 0; i < 4; ++i )
		{
			// number of base_delay_interval intervals to wait before lighting them up
			pin_starts[i]= range_rand(50);
			pin_pwm[i]= lower_pwm;
		}
			
		// fade them up
		delay_count= 0;
		still_fading= 1;
		while( still_fading )
		{
			still_fading= 0;
				
			for( uint8_t i= 0; i < 4; ++i )
			{
				if( delay_count >= pin_starts[i] && pin_pwm[i] < upper_pwm )
				{
					pin_pwm[i]++;
					start_pwm( pins[i], pin_pwm[i] );
				}
					
				if( pin_pwm[i] < upper_pwm )
					still_fading= 1;
			}
				
			_delay_ms( base_delay_interval );
			delay_count++;
		}
			
		// wait for no more motion for 5 seconds
		delay_count= 0;
		while( delay_count < (5000/base_delay_interval) )
		{
			if( PIND && PD2 )
			{
				delay_count= 0;
				continue;
			}
			
			_delay_ms( base_delay_interval );
			delay_count++;
		}
		
		// fade them off
		for( uint8_t i= 0; i < 4; ++i )
		{
			// number of base_delay_interval intervals to wait before shutting them down
			pin_starts[i]= range_rand(60);
			pin_pwm[i]= upper_pwm;
		}
		
		delay_count= 0;
		still_fading= 1;
		while( still_fading )
		{
			still_fading= 0;
			
			for( uint8_t i= 0; i < 4; ++i )
			{
				if( delay_count >= pin_starts[i] )
				{
					if( pin_pwm[i] > lower_pwm )
					{
						pin_pwm[i]--;
					
						start_pwm( pins[i], pin_pwm[i] );
					}
					
					if( pin_pwm[i] <= lower_pwm )
						stop_pwm( pins[i] );
				}
				
				if( pin_pwm[i] > lower_pwm )
					still_fading= 1;
			}
			
			_delay_ms( base_delay_interval );
			delay_count++;
		}
		
		_delay_ms(2000);
	}
	
	return 0;
}




