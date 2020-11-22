/*
 * ATV Control
 * Rev 1.0
 *
 * The main application of this module is to replace three mechanical relays, two for for LEDs and one
 * for a horn on my ATV.
 *
 * The main output is the Horn which supports up to 20A. A pushbutton switch is used to honk the Horn but 
 * only when the ATV is on (Ignition input 12V). The other LED Outputs (V1 and V2) are turned off just 
 * before the Horn is activated. This keeps the total board maximum current to 20A. The other LED outputs 
 * will return to their previous state once the Horn pushbutton is released. The LED indicator on the Horn.
 * switch is RGB. When the ATV is turned on (Ignition input 12V) the Horn indicator LED will cycle through
 * the color spectrum. When the Horn is engaged the indicator LED will flash red.
 *
 * The other two 12V outputs (V1 and V2) support 7.5A each and are controlled by pushbutton switches 
 * (Switch 1 and Switch 2). These pushbutton switches will turn off/on the outputs at any time. If the ATV
 * is off (Ignition input 0V) and an Output is turned on it will remain on for a user configurable amount 
 * of time before automatically turning off. When the ATV is turned off (Ignition input 0V) all Outputs are 
 * automatically turned off. These outputs are connected to TCD5 which would allow dimming of LEDs connected
 * to them but this is currently not part of the code below.
 *
 * In addition to pushbutton control the outputs are automatically enabled by other control inputs. 
 * V1 Output is turned on when the High beam input is turned on (HB input 12V).
 * V2 Output is turned on when the ATV is put into reverse (REV input 12V).
 * When V1 and V2 Outputs are turned on via HB or REV inputs the LED indicator intensity breathes.
 *
 * Delay time to turn Outputs off when Ignition is off is configurable.
 *
 * To enter program mode do the following:
 *  1. Press and Hold both Switch 1 and Switch 2 for 10 seconds.
 *  2. Release both Switch 1 and Switch 2.
 *  3. Switch LEDs should be flashing.
 *  4. Delay time is now defaulted to 0 which means outputs cannot be turned on when Ignition is off.
 *  5. Each time you press and release either switch you will add one minute to the delay.
 *     While either switch is pressed the LED will stop flashing and remain lit.
 *  6. Once you have configured the desired delay wait for 10 seconds (LEDs will be flashing).
 *  7. When the 10 seconds has elapsed the LEDS will slow flash the newly configured number of minutes
 *     as confirmation.
 *
 * Created: 10/25/2020
 * Author : Mike Lawrence
 */

/*
 * System Clock is kept at the power on default of Internal 2MHz. Internal 32.768 kHz clock is also enabled.
 *
 * RTC clock is set to Internal 32.768kHz. RTC is configured for a 1ms overflow. The overflow rate is 0.708%
 * longer than 1ms but precision timing is not the goal here. The RTC Overflow Interrupt is used to keep track 
 * of millisecond counters (tick_ms, delay_ms, prog_ms and led_ms).
 * 
 * The HSW, SW1, SW2, IGN, HB and REV inputs are configured so that any input change will automatically wake
 * the processor from an idle state. The idle state is entered only when Ignition is off (IGN at 0V) and no 
 * LED outputs (V1 and V2) are currently on. HSW, HB and REV inputs are ignored when Ignition is off.
 *
 * All indicator LEDs (HSWLR_EN, HSWLG_EN, HSWLG_EN, SW1L_EN and SW2L_EN) outputs are controlled by 
 * Output Compares which allows PWM of the LED (brightness adjustment).
 * 
 * The main LED outputs V1 and V2 are connected to pins that can be used as Output Compares. So it is possible
 * to use PWM to modulate Output LED light intensity. Currently this code does not support this.
 * Both V1 and V2 can a fixed maximum and a soft start and stop.
 *
 */

#include <avr/io.h>
#include <avr/eeprom.h>
#include <avr/fuse.h>
#include <avr/interrupt.h>
#include <avr/pgmspace.h>
#include <avr/sleep.h>
#include <avr/wdt.h>
#include "math.h"

/*
 * Fuse definitions
 */
FUSES =
{
	// FuseByte 1 = Watchdog timeouts set to 1,000 clocks or 1 s
	.FUSEBYTE1 = 0xFF & (WDWPER_1KCLK_gc | ~NVM_FUSES_WDWPER_gm) & (WDPER_1KCLK_gc | ~NVM_FUSES_WDPER_gm),		
	// FuseByte 2 = Application reset vector enabled, BOD is sampled in Power Down Mode
	.FUSEBYTE2 = 0xFF & (BOOTRST_APPLICATION_gc | ~NVM_FUSES_BOOTRST_bm) & (BODPD_SAMPLED_gc | ~NVM_FUSES_BODPD_gm),		
	// FuseByte 4 = Ext Reset is not disabled, Start Up Time is 0 ms, Watchdog timer is not locked 
	.FUSEBYTE4 = 0xFF & /* FUSE_RSTDISBL & */ (STARTUPTIME_0MS_gc | ~NVM_FUSES_STARTUPTIME_gm) /* & FUSE_WDLOCK */,							
	// FuseByte 5 = BOD continuous in Active Modes, EEPROM saved during chip erase, BOD Level is 2.0V
	.FUSEBYTE5 = 0xFF & (BODACT_CONTINUOUS_gc | ~NVM_FUSES_BODACT_gm) & FUSE_EESAVE & (BODLEVEL_2V0_gc | ~NVM_FUSES_BODLEVEL_gm), 
	// FuseByte 5 = Timer fault and detection defaults
	.FUSEBYTE6 = 0xFF
};

/*
 * Application specific definitions
 */
#define FALSE							0
#define TRUE							1
#define OFF								FALSE
#define ON								TRUE
// Default number of minutes LEDs stay on when turned one with Ignition Off
#define DEFAULT_DELAY_TIME_MINUTES		5
// Number of seconds to activate programming sequence
#define PROG_ACTIVATE_SECONDS			10
// Number of milliseconds for LED to be ON or OFF when flashing
#define LED_FLASH_TIME					500
// Timer Period that will produce normal PWM frequency for LED outputs
// FREQ = CPU_FREQ / 2 * 64 * PER (PER = 255 = 61.27 Hz)
#define OUT_PWM_PERIOD					255
// Timer Period that will produce normal PWM frequency 
// FREQ = CPU_FREQ / 2 * 64 * PER (PER = 255 = 61.27 Hz)
#define LED_PWM_PERIOD					255
// Timer Period that will produce a visible flash for Switch LED indicators
// FREQ = CPU_FREQ / 2 * 64 * PER (PER = 2047 = 7.65 Hz)
#define LED_FLASH_PERIOD				1500
// input debounce time in milliseconds (cannot be larger than 255)
#define DEBOUNCE_TIME					5
// Watchdog timeout setting
#define WATCHDOG_TO						WDTO_2S
// Output defines
#define HEN_port						PORTD
#define HEN_bp							PIN3_bp
#define V1EN_port						PORTD
#define V1EN_bp							PIN4_bp
#define V2EN_port						PORTD
#define V2EN_bp							PIN5_bp
#define HSWLREN_port					PORTC
#define HSWLREN_bp						PIN3_bp
#define HSWLGEN_port					PORTC
#define HSWLGEN_bp						PIN2_bp
#define HSWLBEN_port					PORTC
#define HSWLBEN_bp						PIN1_bp
#define SWL1EN_port						PORTC
#define SWL1EN_bp						PIN5_bp
#define SWL2EN_port						PORTC
#define SWL2EN_bp						PIN4_bp
// Input defines
#define IGN_port						PORTA
#define IGN_bp							PIN3_bp
#define REV_port						PORTA
#define REV_bp							PIN2_bp
#define HB_port							PORTA
#define HB_bp							PIN4_bp
#define HSW_port						PORTA
#define HSW_bp							PIN0_bp
#define SW2_port						PORTA
#define SW2_bp							PIN1_bp
#define SW1_port						PORTC
#define SW1_bp							PIN6_bp

/*
 * Constants
 */

const uint8_t PROGMEM quarter_sine[64] = {128, 131, 134, 137, 140, 143, 146, 149, 
										  152, 155, 158, 162, 165, 167, 170, 173,
										  176, 179, 182, 185, 188, 190, 193, 196,
										  198, 201, 203, 206, 208, 211, 213, 215,
										  218, 220, 222, 224, 226, 228, 230, 232, 
										  234, 235, 237, 238, 240, 241, 243, 244,
										  245, 246, 248, 249, 250, 250, 251, 252,
										  253, 253, 254, 254, 254, 255, 255, 255};
/*
 * Enumerations
 */
enum POWER_SM  { SM_POWER_RESET = 0, SM_POWER_DOWN, SM_POWER_ON_IGN, SM_POWER_ON_SW };
enum PROG_SM   { SM_PROG_RESET = 0, SM_PROG_ACTIVATE, SM_PROG_WAIT, SM_PROG_ON_WAIT, SM_PROG_OFF_WAIT, SM_PROG_DISPLAY_DWELL, SM_PROG_DISPLAY};
enum LED_STATE { LED_OFF = 0, LED_ON, LED_BREATHE, LED_FLASH };
enum SW_TOGGLE { TOGGLE_OFF = 0, TOGGLE_ON, TOGGLE_ON_USER };
enum SW_LED    { SW1_LED = 1, SW2_LED = 2, SW12_LED = 3 };

/* 
 * EEPROM variables
 */
uint32_t EEMEM eeprom_delay_time_ms = DEFAULT_DELAY_TIME_MINUTES * 60ul * 1000ul;

/*
 * Global variables (Note best code optimization (code size) requires all defaults to be 0)
 */
volatile uint8_t  tick_ms = 0;							// Free-running milliseconds tick counter (255 milliseconds max)
volatile uint32_t delay_ms = 0;							// Delay milliseconds counter
volatile uint32_t prog_ms = 0;							// Program milliseconds counter
volatile uint16_t led_ms = 0;							// LED milliseconds counter

volatile uint8_t  ign_db = FALSE;						// Ignition currently debouncing
volatile uint8_t  ign_dbtime = 0;						// Ignition debounce start time
volatile uint8_t  ign_cur = OFF;						// Current Ignition state
volatile uint8_t  ign_last = ON;						// Last Ignition state

volatile uint8_t  rev_db = FALSE;						// Reverse currently debouncing
volatile uint8_t  rev_dbtime = 0;						// Reverse debounce start time
volatile uint8_t  rev_cur = OFF;						// Current Reverse state
volatile uint8_t  rev_last = ON;						// Last Reverse state

volatile uint8_t  hb_db = FALSE;						// High beam currently debouncing
volatile uint8_t  hb_dbtime = 0;						// High beam debounce start time
volatile uint8_t  hb_cur = OFF;							// Current High beam state
volatile uint8_t  hb_last = ON;							// Last High beam state

volatile uint8_t  hsw_db = FALSE;						// Horn Switch currently debouncing
volatile uint8_t  hsw_dbtime = 0;						// Horn Switch debounce start time
volatile uint8_t  hsw_cur = OFF;						// Current Horn Switch state
volatile uint8_t  hsw_last = ON;						// Last Horn Switch state

volatile uint8_t  sw1_db = FALSE;						// Switch 1 currently debouncing
volatile uint8_t  sw1_dbtime = 0;						// Switch 1 debounce start time
volatile uint8_t  sw1_cur = OFF;						// Current Switch 1 state
volatile uint8_t  sw1_toggle = TOGGLE_OFF;				// Switch 1 toggle state
volatile uint8_t  sw1_led_intensity = 0;				// Current Switch 1 LED intensity (0-255)
volatile uint8_t  sw1_led_state = LED_OFF;				// Current Switch 1 LED state

volatile uint8_t  sw2_db = FALSE;						// Switch 2 currently debouncing
volatile uint8_t  sw2_dbtime = 0;						// Switch 2 debounce start time
volatile uint8_t  sw2_cur = OFF;						// Current Switch 2 state
volatile uint8_t  sw2_toggle = TOGGLE_OFF;				// Switch 2 toggle state
volatile uint8_t  sw2_led_intensity = 0;				// Current Switch 2 LED intensity (0-255)
volatile uint8_t  sw2_led_state = LED_OFF;				// Current Switch 2 LED state

/*
 * Return sine wave values offset at 128.
 *  Angle is 0-255 and represents a full period. 
 *  Output is 0-255. Non-negative.
 */
uint8_t get_sine(uint8_t angle)
{
	volatile uint8_t quad = (angle & 0xC0) >> 6;					// what quadrant of sine wave is this angle
	volatile uint8_t newang = angle;
	volatile uint8_t ang = newang & 0x3F;							// force angle into quad angle range 0-63
	
 	if (quad & 0x01)
 	{
 		ang = 63 - ang;									// angle should be reversed (Quadrant 2 and 4)
 	}
	uint8_t val = pgm_read_byte(&quarter_sine[ang]);	// get the value from the table
 	if (quad & 0x02)
 	{
 		val = 255 - val;								// invert value (Quadrant 3 and 4)
 	}
	return val;
}

/*
 * Return sine wave peak values no offset. 
 *  Any negative value returns a 0.
 *  Angle is 0-255 and represents a full period. 
 *  Output is 0-255. Non-negative.
 */
uint8_t get_sine_peak(uint8_t angle)
{
	volatile uint8_t quad = (angle & 0xC0) >> 6;					// what quadrant of sine wave is this angle
	volatile uint8_t newang = angle;
	volatile uint8_t ang = newang & 0x3F;							// force angle into quad angle range 0-63
	
 	if (quad & 0x01)
 	{
 		ang = 63 - ang;									// angle should be reversed (Quadrant 2 and 4)
 	}
	uint8_t val = pgm_read_byte(&quarter_sine[ang]);	// get the value from the table
 	if (quad & 0x02)
 	{
 		val = 255 - val;								// invert value (Quadrant 3 and 4)
 	}
	if (val >= 128)
	{
		return (val - 128) << 1;
	}
	else
	{
		return 0;
	}
}

/* 
 * Turn output V1 and V2 off
 */
void v12_off(void)
{
	TCD5.CCABUF = 0;									// OC5A set to 0% duty cycle (V1_EN)
	TCD5.CCBBUF = 0;									// OC5B set to 0% duty cycle (V2_EN)
	TCD5.CTRLGSET = TC_CMD_UPDATE_gc;					// Force TCD5 timer UPDATE
}

/* 
 * Turn output V1 on
 */
void v1_on(void)
{
	TCD5.CCABUF = OUT_PWM_PERIOD;						// OC5A set to 100% duty cycle (V1_EN)
}

/* 
 * Turn output V1 off
 */
void v1_off(void)
{
	TCD5.CCABUF = 0;									// OC5A set to 0% duty cycle (V1_EN)
}

/* 
 * Turn output V2 on
 */
void v2_on(void)
{
	TCD5.CCBBUF = OUT_PWM_PERIOD;						// OC5B set to 100% duty cycle (V2_EN)
}

/* 
 * Turn output V2 off
 */
void v2_off(void)
{
	TCD5.CCBBUF = 0;									// OC5B set to 100% duty cycle (V2_EN)
}

/* 
 * Set Switch 1 and 2 LED Indicators
 *   led indicates which LED (SW1_LED, SW2_LED or SW12_LED)
 *   state indicates the desired state (LED_OFF, LED_ON, LED_BREATH or LED_FLASH)
 */
void swl12_set(uint8_t led, uint8_t state)
{
	cli();												// Disable global interrupts
	switch (state)
	{
	case LED_ON:
		// Time to turn the Switch LED Indicators full on
		if ((led & 0x1) && (sw1_led_state != LED_ON))
		{
			sw1_led_state = LED_ON;
			TCC5.CCBBUF = LED_PWM_PERIOD;				// OC5B set to 100% duty cycle
		}
		if ((led & 0x2) && (sw2_led_state != LED_ON))
		{
			sw2_led_state = LED_ON;
			TCC5.CCABUF = LED_PWM_PERIOD;				// OC5A set to 100% duty cycle
		}
		if (TCC5.PERBUF != LED_PWM_PERIOD)
		{
			// The LEDs are set to 100% duty cycle so the outputs are always on
			TCC5.PERBUF = LED_PWM_PERIOD;
			TCC5.CTRLGSET = TC_CMD_RESTART_gc;			// Restart TCC5 timer
		}
		break;
	case LED_BREATHE:
		// Millisecond timer interrupt will handle LED breathing
		// These if statements only initialize the breathing
		if ((led & 0x1) && (sw1_led_state != LED_BREATHE))
		{
			sw1_led_state = LED_BREATHE;
			sw1_led_intensity = 0;						// restart LED intensity
			TCC5.CCBBUF = 0;							// OC5B set to 0% duty cycle
			
		}
		if ((led & 0x2) && (sw2_led_state != LED_BREATHE))
		{
			sw2_led_state = LED_BREATHE;
			sw2_led_intensity = 0;						// restart LED intensity
			TCC5.CCABUF = 0;							// OC5A set to 0% duty cycle
		}
		if (TCC5.PERBUF != LED_PWM_PERIOD)
		{
			// This timer period is fast enough to not be visible by the naked eye
			// Thus the duty cycle produces intensity
			TCC5.PERBUF = LED_PWM_PERIOD;
			TCC5.CTRLGSET = TC_CMD_RESTART_gc;			// Restart TCC5 timer
		}
		break;
	case LED_FLASH:
		if ((led & 0x1) && (sw1_led_state != LED_FLASH))
		{
			sw1_led_state = LED_FLASH;
			TCC5.CCBBUF = LED_FLASH_PERIOD/2;			// OC5B set to 50% duty cycle
		}
		if ((led & 0x2) && (sw2_led_state != LED_FLASH))
		{
			sw2_led_state = LED_FLASH;
			TCC5.CCABUF = LED_FLASH_PERIOD/2;			// OC5A set to 50% duty cycle
		}
		if (TCC5.PERBUF != LED_FLASH_PERIOD)
		{
			// Flash timer period is actually slower than normal
			// The 50% duty cycle flash is slow enough to be visible
			TCC5.PERBUF = LED_FLASH_PERIOD;
			TCC5.CTRLGSET = TC_CMD_RESTART_gc;			// Restart TCC5 timer
		}
		break;
	default:
		// all other states are assumed to be LED_OFF
		if ((led & 0x1) && (sw1_led_state != LED_OFF))
		{
			sw1_led_state = LED_OFF;
			TCC5.CCBBUF = 0;							// OC5B set to 0% duty cycle
		}
		if ((led & 0x2) && (sw2_led_state != LED_OFF))
		{
			sw2_led_state = LED_OFF;
			TCC5.CCABUF = 0;							// OC5A set to 0% duty cycle
		}
		if (TCC5.PERBUF != LED_PWM_PERIOD)
		{
			// The LEDs are set to 0% duty cycle so the outputs are always off
			TCC5.PERBUF = LED_PWM_PERIOD;
			TCC5.CTRLGSET = TC_CMD_RESTART_gc;			// Restart TCC5 timer
		}
		break;
	}
	sei();												// Enable global interrupts
}

/* 
 * Set Horn Switch RGB LED Indicators to RGB sine pattern
 *   Running a continuously incrementing angle will cycle 
 *    through all colors.
 *   angle 0 - 255 is like hue.
 */
void inline hswl_rgb(uint8_t angle)
{
	uint8_t bigangle = (uint16_t) angle * 3 / 4;

	// produce red LED intensity
 	if (angle < 85)
 	{
		TCC4.CCDBUF = get_sine_peak(bigangle + 64);
 	}
 	else if (angle >= 170)
 	{
		TCC4.CCDBUF = get_sine_peak(bigangle - 128);		
 	}
 	else
 	{
 		TCC4.CCDBUF = 0;
 	}
 	// produce green LED intensity
 	if (angle <= 170)
 	{
		TCC4.CCCBUF = get_sine_peak(bigangle);
 	}
 	else
 	{
 		TCC4.CCCBUF = 0;
 	}
  	// produce blue LED intensity
  	if (angle < 85)
  	{
  		TCC4.CCBBUF = 0;
  	}
  	else
  	{
  		TCC4.CCBBUF = get_sine_peak(bigangle - 64);
  	}
	// Make sure the period is correct
	if (TCC4.PERBUF != LED_PWM_PERIOD)
	{
		TCC4.PERBUF = LED_PWM_PERIOD;
	}
}

/* 
 * Set Horn LED RGB Indicators to OFF
 */
void inline hswl_off(void)
{
	TCC4.CCDBUF = 0;									// Red LED off
 	TCC4.CCCBUF = 0;									// Green LED off
 	TCC4.CCBBUF = 0;									// Blue LED off
	TCC4.PERBUF = LED_PWM_PERIOD;						// Set normal PWM period
}

/* 
 * Turn output Horn on
 */
void horn_on(void)
{
	// The other two outputs (V1 and V2) cannot be on at the same time
	v12_off();											// Turn OFF all other outputs (V1 and V2)
	swl12_set(SW12_LED, LED_OFF);						// Turn OFF Switch 1 and 2 Indicator LEDs
	
	// Delay a bit before continuing
	asm("nop"); asm("nop"); asm("nop"); asm("nop"); asm("nop");
	asm("nop"); asm("nop"); asm("nop"); asm("nop"); asm("nop");
	asm("nop"); asm("nop"); asm("nop"); asm("nop"); asm("nop");
	asm("nop"); asm("nop"); asm("nop"); asm("nop"); asm("nop");
	
	HEN_port.OUTSET = (1 << HEN_bp);					// Turn Horn ON
	if (TCC4.CCDBUF != LED_FLASH_PERIOD/2) 
	{
		TCC4.CCDBUF = LED_FLASH_PERIOD/2;				// Red LED 50% duty cycle
	}
	if (TCC4.CCCBUF != 0) 
	{
 		TCC4.CCCBUF = 0;								// Green LED off
	}
	if (TCC4.CCBBUF != 0) 
	{
 		TCC4.CCBBUF = 0;								// Blue LED off
	}
	// Make sure the period is correct
	if (TCC4.PERBUF != LED_FLASH_PERIOD)
	{
		TCC4.PERBUF = LED_FLASH_PERIOD;					// Set visible flash period
	}
}

/* 
 * Turn output Horn off
 */
void inline horn_off(void)
{
	HEN_port.OUTCLR = (1 << HEN_bp);					// Turn Horn OFF
}

int main(void)
{
	uint8_t  power_state = SM_POWER_RESET;				// Current power state
	uint8_t  prog_state = SM_PROG_RESET;				// Current programming state
	uint8_t  prog_count = 0;							// Program ON time count in number of # minutes
	uint8_t  prog_led = 0;								// Program mode LED on state

	// Number of minutes to stay on when ignition is off
	uint32_t  delay_time_ms = eeprom_read_dword(&eeprom_delay_time_ms);
	
	// Disable the Watchdog timer on start
	wdt_disable();
    // main loop forever
    while (TRUE) 
    {
		// Each loop of main reset the Watchdog timer
		wdt_reset();
		
		// Power State Machine
		//   Manages initialization and power down of processor
		switch (power_state)
		{
		case SM_POWER_DOWN:								// Powered Down State
			// See if IGN has switched ON
			if (ign_cur)
			{
				// IGN is ON
				power_state = SM_POWER_ON_IGN;			// Switch to Power ON due to Ignition State
			}
			else if ((sw1_toggle || sw2_toggle) && (delay_time_ms != 0))
			{
				// IGN is OFF, one of the switches were toggled ON and delay time is set to something other than 0
				//  Time to wake up and do something
				power_state = SM_POWER_ON_SW;			// Switch to Power ON due to Switch State
				cli();									// prevent interrupts from corrupting non-atomic instructions
				delay_ms = 0;
				sei();
			}
			else
			{
				// There is nothing to do so Power Down
				horn_off();								// Turn OFF horn
				hswl_off();								// Turn OFF horn RGD LED indicators
				wdt_disable();							// Disable the watchdog timer before going to sleep
 				TCC4.CTRLA = TC_CLKSEL_OFF_gc;			// TCC4 Clock is OFF
				TCC5.CTRLA = TC_CLKSEL_OFF_gc;			// TCC5 Clock is OFF
				TCD5.CTRLA = TC_CLKSEL_OFF_gc;			// TCD5 Clock is OFF
				REV_port.INTMASK &= ~(1 << REV_bp)		// REV Input Change Interrupt interrupt disabled
								 &  ~(1 << HB_bp);		// HB Input Change Interrupt interrupt disabled
				set_sleep_mode(SLEEP_SMODE_PSAVE_gc);	// Set Power Down Mode when sleep is executed
				sleep_mode();							// Enter Power Down State now
				REV_port.INTMASK |= (1 << REV_bp) 		// REV Input Change Interrupt interrupt enabled
								 |  (1 << HB_bp);		// HB Input Change Interrupt interrupt enabled
 				TCC4.CTRLA = TC_CLKSEL_DIV64_gc;		// Clock is main/64 or 2MHz/64 or 31.25kHz
 				TCC4.CTRLC = (0 << TC4_POLA_bp)			// Non-Invert OC4A output (Not used)
 						   | (1 << TC4_POLB_bp)			// Invert OC4B output (HSWLB_EN)
 						   | (1 << TC4_POLC_bp)			// Invert OC4C output (HSWLG_EN)
 						   | (1 << TC4_POLD_bp)			// Invert OC4D output (HSWLR_EN)
 						   | (0 << TC4_CMPA_bp)			// Force OC4A output OFF (Not used)
 						   | (0 << TC4_CMPB_bp)			// Force OC4B output OFF (HSWLB_EN) 
 						   | (0 << TC4_CMPC_bp)			// Force OC4C output OFF (HSWLG_EN)
 						   | (0 << TC4_CMPD_bp);		// Force OC4D output OFF (HSWLR_EN)
				TCC5.CTRLA = TC_CLKSEL_DIV64_gc;		// Clock is main/64 or 2MHz/64 or 31.25kHz
				TCC5.CTRLC = (1 << TC5_POLA_bp)			// Invert OC5A output (SWL2_EN)
						   | (1 << TC5_POLB_bp)			// Invert OC5B output (SWL1_EN)
						   | (0 << TC5_CMPA_bp)			// Force OC5A output OFF (SWL2_EN) OFF
						   | (0 << TC5_CMPB_bp);		// Force OC5B output OFF (SWL1_EN) OFF
				TCD5.CTRLA = TC_CLKSEL_DIV64_gc;		// Clock is main/64 or 2MHz/64 or 31.25kHz
				TCD5.CTRLC = (1 << TC5_POLA_bp)			// Invert OC5A output (V1_EN)
						   | (1 << TC5_POLB_bp)			// Invert OC5D output (V2_EN)
						   | (0 << TC5_CMPA_bp)			// Force OC5A output OFF (V1_EN)
						   | (0 << TC5_CMPB_bp);		// Force OC5B output OFF (V2_EN)
				wdt_reset();							// reset watchdog before enabling it
				wdt_enable(WATCHDOG_TO);				// Enable the Watchdog timer
				continue;								// We were woken from Power Down state, restart the while(TRUE) loop
			}
			break;
		case SM_POWER_ON_IGN:							// Power ON due to Ignition State
			// See if IGN has switched OFF
			if (!ign_cur)
			{
				// IGN switched OFF so Power Down
				power_state = SM_POWER_DOWN;			// Switch to Power Down State
				sw1_toggle = TOGGLE_OFF;				// SW1 toggle forced off when IGN turns off
				sw2_toggle = TOGGLE_OFF;				// SW2 toggle forced off when IGN turns off
			}
			else
			{
				if (hsw_cur)
				{
					// Horn Switch changed
					horn_on();
					// force difference for High Beam and Reverse
					hb_last = !hb_cur;
					rev_last = !rev_last;
				}
				else
				{
					// horn is not engaged
					horn_off();
					if (hb_cur != hb_last)
					{
						// High beam changed
						if (hb_cur)
						{
							// High beam turned ON
							if (sw1_toggle != TOGGLE_ON_USER)
							{
								// SW1 Toggle is not already ON from user request
								sw1_toggle = TOGGLE_ON;	// SW1 Toggle automatically turned ON
							}
						}
						else
						{
							// High beam turned OFF
							if (sw1_toggle != TOGGLE_ON_USER)
							{
								// SW1 Toggle is not already ON from user request
								sw1_toggle = TOGGLE_OFF;// SW1 Toggle automatically turned OFF
							}
						}
						hb_last = hb_cur;
					}
					if (rev_cur != rev_last)
					{
						// Reverse changed
						if (rev_cur)
						{
							// Reverse turned ON
							if (sw2_toggle != TOGGLE_ON_USER)
							{
								// SW2 Toggle is not already ON from user request
								sw2_toggle = TOGGLE_ON;	// SW2 Toggle automatically turned ON
							}
						}
						else
						{
							// Reverse turned OFF
							if (sw2_toggle != TOGGLE_ON_USER)
							{
								// SW2 Toggle is not already ON from user request
								sw2_toggle = TOGGLE_OFF;// SW2 Toggle automatically turned OFF
							}
						}
						rev_last = rev_cur;
					}
				}
			}
			break;
		case SM_POWER_ON_SW:							// Power ON due to Switch State
			if (ign_cur)
			{
				// IGN switched on so we are no longer Power ON due to Switch
				power_state = SM_POWER_ON_IGN;			// Switch to Power ON due to Ignition turning ON
			}
			else if (!sw1_toggle && !sw2_toggle)
			{
				// No switch is active, time to Power Down
				power_state = SM_POWER_DOWN;			// Switch to Power Down State
			}
			else if (delay_ms >= delay_time_ms)
			{
				// Delay timeout
				sw1_toggle =  TOGGLE_OFF;
				sw2_toggle =  TOGGLE_OFF;				// Turn off SW1 and SW2 toggle before entering power down
				power_state = SM_POWER_DOWN;			// Switch to Power Down State
			} 
			break;
		default:
			// Anything else is considered to SM_POWER_RESET
			cli();										// Disable interrupts
			// clock setup
			OSC.CTRL = (1 << OSC_RC32KEN_bp)			// Enable internal 32.768 kHz oscillator
			         | (1 << OSC_RC2MEN_bp);			// Keep internal 2MHz oscillator enabled
			// Initialize IOs, default all pins to input and have pull-downs enabled
			PORTA.DIRCLR = 0xFF;						// PORTA is all inputs
			PORTCFG.MPCMASK = 0xFF;						// Multi-Pin Configuration select all bits
			PORTA.PIN0CTRL = PORT_OPC_PULLDOWN_gc;		// PORTA is all pull-downs
			PORTC.DIRCLR = 0xFF;						// PORTC is all inputs
			PORTCFG.MPCMASK = 0xFF;						// Multi-Pin Configuration select all bits
			PORTC.PIN0CTRL = PORT_OPC_PULLDOWN_gc;		// PORTC is all pull-downs
			PORTD.DIRCLR = 0xFF;						// PORTD is all inputs
			PORTCFG.MPCMASK = 0xFF;						// Multi-Pin Configuration select all bits
			PORTD.PIN0CTRL = PORT_OPC_PULLDOWN_gc;		// PORTD is all pull-downs
			PORTR.DIRCLR = 0xFF;						// PORTR is all inputs
			PORTCFG.MPCMASK = 0xFF;						// Multi-Pin Configuration select all bits
			PORTR.PIN0CTRL = PORT_OPC_PULLDOWN_gc;		// PORTR is all pull-downs
			// Configure IGN input
			PORTCFG.MPCMASK = (1 << IGN_bp);			// Multi-Pin Configuration select input
			IGN_port.PIN0CTRL = PORT_OPC_TOTEM_gc		// Input is not pulled down
			                  | PORT_ISC_BOTHEDGES_gc;	// Input interrupt on any edge
			IGN_port.INTCTRL = PORT_INTLVL_HI_gc;		// Interrupt will be high level
			IGN_port.INTMASK |= (1 << IGN_bp);			// Input interrupt enabled
			// Configure REV input
			PORTCFG.MPCMASK = (1 << REV_bp);			// Multi-Pin Configuration select input
			REV_port.PIN0CTRL = PORT_OPC_TOTEM_gc		// Input is not pulled down
			                  | PORT_ISC_BOTHEDGES_gc;	// Input interrupt on any edge
			REV_port.INTCTRL = PORT_INTLVL_HI_gc;		// Interrupt will be high level
			REV_port.INTMASK |= (1 << REV_bp);			// Input interrupt enabled
			// Configure HB input
			PORTCFG.MPCMASK = (1 << HB_bp);				// Multi-Pin Configuration select input
			HB_port.PIN0CTRL = PORT_OPC_TOTEM_gc		// Input is not pulled down
			                 | PORT_ISC_BOTHEDGES_gc;	// Input interrupt on any edge
			HB_port.INTCTRL = PORT_INTLVL_HI_gc;		// Interrupt will be high level
			HB_port.INTMASK |= (1 << HB_bp);			// Input interrupt enabled
			// Configure HSW input
			PORTCFG.MPCMASK = (1 << HSW_bp);			// Multi-Pin Configuration select input
			HSW_port.PIN0CTRL = PORT_OPC_TOTEM_gc		// Input is not pulled down
			                  | PORT_ISC_BOTHEDGES_gc	// Input interrupt on any edge
							  | (1 << PORT_INVEN_bp);	// Invert this input
			HSW_port.INTCTRL = PORT_INTLVL_HI_gc;		// Interrupt will be high level
			HSW_port.INTMASK |= (1 << HSW_bp);			// Input interrupt enabled
			// Configure SW1 input
			PORTCFG.MPCMASK = (1 << SW1_bp);			// Multi-Pin Configuration select input
			SW1_port.PIN0CTRL = PORT_OPC_TOTEM_gc		// Input is not pulled down
			                  | PORT_ISC_BOTHEDGES_gc	// Input interrupt on any edge
							  | (1 << PORT_INVEN_bp);	// Invert this input
			SW1_port.INTCTRL = PORT_INTLVL_HI_gc;		// Interrupt will be high level
			SW1_port.INTMASK |= (1 << SW1_bp);			// Input interrupt enabled
			// Configure SW2 input
			PORTCFG.MPCMASK = (1 << SW2_bp);			// Multi-Pin Configuration select input
			SW2_port.PIN0CTRL = PORT_OPC_TOTEM_gc		// Input is not pulled down
			                  | PORT_ISC_BOTHEDGES_gc	// Input interrupt on any edge
							  | (1 << PORT_INVEN_bp);	// Invert this input
			SW2_port.INTCTRL = PORT_INTLVL_HI_gc;		// Interrupt will be high level
			SW2_port.INTMASK |= (1 << SW2_bp);			// Input interrupt enabled
			// Configure HEN, V1EN and V2EN as totem-pole outputs
			HEN_port.OUTCLR = (1 << HEN_bp)
							| (1 << V1EN_bp)
							| (1 << V2EN_bp);			// All will be low when turned into outputs
			PORTCFG.MPCMASK = (1 << HEN_bp)
							| (1 << V1EN_bp)
							| (1 << V2EN_bp);			// Multi-Pin Configuration select all outputs
			HEN_port.PIN0CTRL = PORT_OPC_TOTEM_gc;		// All will be totem-pole outputs
			HEN_port.DIRSET = (1 << HEN_bp)
							| (1 << V1EN_bp)
							| (1 << V2EN_bp);			// All are now outputs
			// Configure HSWLREN, HSWLGEN, HSWLBEN, SWL1EN and SWL2EN as totem-pole outputs
			HSWLREN_port.OUTCLR = (1 << HSWLREN_bp)
								| (1 << HSWLGEN_bp)
								| (1 << HSWLBEN_bp)
								| (1 << SWL1EN_bp)
								| (1 << SWL2EN_bp);		// All will be not high when turned into outputs
			PORTCFG.MPCMASK = (1 << HSWLREN_bp)
							| (1 << HSWLGEN_bp)
							| (1 << HSWLBEN_bp)
							| (1 << SWL1EN_bp)
							| (1 << SWL2EN_bp);			// Multi-Pin Configuration select all outputs
			HSWLREN_port.PIN0CTRL = PORT_OPC_TOTEM_gc;	// All will be totem-pole outputs
			HSWLREN_port.DIRSET = (1 << HSWLREN_bp)
								| (1 << HSWLGEN_bp)
								| (1 << HSWLBEN_bp)
								| (1 << SWL1EN_bp)
								| (1 << SWL2EN_bp);		// All are now outputs
			// Configure TCC4 timer
			TCC4.CTRLB = TC_WGMODE_DSTOP_gc				// Dual Slope Top Update
					   | TC_CIRCEN_DISABLE_gc			// Circular Buffer disabled
					   | TC_BYTEM_NORMAL_gc;			// Normal Mode
			TCC4.CTRLC = (0 << TC4_POLA_bp)				// Non-Invert OC4A output (Not used)
			           | (1 << TC4_POLB_bp)				// Invert OC4B output (HSWLB_EN)
			           | (1 << TC4_POLC_bp)				// Invert OC4C output (HSWLG_EN)
			           | (1 << TC4_POLD_bp);			// Invert OC4D output (HSWLR_EN)
			TCC4.CTRLE = TC_CCAMODE_DISABLE_gc			// OC4A disabled  (Not used)
					   | TC_CCBMODE_COMP_gc				// OC4B enabled (HSWLB_EN)
					   | TC_CCCMODE_COMP_gc				// OC4C enabled (HSWLG_EN)
					   | TC_CCDMODE_COMP_gc;			// OC4D enabled (HSWLR_EN)
			TCC4.PERBUF = LED_PWM_PERIOD;				// FREQ = CPU_FREQ / (64 * 2 * LED_PWM_PERIOD)
			TCC4.PER = LED_PWM_PERIOD;
			TCC4.CTRLA = TC_CLKSEL_DIV64_gc;			// Clock is main/64 or 2MHz/64 or 31.25kHz
			// Configure TCC5 timer
			TCC5.CTRLB = TC_WGMODE_DSTOP_gc				// Dual Slope Top Update
					   | TC_CIRCEN_DISABLE_gc			// Circular Buffer disabled
					   | TC_BYTEM_NORMAL_gc;			// Normal Mode
			TCC5.CTRLC = (1 << TC5_POLA_bp)				// Invert OC5A output (SWL2_EN)
			           | (1 << TC5_POLB_bp);			// Invert OC5B output (SWL1_EN)
			TCC5.CTRLE = TC_CCAMODE_COMP_gc				// OC5A enabled (SWL2_EN)
					   | TC_CCBMODE_COMP_gc;			// OC5B enabled (SWL1_EN)
			TCC5.PERBUF = LED_PWM_PERIOD;				// FREQ = CPU_FREQ / (64 * 2 * LED_PWM_PERIOD)
			TCC5.PER = LED_PWM_PERIOD;
			TCC5.CTRLA = TC_CLKSEL_DIV64_gc;			// Clock is main/64 or 2MHz/64 or 31.25kHz
			// Configure TCD5 timer
			TCD5.CTRLB = TC_WGMODE_DSTOP_gc				// Dual Slope Top Update
					   | TC_CIRCEN_DISABLE_gc			// Circular Buffer disabled
					   | TC_BYTEM_NORMAL_gc;			// Normal Mode
			TCD5.CTRLC = (1 << TC5_POLA_bp)				// Invert OC5A output (V1_EN)
			           | (1 << TC5_POLB_bp);			// Invert OC5D output (V2_EN)
			TCD5.CTRLE = TC_CCAMODE_COMP_gc				// OC5A enabled (V1_EN)
					   | TC_CCBMODE_COMP_gc;			// OC5B enabled (V2_EN)
			TCD5.PERBUF = OUT_PWM_PERIOD;				// FREQ = CPU_FREQ / (64 * 2 * OUT_PWM_PERIOD)
			TCD5.PER = OUT_PWM_PERIOD;
			TCD5.CTRLA = TC_CLKSEL_DIV64_gc;			// Clock is main/64 or 2MHz/64 or 31.25kHz
			// Configure RTC Clock
			CLK.RTCCTRL = (1 << CLK_RTCEN_bp)			// enable RTC Clock
						| CLK_RTCSRC_RCOSC32_gc;		// RTC Clock is Internal 32.768 kHz Clock
			// Configure RTC
			while (RTC.STATUS & RTC_SYNCBUSY_bm);		// wait for RTC sync ready
			RTC.PER = 33;								// set RTC overflow to every 1ms (slightly more than)
			RTC.CTRL = RTC_PRESCALER_DIV1_gc			// RTC prescaler is divide 1
			         | (0 << RTC_CORREN_bp);			// RTC correction: disabled
			RTC.INTCTRL = RTC_OVFINTLVL_HI_gc			// Overflow High level interrupt priority
			            | RTC_COMPINTLVL_OFF_gc;		// Compare interrupt disabled
			// Enable high level interrupts
			PMIC.CTRL = 0 << PMIC_RREN_bp				// Round-Robin Priority Enable: disabled
			          | 0 << PMIC_IVSEL_bp				// Interrupt Vector Select: disabled
			          | 1 << PMIC_HILVLEN_bp			// High Level Enable: enabled
					  | 0 << PMIC_MEDLVLEN_bp			// Medium Level Enable: disabled
					  | 0 << PMIC_LOLVLEN_bp;			// Low Level Enable: disabled
			// Configure Power Reduction
			PR.PRGEN = 1 << PR_XCL_bp					// XCL power down: enabled
					 | 0 << PR_RTC_bp					// RTC power down: disabled
					 | 1 << PR_EVSYS_bp					// EVSYS power down: enabled
					 | 1 << PR_EDMA_bp;					// EDMA power down: enabled
			PR.PRPA = 1 << PR_DAC_bp					// DACA power down: enabled
					| 1 << PR_ADC_bp					// ADCA power down: enabled
					| 1 << PR_AC_bp;					// ACA power down: enabled
			PR.PRPC = 1 << PR_TWI_bp					// TWIC power down: enabled
					| 1 << PR_USART0_bp					// USART0C power down: enabled
					| 1 << PR_SPI_bp					// SPIC power down: enabled
					| 1 << PR_HIRES_bp					// HIRESC power down: enabled
					| 0 << PR_TC5_bp					// TCC5 power down: disabled
					| 0 << PR_TC4_bp;					// TCC4 power down: disabled
			PR.PRPD = 1 << PR_USART0_bp					// USART0D power down: enabled
					| 0 << PR_TC5_bp;					// TCD5 power down: disabled
			ign_cur = IGN_port.IN & (1 << IGN_bp);		// Get current state of Ignition Input
			rev_cur = REV_port.IN & (1 << REV_bp);		// Get current state of Reverse Input
			power_state = SM_POWER_DOWN;				// Goto to Power Down State
			wdt_enable(WATCHDOG_TO);					// Enable the Watchdog timer
			sei();										// Enable global interrupts
			continue;									// Restart the main forever loop
		}
		
		// Programming State Machine
		//   Looks for programming state based on SW1 and SW2 inputs
		switch (prog_state)
		{
		case SM_PROG_ACTIVATE:							// Program Activate State
			if (!ign_cur || !sw1_cur || !sw2_cur)
			{
				// Either Ignition, Switch 1 or Switch 2 deactivated  
				prog_state = SM_PROG_RESET;				// Goto Program Reset State
			} 
			else
			{
				// Ignition, Switch 1 and Switch 2 still active
				cli();									// prevent interrupts from corrupting non-atomic instructions
				if (prog_ms >= PROG_ACTIVATE_SECONDS * 1000)
				{
					// Ignition, Switch 1 and Switch 2 active long enough to activate programming sequence
					prog_ms = 0;						// Reset program millisecond counter
					prog_count = 0;						// Start with Outputs will NOT turn on when ignition is OFF
					swl12_set(SW12_LED, LED_FLASH);		// Flash LEDs when in programming mode
					prog_state = SM_PROG_WAIT;			// Goto Program wait for SW1 and SW2 to deactivate State
				}
				sei();
			}
			break;
		case SM_PROG_WAIT:								// Program wait for SW1 and SW2 to deactivate State
			if (!ign_cur)
			{
				// Ignition turned OFF
				prog_state = SM_PROG_RESET;				// Goto Program Reset State
				sw1_toggle = TOGGLE_OFF;				// SW1 toggle forced off when exiting from Programming Mode
				sw2_toggle = TOGGLE_OFF;				// SW2 toggle forced off when exiting from Programming Mode
			}
			else if (!sw1_cur && !sw2_cur)
			{
				// SW1 and SW2 deactivated
				prog_state = SM_PROG_ON_WAIT;			// Goto Program Wait for Switch ON
			}
			break;
		case SM_PROG_ON_WAIT:							// Program Wait for Switch ON State
			if (!ign_cur)
			{
				// Ignition turned OFF
				prog_state = SM_PROG_RESET;				// Goto Program Reset State
				sw1_toggle = TOGGLE_OFF;				// SW1 toggle forced off when exiting from Programming Mode
				sw2_toggle = TOGGLE_OFF;				// SW2 toggle forced off when exiting from Programming Mode
			} 
			else if (sw1_cur || sw2_cur)
			{
				// SW1 or SW2 turned ON
				prog_count++;							// Add one to the number of minutes to stay on when ignition is off
				prog_state = SM_PROG_OFF_WAIT;			// Goto Program Wait for Switch OFF State
				swl12_set(SW12_LED, LED_ON);			// Turn LEDs on
			}
			else if (prog_ms >= (PROG_ACTIVATE_SECONDS * 1000 / 2))
			{
				// Programming mode timeout
				//  Delay can't be longer than 20 minutes
				if (prog_count > 20)
				{
					prog_count = 20;
				}
				swl12_set(SW12_LED, LED_OFF);			// Turn LEDs off
				//  Update delay time in RAM
				delay_time_ms = prog_count * 60ul * 1000ul;
				cli();									// prevent interrupts from corrupting non-atomic instructions
				wdt_disable();							// Disable Watchdog timer before programming EEPROM
				//  Write the delay time in milliseconds to EEPROM
				eeprom_write_dword(&eeprom_delay_time_ms, delay_time_ms);
				wdt_enable(WATCHDOG_TO);
				led_ms = 0;								// reset LED millisecond counter
				sei();
				prog_led = OFF;							// start with LED off
				prog_state = SM_PROG_DISPLAY_DWELL;		// Goto Program Display New ON Time
			}
			break;
		case SM_PROG_OFF_WAIT:							// Program Wait for Switch OFF State
			if (!ign_cur)
			{
				// Ignition turned OFF or 0 delay time
				prog_state = SM_PROG_RESET;				// Goto Program Reset State
				sw1_toggle = TOGGLE_OFF;				// SW1 toggle forced off when exiting from Programming Mode
				sw2_toggle = TOGGLE_OFF;				// SW2 toggle forced off when exiting from Programming Mode
			}
			else if (!sw1_cur && !sw2_cur)
			{
				// SW1 and SW2 are both OFF
				prog_state = SM_PROG_ON_WAIT;			// Goto Program Wait for Switch ON State
				swl12_set(SW12_LED, LED_FLASH);			// back to Flash LEDs
				cli();									// prevent interrupts from corrupting non-atomic instructions
				prog_ms = 0;							// reset program millisecond counter
				sei();
			}
			break;
		case SM_PROG_DISPLAY_DWELL:						// Display new Delay On Time, Initial Dwell State
			if (!ign_cur || delay_time_ms == 0)
			{
				// Ignition turned OFF
				prog_state = SM_PROG_RESET;				// Goto Program Reset State
				sw1_toggle = TOGGLE_OFF;				// SW1 toggle forced off when exiting from Programming Mode
				sw2_toggle = TOGGLE_OFF;				// SW2 toggle forced off when exiting from Programming Mode
			}
			else if (led_ms >= 1000)
			{
				// Initial dwell with LEDS off has expired
				prog_state = SM_PROG_DISPLAY;			// Goto Display new Delay On Time, LED Toggle State
				cli();									// prevent interrupts from corrupting non-atomic instructions
				led_ms = 0;								// reset LED millisecond counter
				sei();
			}
			break;
		case SM_PROG_DISPLAY:							// Display new Delay On Time, LED Off State
			if (!ign_cur)
			{
				// Ignition turned OFF
				prog_state = SM_PROG_RESET;				// Goto Program Reset State
				sw1_toggle = TOGGLE_OFF;				// SW1 toggle forced off when exiting from Programming Mode
				sw2_toggle = TOGGLE_OFF;				// SW2 toggle forced off when exiting from Programming Mode
			}
			else if (led_ms >= LED_FLASH_TIME)
			{
				cli();									// prevent interrupts from corrupting non-atomic instructions
				led_ms = 0;								// reset LED millisecond counter
				sei();
				if (prog_led)
				{
					// LED is turning off which completes a single count display
					if (--prog_count == 0)
					{
						// no more flashes needed
						prog_state = SM_PROG_RESET;		// Goto Program Reset State
						sw1_toggle = TOGGLE_OFF;		// SW1 toggle forced off when exiting from Programming Mode
						sw2_toggle = TOGGLE_OFF;		// SW2 toggle forced off when exiting from Programming Mode
					}
				}
				prog_led = !prog_led;					// toggle LED state
			}
			prog_led ? swl12_set(SW12_LED, LED_ON) : swl12_set(SW12_LED, LED_OFF);	// Set LEDs to correct state
			break;
		default:										// Program Reset State
			if (ign_cur && sw1_cur && sw2_cur)
			{
				// Ignition, Switch 1 and Switch 2 are simultaneously active
				cli();									// prevent interrupts from corrupting non-atomic instructions
				prog_ms = 0;							// reset program millisecond counter
				sei();								
				sw1_toggle = TOGGLE_OFF;				// SW1 toggle forced off when entering Programming Mode
				sw2_toggle = TOGGLE_OFF;				// SW2 toggle forced off when entering Programming Mode
				swl12_set(SW12_LED, LED_OFF);			// Turn off Switch 1 and 2 Indicator LEDs
				v12_off();								// Outputs turned off when in programming mode
				prog_state = SM_PROG_ACTIVATE;			// Goto Program Activate State
			}
			else
			{
				// LEDs and Outputs operate normally when not in programming mode and horn not one
				if (!hsw_cur)
				{
					if (sw1_toggle)
					{
						if (sw1_toggle == TOGGLE_ON_USER)
						{
							// User requested Toggle ON
							swl12_set(SW1_LED, LED_ON);
						}
						else
						{
							// Automatic requested Toggle ON
							swl12_set(SW1_LED, LED_BREATHE);
						}
						v1_on();
					}
					else
					{
						swl12_set(SW1_LED, LED_OFF);
						v1_off();
					}
					if (sw2_toggle)
					{
						if (sw2_toggle == TOGGLE_ON_USER)
						{
							// User requested Toggle ON
							swl12_set(SW2_LED, LED_ON);
						}
						else
						{
							// Automatic requested Toggle ON
							swl12_set(SW2_LED, LED_BREATHE);
						}
						v2_on();
					}
					else
					{
						swl12_set(SW2_LED, LED_OFF);
						v2_off();
					}
				}			
			}
			break;
		}
    }
}

/* 
 * RTC Overflow interrupt.
 *  RTC is configured to overflow every millisecond.
 *  Update delay counters and tick count
 *  Use tick count to handle debouncing inputs
 */
ISR(RTC_OVF_vect)
{
	static uint8_t rainbow_ms = 0;						// number of milliseconds before incrementing rainbow_cnt
	static uint8_t rainbow_cnt = 85;					// used to count through sine wave for Horn RGM LED rainbow
	static uint8_t breathe_ms = 0;						// number of milliseconds before incrementing breathe_cnt
	static uint8_t breathe_cnt = 0;						// used to count through sine wave for LED breathing
	
	tick_ms++;											// increment tick millisecond counter
	delay_ms++;											// Increment delay milliseconds
	prog_ms++;											// increment program milliseconds counter
	led_ms++;											// increment LED milliseconds counter
	
	// Handle Horn Switch RGB LED Indicator rainbow
	if (ign_cur)
	{
		if (hsw_cur)
		{
			// Horn is currently on
			rainbow_ms = 64;
			rainbow_cnt = 0;
		}
		else if (rainbow_ms++ >= 64)
		{
			rainbow_ms = 0;
 			hswl_rgb(rainbow_cnt++);
		}
	}
	else
	{
		rainbow_ms = 0;
		rainbow_cnt = 85;
	}
	// Handle Switch 1 and LED Indicator breathing
	if (breathe_ms++ >= 8)
	{
		breathe_ms = 0;
		if ((sw1_led_state != LED_BREATHE) && (sw2_led_state != LED_BREATHE))
		{
			breathe_cnt = 64;							// No LEDs are breathing
														// set to peak sine value
		}
		else
		{
			breathe_cnt++;								// At least one LED is breathing
			if (sw1_led_state == LED_BREATHE)
			{
				TCC5.CCBBUF = get_sine(breathe_cnt);	// OC5B set to next duty cycle
			}
			if (sw2_led_state == LED_BREATHE)
			{
				TCC5.CCABUF = get_sine(breathe_cnt);	// OC5A set to next duty cycle
			}
		}
	}
	
	// Handle all input debouncing
	if (hsw_db)
	{
		// Input is in the process of debouncing
		if (tick_ms >= hsw_dbtime)
		{
			// It has been long enough since last input change
			hsw_db = FALSE;								// We are no longer debouncing
			hsw_cur = HSW_port.IN & (1 << HSW_bp);		// Set to current state of input
		}
	}
	if (sw1_db)
	{
		// Input is in the process of debouncing
		if (tick_ms >= sw1_dbtime)
		{
			// It has been long enough since last input change
			sw1_db = FALSE;								// We are no longer debouncing
			sw1_cur = SW1_port.IN & (1 << SW1_bp);		// Set to current state of input
		}
	}
	if (sw2_db)
	{
		// Input is in the process of debouncing
		if (tick_ms >= sw2_dbtime)
		{
			// It has been long enough since last input change
			sw2_db = FALSE;								// We are no longer debouncing
			sw2_cur = SW2_port.IN & (1 << SW2_bp);		// Set to current state of input
		}
	}
	if (ign_db)
	{
		// Input is in the process of debouncing
		if (tick_ms >= ign_dbtime)
		{
			// It has been long enough since last input change
			ign_db = FALSE;								// We are no longer debouncing
			ign_cur = IGN_port.IN & (1 << IGN_bp);		// Set to current state of input
		}
	}
	if (rev_db)
	{
		// Input is in the process of debouncing
		if (tick_ms >= rev_dbtime)
		{
			// It has been long enough since last input change
			rev_db = FALSE;								// We are no longer debouncing
			rev_cur = REV_port.IN & (1 << REV_bp);		// Set to current state of input
		}
	}
	if (hb_db)
	{
		// Input is in the process of debouncing
		if (tick_ms >= hb_dbtime)
		{
			// It has been long enough since last input change
			hb_db = FALSE;								// We are no longer debouncing
			hb_cur = HB_port.IN & (1 << HB_bp);			// Set to current state of input
		}
	}
}

/*
 * PORTA Interrupt. (HSW, SW2, IGN, HB and REV Input Sense Interrupt)
 *  Used to detect these inputs changing, any edge will cause this interrupt.
 */
ISR(PORTA_INT_vect)
{
	if (PORTA.INTFLAGS & (1 << HSW_bp))
	{
		// HSW input has changed
		PORTA.INTFLAGS |= (1 << HSW_bp);				// Clear the interrupt flag
		hsw_db = TRUE;									// We are currently debouncing
		hsw_dbtime = tick_ms + DEBOUNCE_TIME;			// Reset debounce timer
		if (!hsw_cur)
		{
			// This is the input turning on (rising edge)
			hsw_cur = ON;								// Input is now ON
		}
	}
	if (PORTA.INTFLAGS & (1 << SW2_bp))
	{
		// SW2 input has changed
		PORTA.INTFLAGS |= (1 << SW2_bp);				// Clear the interrupt flag
		sw2_db = TRUE;									// We are currently debouncing
		sw2_dbtime = tick_ms + DEBOUNCE_TIME;			// Reset debounce timer
		if (!sw2_cur)
		{
			// This is the input turning on (rising edge)
			sw2_cur = ON;								// Input is now ON
			// This is a user requested toggle
			if (sw2_toggle == TOGGLE_OFF) 
			{
				sw2_toggle = TOGGLE_ON_USER;			// User turned ON
			}
			else
			{
				sw2_toggle = TOGGLE_OFF;				// User turned OFF
			}
		}
	}
	if (PORTA.INTFLAGS & (1 << IGN_bp))
	{
		// IGN input has changed
		PORTA.INTFLAGS |= (1 << IGN_bp);				// Clear the interrupt flag
		ign_db = TRUE;									// We are currently debouncing
		ign_dbtime = tick_ms + DEBOUNCE_TIME;			// Reset debounce timer
		if (!ign_cur)
		{
			// This is the input turning on (rising edge)
			ign_cur = ON;								// Input is now ON
		}
	}
	if (PORTA.INTFLAGS & (1 << HB_bp))
	{
		// HB input has changed
		PORTA.INTFLAGS |= (1 << HB_bp);					// Clear the interrupt flag
		hb_db = TRUE;									// We are currently debouncing
		hb_dbtime = tick_ms + DEBOUNCE_TIME;			// Reset debounce timer
		if (!hb_cur)
		{
			// This is the input turning on (rising edge)
			hb_cur = ON;								// Input is now ON
		}
	}
	if (PORTA.INTFLAGS & (1 << REV_bp))
	{
		// REV input has changed
		PORTA.INTFLAGS |= (1 << REV_bp);				// Clear the interrupt flag
		rev_db = TRUE;									// We are currently debouncing
		rev_dbtime = tick_ms + DEBOUNCE_TIME;			// Reset debounce timer
		if (!rev_cur)
		{
			// This is the input turning on (rising edge)
			rev_cur = ON;								// Input is now ON
		}
	}
}


/*
 * PORTC Interrupt. (SW1 Input Sense Interrupt)
 *  Used to detect this input changing, any edge will cause this interrupt.
 */
ISR(PORTC_INT_vect)
{
	if (PORTC.INTFLAGS & (1 << SW1_bp))
	{
		// REV input has changed
		PORTC.INTFLAGS |= (1 << SW1_bp);				// Clear the interrupt flag
		sw1_db = TRUE;									// We are currently debouncing
		sw1_dbtime = tick_ms + DEBOUNCE_TIME;			// Reset debounce timer
		if (!sw1_cur)
		{
			// This is the input turning on (rising edge)
			sw1_cur = ON;								// Input is now ON
			// This is a user requested toggle
			if (sw1_toggle == TOGGLE_OFF)
			{
				sw1_toggle = TOGGLE_ON_USER;			// User turned ON
			}
			else
			{
				sw1_toggle = TOGGLE_OFF;				// User turned OFF
			}
		}
	}
}