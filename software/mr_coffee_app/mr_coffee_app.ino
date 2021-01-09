/* What is this??
 *
 * Mr Coffee is a ...
 * Matrix 24*5 led display
 * For both Arduino UNO/Pro-trinket and Sparkfun ESP8266 thing dev.
 *
 * Software overview :
 * ===================
 * Basically a Foreground/background architecture.
 *
 * main loop( )
 * -A timer triggers a ISR on overflow to trigger ADC read. see ( ISR_FREQ_HZ )
 *
 * -If the input signal has a DC offset e.g. ADC/2 then remove the offset.
 *
 * -Autogain: This is to help utilise the full display output from whatever
 * input. e.g. if the ADC signal only ever goes from 0-512, and the full scale
 * range is from 0-1023 then we only ever use half the LED's, we don't want
 * this.
 * This value is mapped to a value range of 0-11, which is one half of the
 * Matrix LED array.
 *
 * -Buttons: One for LED brightness, one for Face type.
 *
 * -Send data to the Matrix LED display.
 *
 * -Every second the status/Live signal is flashed, if YES_TO_LIFE_LED is set.
 *
 * Other function to create a sine table during setup( ). see, make_sine_table( ).
 *
 *
 * Configuration:
 * ==============
 * see, coffee_defines.h
 *
 *
 * Debugging this:
 * ===============
 * Plenty of Serial debug for various parts of the code/hw
 *
*/

/*
ToDo:
-AnalogRead( ) returns an int - i am using a uint16_t also lDCOffset ighRead are
uint's tutu tut
-remove button_faceState, button_brightnessState.
-rename ISR_FREQ_HZ
-rename TIMER1_COUNTER
-camel case not snake case
-structs for status led?
-print_autogain_display( ) hard-coded could be better...

Magic numbers:
==============
-static uint16_t highest_input_read  = 250;          // an arbitrary value
-const uint8_t max_map_val = 11;    //MIDDLE_COLUMN_VAL-1 or something
-mouth_data_t init's
-make_sine_table( )


functionality:
==============
-esp8266 - wifi, buttons, slider, etc etc.
-ability to process a adc input signal that a DC offset of ADC/2. --check working
-check autogain MINMUM_MIC_RANGE etc - how is this determined? maybe better,
a dynamic variable would be more fun. e.g. knobs or sliders. or just leave at '11'.
-generate gauss table or others ...
-implement the song recognition thing, Nian cat mode
-pong mode...
-Pacman mode
-my own smoothing filters ( moving average, what else? )
-ability to dynamically move the centre point of the matrix,
-if i had a accelerometer that could detect head tilt...
-LDR to effect the pwm brightness?
-music/speak

Documentation:
==============
schematics etc etc etc

*/

/*******************************************************************************
 * Includes
 *
*******************************************************************************/
//#include "LRAS1130-master/LRAS1130.h"
#include <LRAS1130.h>
#include <LRAS1130Picture12x11.h>
#include <LRAS1130Picture24x5.h>
#include "coffee_define.h"
#if defined ESP8266
#   include <ESP8266WiFi.h>
/* This define must be placed at the beginning before #include "ESP8266TimerInterrupt.h" */
#   define TIMER_INTERRUPT_DEBUG        1
#   include "ESP8266TimerInterrupt.h"
/* Init ESP8266 timer 0 */
ESP8266Timer ITimer;
#endif /* defined ESP8266 */

/*******************************************************************************
 * Prototypes
 *
*******************************************************************************/
//reorganise the parameter LHS/clear first
void set_matrix_leds( uint8_t pwmset, uint8_t frame, uint8_t row_val, uint8_t clear, mouth_data_t *mouthPiece, uint8_t lhs_or_rhs );
void set_matrix_leds_bender( uint8_t pwmset, uint8_t frame, uint8_t row_val, uint8_t clear, mouth_data_t *mouthPiece );

void print_autogain_display( int16_t adcV, uint16_t lDCOff, uint16_t highRead );
void make_sine_table( mouth_data_t *mouthD, uint8_t brightness );

void ERROR_HOOK( void );

#if defined __AVR__
void ISR_button_ledBrightness( );
void ISR_button_FaceType( );
#endif /* __AVR__ */

/*******************************************************************************
 * Globals
 *
*******************************************************************************/
using namespace lr;
AS1130 ledDriver;

    /***************
     * Matrix
    ***************/
const uint8_t max_map_val = 11;    //MIDDLE_COLUMN_VAL-1 or something

const uint8_t AllOnFrame[ ] =
{
    0b11111111, 0b11111111, 0b11111111,
    0b11111111, 0b11111111, 0b11111111,
    0b11111111, 0b11111111, 0b11111111,
    0b11111111, 0b11111111, 0b11111111,
    0b11111111, 0b11111111, 0b11111111
};

// writes "Mr" on the mouth piece when talking
const uint8_t MRFrame[ ] =
{
    0b00000000, 0b00000000, 0b00000000,
    0b00000000, 0b10100000, 0b00000000,
    0b00000000, 0b11101110, 0b00000000,
    0b00000000, 0b10101000, 0b00000000,
    0b00000000, 0b10101000, 0b00000000
};

// writes "coffee" on the mouth piece when talking
const uint8_t COFFEEFrame[ ] =
{
    0b11101110, 0b11101110, 0b11101110,
    0b10001010, 0b10001000, 0b10001000,
    0b10001010, 0b11101110, 0b11101110,
    0b10001010, 0b10001000, 0b10001000,
    0b11101110, 0b10001000, 0b11101110
};

uint8_t background_array[ MATRIX_LONG_24 ] = { BACKGROUND_PWM_VAL };

    /***************
     * Matrix Gauss
    ***************/
uint8_t gauss_dist_array[ ] =  { 0,0,0,0,0,0,0,0,0,3,34,154,255,154,34,3,0,0,0,0,0,0,0,0,0 };     // .trail_width = 3
//uint8_t gauss_dist_array[ ] = { 0,0,0,1,2,4,7,11,15,20,24,27,28,27,24,20,15,11,7,4,2,1,0,0,0 }; // .trail_width = 9


static mouth_data_t mouth_gauss =
{
    .trail_start_point  = 12,// peak brightness
    .trail_width        = 3, // width of bars excluding the peak value
    .array_length       = MATRIX_LONG_24,    // loop length was smaller...!
    .trail_clear        = &background_array[ 0 ],
    .trail_data         = &gauss_dist_array[ 0 ],
};

    /***************
     * Matrix SINE
    ***************/
static uint8_t sine_table[ MATRIX_LONG_24 ] = { 0 };

static mouth_data_t mouth_sine =
{
    .trail_start_point  = 12,                   //where in array to start looping from, was 0
    .trail_width        = 6,                    // width of bars excluding the peak value
    .array_length       = MATRIX_LONG_24,       // loop length was smaller...!
    .trail_clear        = &background_array[ 0 ],
    .trail_data         = &sine_table[ 0 ] ,
};

static uint8_t all_led_on_full[ MATRIX_LONG_24 ] = { MAX_PWM_BRIGHTNESS };
static mouth_data_t *mouth = &mouth_gauss;

/* Used by ISR's
 * Variables used both inside and outside an ISR should be volatile.
*/
static volatile bool     sampleNProcess = FINISHED_SAMPLE_AND_PROCESS;  // either FINISHED_SAMPLE_AND_PROCESS or GO_SAMPLE_AND_PROCESS
// buttons
static volatile uint8_t  button_LedBrightnessCount = 0;
static volatile uint8_t  button_FaceType = 0;
// status led flashes every second
static volatile uint8_t  statusLedCnt = 0;
//error handling if interrupt, interrupts the process function
static volatile uint8_t  checkDataCoherancy = 0;
static volatile bool     callERROR_HOOK = 0;

/*******************************************************************************
 * ISR's
 *
*******************************************************************************/
#if defined __AVR__
ISR( TIMER1_OVF_vect )
#elif defined ESP8266
void ICACHE_RAM_ATTR TimerHandler( void )
#endif /* __AVR__ || ESP8266 */
{
    /* When timers overflow, Acquire data */
    if( sampleNProcess == FINISHED_SAMPLE_AND_PROCESS )
    {
        sampleNProcess = GO_SAMPLE_AND_PROCESS;
    }
    else
    {
        //bounds / error check ?
    }

    ++statusLedCnt;

    if( checkDataCoherancy == 1 )
    {
        /* checkDataCoherancy is set to '1' at start of process loop, and then '0'd at the end of process loop */
        callERROR_HOOK = 1;
    }
#if defined __AVR__
    TCNT1 = TIMER1_COUNTER;   // reload timer
#endif /* __AVR__ */
}

/* isr for button press, changes brightness of leds */
void ISR_button_ledBrightness( )
{
    ++button_LedBrightnessCount;
    button_LedBrightnessCount %= BUTTON_LED_CURRENT_MODES;
}

/* isr for button press - changes the face type */
void ISR_button_FaceType( )
{
    ++button_FaceType;
    button_FaceType %= BUTTON_FACE_TYPE_MODES;
}



/******************************************************************************/
/******************************************************************************/
/******************************************************************************/
/******************************************************************************/
/*******************************************************************************
 * Setup
 *
*******************************************************************************/
void setup( )
{
    pinMode( LIFE_LED, OUTPUT );    /* Status LED will flash every second. */

#if defined DO_SERIAL_DEBUG      || defined DO_PRINT_ADCVALUE         || defined DO_MATRIX_DEBUG        \
 || defined DO_PRINT_MATRIXVALUE || defined DO_PRINT_AUTOGAIN_DISPLAY || defined DO_PRINT_AUTOGAIN_VALS
    Serial.begin( SERIAL_PORT_BAUD );
    SERIALPRINTLN( "Serial port open..." );
#endif /* enable some mind of serial debugging. */

    make_sine_table( &mouth_sine, MAX_PWM_BRIGHTNESS );

#if ! defined DISABLE_MATRIX_OVER_I2C_COMMS
    Wire.begin( );
    Wire.setClock( I2C_SPEED );

    // Wait until the chip is ready.
    delay( 500 );

    SERIALPRINTLN( F("Reset chip") );

    // hard reset the matrix
    digitalWrite( MATRIX_RESETPIN, LOW );
    delay( 500 );
    digitalWrite( MATRIX_RESETPIN, HIGH );

    SERIALPRINTLN( F("Initialize chip") );

    // Check if the chip is addressable.
    if( ! ledDriver.isChipConnected( ) )
    {
        SERIALPRINTLN( F("Communication problem with chip.") );
        //Serial.flush();
        return;
    }

    /*  Set-up everything.
    *  Ram config 1, with 36 frames and 1 pwm set, we require 1 frame (all leds on) and 1 pwm set ( which we alter)
    */
    ledDriver.setRamConfiguration( AS1130::RamConfiguration3 ); //  ramconf 1 = 36 frames and 1 pwmset  // ram cofig 5 = 5 pwm sets and 12 frames

    ledDriver.setOnOffFrame24x5( 0, MRFrame,     0 ); // writes the mr
    ledDriver.setOnOffFrame24x5( 1, COFFEEFrame, 0 ); // writes the coffee on the mouth piece
    ledDriver.setOnOffFrame24x5( 2, AllOnFrame,  1 ); // void setOnOffFrame24x5(uint8_t frameIndex, const uint8_t *data, uint8_t pwmSetIndex = 0);


    // Set-up a blink&PWM set with values for all LEDs.
    ledDriver.setBlinkAndPwmSetAll( 0, false, MAX_PWM_BRIGHTNESS ); // void setBlinkAndPwmSetAll(uint8_t setIndex, bool doesBlink = false, uint8_t pwmValue = 0xff);
    ledDriver.setBlinkAndPwmSetAll( 1, false, BACKGROUND_PWM_VAL ); // void setBlinkAndPwmSetAll(uint8_t setIndex, bool doesBlink = false, uint8_t pwmValue = 0xff);

    ledDriver.setCurrentSource( AS1130::Current5mA );
    ledDriver.setScanLimit( AS1130::ScanLimitFull );
    ledDriver.startPicture( 0, false ); // startPicture(uint8_t frameIndex, bool blinkAll = false);

    // Enable the chip
    ledDriver.startChip( );
#endif /* defined ! DISABLE_MATRIX_OVER_I2C_COMMS */

    // initialize timer
    noInterrupts( );

#if defined __AVR__
    /* defines timer and 2* button ISR's */
    TCCR1A  = 0;
    TCCR1B  = 0;
    TCNT1   = TIMER1_COUNTER;   // preload timer
    TCCR1B |= ( 1 << CS12 );    // 256 prescaler
    TIMSK1 |= ( 1 << TOIE1 );   // enable timer overflow interrupt
    // button interrupts
    pinMode( BUTTON_PIN_LED_BRIGHTNESS, INPUT_PULLUP );
    attachInterrupt( digitalPinToInterrupt( BUTTON_PIN_LED_BRIGHTNESS ), ISR_button_ledBrightness, FALLING );
    pinMode( BUTTON_PIN_FACE, INPUT_PULLUP );
    attachInterrupt( digitalPinToInterrupt( BUTTON_PIN_FACE ), ISR_button_FaceType, FALLING );
#elif defined ESP8266
    if( ITimer.attachInterruptInterval( TIMER_INTERVAL_MS * 1000, TimerHandler ) )
    {
        SERIALPRINTLN( "Starting  ITimer OK, millis( ) = " + String( millis( ) ) ); // always gives wrong answer...
    }
    else
    {
        SERIALPRINTLN( "Can't set ITimer correctly. Select another freq. or interval" );
    }
    /* esp8266 doe not have hw buttons .. use wifi! :)*/
#endif /* __AVR__ || ESP8266 */

    SERIALPRINTLN( "Enabling Interrupts and starting loop( ).");

    interrupts( );             // enable all interrupts
}








/******************************************************************************/
/******************************************************************************/
/******************************************************************************/
/******************************************************************************/
void loop( )
{
    /* These have to be static or get reset every loop?? something like that. */
    static uint16_t adc_val;
    static uint16_t new_matrix_peak_val = 0;
    static uint16_t old_matrix_peak_val = 1;
    static int16_t  adcVals = { 0 };           // i think this was an array for a moving average filter, could be refactor'd
    static bool     statusLedState = 0;
    // auto gain variables
    static uint16_t lowest_dcoffset     = ADC_FULL_SCALE_RANGE;
    static uint16_t highest_input_read  = 0;          // an arbitrary value -was '250', now '0', to ensure full range.
    uint8_t  frame  = 0;
    uint8_t  pwmset = 0;
    uint8_t  button_faceState = 0;      // can i not just remove this? todo
#ifdef __AVR__
    uint8_t button_brightnessState = 0; //divergent code from ESP8266 because of the ISR_button_xxx( )
#elif ESP8266
    uint8_t button_brightnessState = AS1130::Current5mA; /* ESP8266 compiler complained about this */
#endif /* defined __AVR__, ESP8266 */

    /* ADC sample then process data. */
    if( sampleNProcess == GO_SAMPLE_AND_PROCESS )
    {
        ++checkDataCoherancy;

#if defined SIGNAL_INPUT_IS_SAWTOOTH
        adc_val = ( ++adc_val & ADC_FULL_SCALE_RANGE );
#else
        adc_val = analogRead( ADC_PIN );
#endif /* SIGNAL_INPUT_IS_SAWTOOTH */

        PRINT_ADCVALUE( adc_val );

        /* AutoGain Section */
        /* better description here... */
#if SIGNAL_HAS_DCOFFSET
        /*
        * If the input of the signal has a dc offset? eg ADC/2.
        * in that case then we right shift by '1'
        */
        adc_val >>= DCOFFSET_VALUE;
#endif /* SIGNAL_HAS_DCOFFSET */

        /* not really happy with variable names  lowest_dcoffset, highest_input_read */
        // :recalibrate range of data, to max the range because plosives make a mess of the range
        // ignore low end noise
        // ignore top end space

        lowest_dcoffset += INCREASE_MIN_RANGE_VALUE; //always knock minimum value up

        if( lowest_dcoffset > adc_val )
        {
            lowest_dcoffset = adc_val; // recalibrate min -knock back down
        }

        if( highest_input_read > ( lowest_dcoffset + MINMUM_MIC_RANGE ) ) // range minimum is 200
        {
            highest_input_read -= DECREASE_MAX_RANGE_VALUE; // always knock max value down if above minimum range
        }

        if( highest_input_read < adc_val )
        {
            highest_input_read = adc_val; // recalibrate max  -knock back up
        }

        if( lowest_dcoffset == highest_input_read )
        {
            //exception in map( ) if (lowest_dcoffset == highest_input_read)
            highest_input_read += max_map_val;   //
        }

        PRINT_AUTOGAIN( adc_val, lowest_dcoffset, highest_input_read );

        // print graphics to UART
        print_autogain_display( adc_val, lowest_dcoffset, highest_input_read );

        // capture button state, don't turn off interrupts, because of i2c
        button_faceState = button_FaceType; //Todo remove this unnecessary step

        // map to range
        // https://www.arduino.cc/reference/en/language/functions/math/map/
        // map(value, fromLow, fromHigh, toLow, toHigh) e.g. val = map(val, 0, 1023, 0, 255);
        if( button_faceState != benderMouth )
        {
            // not bender mouth,
            // maps: full ADC read range to matrix's 0-11 columns, which is half a matrix's size
            adcVals = map( adc_val, lowest_dcoffset, highest_input_read, max_map_val, 0 );
        }
        else
        {
            //bender mouth
            adcVals = map( adc_val, lowest_dcoffset, highest_input_read, 0, 2 );
        }

//All this shold be removed it is not required just use button_LedBrightnessCount
//Todo remove this unnecessary extra step
        // i jusr want ledDriver.setCurrentSource( button_LedBrightnessCount );
#if defined __AVR__
        // capture button state,
        button_brightnessState = button_LedBrightnessCount;

        // set brightness from brightness button
        ledDriver.setCurrentSource( button_brightnessState );
#elif defined ESP8266
#   warning no buttons yet ...
        ledDriver.setCurrentSource( AS1130::Current5mA );
#endif /*  */


        // no lowpass
        new_matrix_peak_val = adcVals;

        PRINT_MATRIXVALUE( new_matrix_peak_val );

        /*  */
        //if(new_matrix_peak_val != old_matrix_peak_val )
        {
          /* Send values to the Matrix
            depends on button_faceState (1)
            0) Serial.println("MR COFFEE bold flash");
            1) Serial.println("MR COFFEE strobed");
            2) Serial.println("gauss sweeper");
            3) Serial.println("bars");
            4) Serial.println("bender");
            5)
          */

            switch( button_faceState )
            {
                case mrCoffeeBoldFlashMouth:
                    PRINT_FACE_TYPE( "MR COFFEE bold flash" );
                    if( old_matrix_peak_val >= 11 ) // new_matrix_peak_val
                    {
                        //show MR
                        frame  = 0;            // 'mr' frame
                        pwmset = 0;            // bold pwm set
                        mouth  = &mouth_sine;  // update mouth type
                        ledDriver.setBlinkAndPwmSetAll( pwmset, false, MAX_PWM_BRIGHTNESS ); //all pwm values to max // void setBlinkAndPwmSetAll(uint8_t setIndex, bool doesBlink = false, uint8_t pwmValue = 0xff);

                        ledDriver.startPicture( frame, false );               // draw picture
                    }
                    else
                    {
                        //show coffee
                        frame  = 1;            // 'coffee' frame
                        pwmset = 0;           // bold pwm set
                        mouth  = &mouth_sine;  // update mouth type
                        ledDriver.setOnOffFrame24x5( frame, COFFEEFrame, pwmset );  // update frame with pwm set
                        ledDriver.setBlinkAndPwmSetAll( pwmset, false, MAX_PWM_BRIGHTNESS ); //all pwm values to max

                        ledDriver.startPicture( frame, false );
                  }
                  break;

                case 1:
                    PRINT_FACE_TYPE( "MR COFFEE wavy" );
                    if( old_matrix_peak_val >= 11 ) // new_matrix_peak_val
                    {
                        //show MR
                        frame  = 0;            // 'mr' frame
                        pwmset = 0;            // bold pwm set
                        mouth  = &mouth_sine;  // update mouth type

                        ledDriver.startPicture( frame, false );
                    }
                    else
                    {
                        //show coffee
                        frame  = 1;            // 'coffee' frame
                        pwmset = 1;            // wavy pwmset
                        mouth  = &mouth_gauss; // update mouth type
                        ledDriver.setOnOffFrame24x5( frame, COFFEEFrame, pwmset );  // update frame with pwm set

                        set_matrix_leds( pwmset, frame, old_matrix_peak_val,                CLEAR_ARRAY, mouth, LHS );
                        set_matrix_leds( pwmset, frame, new_matrix_peak_val,                WRITE_ARRAY, mouth, LHS );
                        set_matrix_leds( pwmset, frame, MATRIX_LONG_24-old_matrix_peak_val, CLEAR_ARRAY, mouth, RHS );
                        set_matrix_leds( pwmset, frame, MATRIX_LONG_24-new_matrix_peak_val, WRITE_ARRAY, mouth, RHS );
                    }
                    break;

                case 2:
                /* gauss inwards to centre sweep */
                    PRINT_FACE_TYPE( "gauss sweeper" );
                    frame  = 2;              // 'all on' frame
                    pwmset = 1;              // wavy pwmset
                    mouth  = &mouth_gauss;   // update mouth type
                    ledDriver.setOnOffFrame24x5( frame, AllOnFrame, pwmset ); // update frame with pwm set // void setOnOffFrame24x5(uint8_t frameIndex, const uint8_t *data, uint8_t pwmSetIndex = 0);

                    static uint8_t cnt = 0;
                    ++cnt;
                    cnt %= 12;

                    static uint8_t offset = 12;
                    offset += cnt;
                    offset %= MATRIX_LONG_24;

                    set_matrix_leds( pwmset, frame, cnt-1,                CLEAR_ARRAY, mouth, LHS );
                    set_matrix_leds( pwmset, frame, cnt,                  WRITE_ARRAY, mouth, LHS );
                    set_matrix_leds( pwmset, frame, MATRIX_LONG_24-cnt-1, CLEAR_ARRAY, mouth, RHS );
                    set_matrix_leds( pwmset, frame, MATRIX_LONG_24-cnt,   WRITE_ARRAY, mouth, RHS );
                    break;

                case 3:
                /* knight rider */
                    PRINT_FACE_TYPE( "bars" );
                    frame  = 2;              // 'all on' frame
                    pwmset = 1;             // wavy pwmset
                    mouth  = &mouth_gauss;   // update mouth type
                    ledDriver.setOnOffFrame24x5( frame, AllOnFrame, pwmset ); // update frame with pwm set // void setOnOffFrame24x5(uint8_t frameIndex, const uint8_t *data, uint8_t pwmSetIndex = 0);

                    set_matrix_leds( pwmset, frame, old_matrix_peak_val,                CLEAR_ARRAY, mouth, LHS );
                    set_matrix_leds( pwmset, frame, new_matrix_peak_val,                WRITE_ARRAY, mouth, LHS );
                    set_matrix_leds( pwmset, frame, MATRIX_LONG_24-old_matrix_peak_val, CLEAR_ARRAY, mouth, RHS );
                    set_matrix_leds( pwmset, frame, MATRIX_LONG_24-new_matrix_peak_val, WRITE_ARRAY, mouth, RHS );
                    break;
                case 4:
                /* Bender */
                    PRINT_FACE_TYPE( "Bender" );
                    frame  = 2;              // 'all on' frame
                    pwmset = 1;             // wavy pwmset
                    mouth  = &mouth_gauss;   // update mouth type
                    ledDriver.setOnOffFrame24x5( frame, AllOnFrame, pwmset ); // update frame with pwm set // void setOnOffFrame24x5( uint8_t frameIndex, const uint8_t *data, uint8_t pwmSetIndex = 0 );
                    //ledDriver.setBlinkAndPwmSetAll( pwmset, false, BACKGROUND_PWM_VAL );//reset all leds to off no too flashy on display bad effect
                    set_matrix_leds_bender( pwmset, frame, old_matrix_peak_val, CLEAR_ARRAY, mouth );
                    set_matrix_leds_bender( pwmset, frame, new_matrix_peak_val, WRITE_ARRAY, mouth );
                    break;

                default:
                    PRINT_FACE_TYPE( "default button_faceState" );

            } // end of switch(button_faceState)

            //capture old value which will be erased next time
            old_matrix_peak_val = new_matrix_peak_val;

        } // end of update matrix

#if YES_TO_LIFE_LED
        if( statusLedCnt == ISR_FREQ_HZ )
        {
            statusLedState ^= 1;
            digitalWrite( LIFE_LED, statusLedState );
            statusLedCnt = 0;
        }
#endif /* YES_TO_LIFE_LED */
        /* reset state ready for next interrupt */
        sampleNProcess = FINISHED_SAMPLE_AND_PROCESS;
        --checkDataCoherancy;
    }

    if( callERROR_HOOK == 1 )
    {
        ERROR_HOOK( );
    }

} /* end of main loop */

/*
* function to make benders face
*
*/
void set_matrix_leds_bender( uint8_t pwmset, uint8_t frame, uint8_t row_val, uint8_t clear, mouth_data_t *mouthPiece )
{
    uint8_t col   = 0;
    uint8_t i     = 0;
    uint8_t point = 12;
    uint8_t *data = NULL;
    //Serial.print( "\t row_val=\t" ); Serial.println( row_val );

    if( clear )
    {
        data = mouthPiece->trail_clear;
    }
    else
    {
        data = mouthPiece->trail_data;
    }

    for( col = 5; col < MATRIX_LONG_24; col += MATRIX_WIDE_5 )
    {
        // do teeth
        //Serial.print("col=\t"); Serial.print(col);
        for( i = 0; i < MATRIX_WIDE_5; ++i )
        {
            ledDriver.setPwmValue( pwmset, ledDriver.getLedIndex24x5( col, i ), data[ point ] );
        }
    }

    switch( row_val )
    {
        case 0:
        // mouth closed
            for( i = 0; i < MATRIX_LONG_24; ++i )
            {
                // middle lip
                ledDriver.setPwmValue( pwmset, ledDriver.getLedIndex24x5( i, row_val+2 ), data[ point ] );
            }
            break;

        case 1:
        // mouth mid open
            for( i = 0; i < MATRIX_LONG_24; ++i )
            {
                // top lip
                ledDriver.setPwmValue( pwmset, ledDriver.getLedIndex24x5( i, 2-row_val ), data[ point ] );
            }

            for( i = 0; i < MATRIX_LONG_24; ++i )
            {
                //bottom lip
                ledDriver.setPwmValue( pwmset, ledDriver.getLedIndex24x5( i, row_val+2 ), data[ point ] );
            }
            break;

        case 2:
        // mouth open
            for( i = 0; i < MATRIX_LONG_24; ++i )
            {
                // top lip
                ledDriver.setPwmValue( pwmset, ledDriver.getLedIndex24x5( i, 2-row_val ), data[ point ] );
            }

            for( i = 0; i < MATRIX_LONG_24; ++i )
            {
                //bottom lip
                ledDriver.setPwmValue( pwmset, ledDriver.getLedIndex24x5( i, row_val+2 ), data[ point ] );
            }
            break;

    }
    //Serial.println();
}


/*
* function to light up matrix
*
*/
void set_matrix_leds( uint8_t pwmset, uint8_t frame, uint8_t row_val, uint8_t clear, mouth_data_t *mouthPiece, uint8_t lhs_or_rhs )
{
    /*
    * Error if row_val goes higher than 11/12 still prints the LHS trail
    * Error if row_val goes higher than 23 RHS trail goes over 9/10/11
    */
    uint8_t col      = 0;
    uint8_t i        = 0;
    uint8_t lhsvalue = 0;
    uint8_t rhsvalue = 0;
    uint8_t point;
    uint8_t  mod     = 0;
    uint8_t *data    = NULL;

    mod = mouthPiece->array_length;

    MATRIX_DEBUG( "Enter set_matrix_leds( )" );

    if( lhs_or_rhs == LHS )
    {
        MATRIX_DEBUG( ",LHS" );
        lhsvalue = 0;
        rhsvalue = 11;
    }
    else
    {
        MATRIX_DEBUG( ",RHS" );
        lhsvalue = MIDDLE_COLUMN_VAL; //mkr
        rhsvalue = LAST_COLUMN_VAL;   //mkr
    }


    if( clear )
    {
        //mod = mouthPiece->array_length; // 1;
        data = mouthPiece->trail_clear;
    }
    else
    {
        data = mouthPiece->trail_data;
    }

    for( col = 0; col <= 4; ++col )
    {
        //Serial.print("col=\t"); Serial.print(col); Serial.print("\t row_val=\t"); Serial.println(row_val);
        // centre of array is trail_pattern_array[ 12 ]
        // therefore row_val should be trail_pattern_array[12]
        //

        // centre trail - bighest
        // (pwmset '0' , led index, vlaue)
        //ledDriver.setPwmValue(pwmset, ledDriver.getLedIndex24x5(row_val, col), trail_pattern_array[ pattern_start_point ] );
        MATRIX_DEBUG( "LHS trail " );

        // LHS trail and peak value, if (row_val >= 1 ) ok LHS trail
        for( i = 0; i <= mouthPiece->trail_width; ++i )
        {
            if( row_val >= lhsvalue + i )
            {
                // pattern start pointer must not go negative or greter than array size - pass in array size?? or have all same size?
                // this was [ pattern_start_point-i ] but
                point = ( mouthPiece->trail_start_point-i ) % mod;

                ledDriver.setPwmValue( pwmset, ledDriver.getLedIndex24x5( row_val - i, col ), data[ point ] );
                //Serial.print(" LHS data[ "); Serial.print(point);  Serial.print(" ] = "); Serial.print(data[ point ]); Serial.print("\ti="); Serial.println(i);

            }
        }

        MATRIX_DEBUG( "" );
        MATRIX_DEBUG( "RHS trail " );
        // RHS trail, if (row_val <= 10) ok RHS trail
        for( i = 1; i <= mouthPiece->trail_width; ++i )
        {
            // 11 -1 = 10
            // 11 -2 = 9
            if( row_val <= ( rhsvalue - i ) )
            {
              point = ( mouthPiece->trail_start_point + i ) % mod;
              ledDriver.setPwmValue( pwmset, ledDriver.getLedIndex24x5( row_val + i, col ), data[ point ] );
            //Serial.print( " RHS data[ " ); Serial.print( point );  Serial.print( " ] = " ); Serial.print( data[ point ] ); Serial.print( "\ti=" ); Serial.println( i );
            }
        }

        MATRIX_DEBUG( "" );
    }

    //write picture back
    ledDriver.startPicture( frame, false );

    MATRIX_DEBUG( "Leave set_matrix_leds( )" );
}


/*
 *
 * only want 1/4 of the sin/cos putting into half the table
 *
 * |   <-MM->    |
 *
 * max quarter wave length sine is 0-11 = 12 bins
 *
 * sine
 *
*/
void make_sine_table( mouth_data_t *mouthD, uint8_t brightness )
{
    uint8_t i   = 0;
    uint8_t val = 0;

    if( ( mouthD->trail_width > 12 ) || ( mouthD->trail_width < 2 ) )
    {
        //error
        mouthD->trail_width = 11;
    }

    for( i = 0; i <= mouthD->trail_width; ++i )
    {
        val = uint8_t ( (double)brightness * cos( i * ( (PI/2) / (float)mouthD->trail_width) ) ); // do i actually need a double here ?? mkr
        SERIALPRINT( "val = \t" );
        SERIALPRINTLN( val );

        //RHS
        mouthD->trail_data[ 12 + i ] = val;

        //LHD
        mouthD->trail_data[ 11 - i ] = val;
    }

    //
    SERIALPRINT( "\t mouthD->trail_width = \t" ); SERIALPRINTLN( mouthD->trail_width );

    for( i = 0; i < MATRIX_LONG_24; ++i)
    {
        SERIALPRINT( "i = " ); SERIALPRINT( i ); SERIALPRINT( "mouthD->trail_data[ i ] = " ); SERIALPRINTLN( mouthD->trail_data[ i ] );
    }

    SERIALPRINTLN( "" );
}


/*
 * draws a picture  |__*__  |
 * '|' bars indicate entire adc range, 0-1023, left '|' == 0, right '| == ADC_FULL_SCALE_RANGE.
 * '__' indicate range under inspection ie lowestDCOffset/highest_readVAlue adc range.
 * the '*' is the adc val.
 * range of adc/lDCOff/highRead == ADC_FULL_SCALE_RANGE, i.e. 0-1023.
 * hardcoded... :(
 * divides that by 64, i.e. into 16 chunks (although actual Matrix is 11)
 * looks similar to DO_PRINT_AUTOGAIN_VALS( ) but fancier.
*/
/*
could improve readability of this function
*/
void print_autogain_display( int16_t adcV, uint16_t lDCOff, uint16_t highRead )
{
#ifdef DO_PRINT_AUTOGAIN_DISPLAY
    // convert to 16 bit chunks value
    uint8_t verticalBarLine = 16;
    adcV     /= 64;
    lDCOff   /= 64;
    highRead /= 64;

    Serial.print( "|" );

    while( lDCOff > 0 )
    {
        Serial.print( " " );
        if( adcV     > 0 ) adcV--;
        if( lDCOff   > 0 ) lDCOff--;
        if( highRead > 0 ) highRead--;
        if( verticalBarLine > 0 )  verticalBarLine--;
    }

    while( adcV > 0 )
    {
        Serial.print( "_" );
        if( adcV     > 0 ) adcV--;
        if( lDCOff   > 0 ) lDCOff--;
        if( highRead > 0 ) highRead--;
        if( verticalBarLine > 0 )  verticalBarLine--;
    }

    Serial.print( "*" );
    if( adcV     > 0) adcV--;
    if( lDCOff   > 0) lDCOff--;
    if( highRead > 0) highRead--;
    if( verticalBarLine > 0 )  verticalBarLine--;

    while( highRead > 0 )
    {
        Serial.print( "_" );
        if( adcV     > 0) adcV--;
        if( lDCOff   > 0) lDCOff--;
        if( highRead > 0) highRead--;
        if( verticalBarLine > 0 )  verticalBarLine--;
    }

    while( verticalBarLine > 0 )
    {
        Serial.print( " " );
        if( adcV     > 0 ) adcV--;
        if( lDCOff   > 0 ) lDCOff--;
        if( highRead > 0 ) highRead--;
        if( verticalBarLine > 0 )  verticalBarLine--;
    }

  Serial.println( "|" );
#endif
}

/*
 * if timer interrupt, interrupts the process loop, then things are bad :(
 * wasn't sure weather to count the number of errors or while( 1 );
 * went with while( 1 ); because i don't want any errors, so one is bad enough
 * already...
*/
void ERROR_HOOK( void )
{
    SERIALPRINTLN( "ERROR_HOOK!\t" );
    while( 1 )
    {
        /* LED pattern. */
        digitalWrite( LIFE_LED, HIGH );
        delay( 500 );
        digitalWrite( LIFE_LED, LOW );
        delay( 250 );
    }
}
