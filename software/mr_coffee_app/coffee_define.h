#ifndef COFFEE_DEFINES_H
#define COFFEE_DEFINES_H
/*
 * Arduino UNO/protrinket Notes:
*  Analogue: PINS (Arduino Uno / pro trinket )
*  ========
* sck -> arduino a0 to mic opamp
* Assumes 10 bit ADC 0-5V
* Different power supplies/lines for both adc and Matrix
* sck -> arduino a4
* sda -> arduino a5
*
*  Digital:
*  =======
* pin output to matrix reset -> arduino pin d4
* button interrupt_0   d2 // -> there is no D2 on protrinket!! just uno so use
* 'pin-change interrupt' see http://playground.arduino.cc/Main/PinChangeInterrupt
* button interrupt_1   d3
* mosfet to clear peak detector -> arduino d7 - now removed
* speaker -> arduino
* power -> arduino 5V to  matrix
* power -> arduino 3.3V to .. nothing
* ground -> mic and to matrix
*
* debounce circuit from http://www.ganssle.com/debouncing-pt2.htm
* DO_BUTTON_INTERRUPT true
* R1 = 82kOhm
* R2 = 17K7 Ohm
* C = 1uF
* ISR_button_ledBrightness()
*/

/*
* ESP8266 Notes:
*/

#define ISR_FREQ_HZ                 ( 30 )

/**********************
 * Hardware
***********************/
/*
 * Board specific hardware.
 * Compiler defines: ESP8266, __AVR__,
*/
#if defined __AVR__
#   define CLOCK_SPEED_MHZ          ( 16000000 )
#   define TIMER1_COUNTER           ( 65536 - ( CLOCK_SPEED_MHZ / 256 ) / ISR_FREQ_HZ ) // maybe rename sample_isr>>
    /* HW buttons digital input pins */
#   define BUTTON_PIN_LED_BRIGHTNESS ( 2 )
#   define BUTTON_PIN_FACE          ( 3 )
#   define LIFE_LED                 ( 13 )
#   define ADC_PIN                  ( A0 )
#   define ADC_FULL_SCALE_RANGE     ( 1023 ) /* UNO has 10-bit 5Volt available. Value must be (pow(2,x)-1). */
#   define MATRIX_RESETPIN          ( 4 )
#   define I2C_SPEED                ( 800000L )
#   define SIGNAL_HAS_DCOFFSET      ( 0 )
#   warning building for AVR
#elif defined ESP8266
#   define TIMER_INTERVAL_MS        ( (uint8_t)( 1000 / ISR_FREQ_HZ ) ) /* 1000 to convert Hz to milliseconds. */
/* buttons ... */
#   define LIFE_LED                 ( 5 )
#   define ADC_PIN                  ( A0 )
#   define ADC_FULL_SCALE_RANGE     ( 1023 ) /* 10-bit 1Volt ADC available. Value must be (pow(2,x)-1). */
#   define MATRIX_RESETPIN          ( 4 )   /* '0' or '4' available */
/* I2C
 * how does wire know which I2C line to use?
 * ESP8266 == i2c pins 2==Data clk==14
 * Also, what is the max speed for each board?
 * do i just set this to the highest freq available it it does the rest?
 * lr:AS1130 driver does not have 800000, it has 500K or 1MHz ..hmmm, this worked for the UNO
*/
#   define I2C_SPEED                ( 800000L )
#   define SIGNAL_HAS_DCOFFSET      ( 1 )
#   warning building for esp8266
#endif /* ESP8266 */

/*
* If the input of the signal has a dc offset eg ADC/2.
* In that case then we right shift by '1' to half the ADC read value.
* Could have used a subtraction in the code but this should be fine.
*/
#if SIGNAL_HAS_DCOFFSET
#   define DCOFFSET_VALUE           ( 1 )
#endif /* SIGNAL_HAS_DCOFFSET */
/*
 * Serial Hardware
 */
#define SERIAL_PORT_BAUD            ( 115200 )

/* if enabled then a led (LIFE_LED) will turn on/off on the board every second. */
#define YES_TO_LIFE_LED             ( 1 )
/**********************
 * Software
**********************/
/* this is modes +1 because this value is modulous'd */
#define BUTTON_LED_CURRENT_MODES    ( 7 )   /* the number of available lr:AS1130's 'enum Current'. */
#define BUTTON_FACE_TYPE_MODES      ( 5 )   /* the number of different faces in the switch block. */
/*
 * Matrix led array descriptions
*/
#define BACKGROUND_PWM_VAL          ( 0 )
#define MAX_PWM_BRIGHTNESS          ( 255 )
#define LHS                         ( 0 )
#define RHS                         ( 1 )
#define MIDDLE_COLUMN_VAL           ( 12 )
#define LAST_COLUMN_VAL             ( 23 )
#define NUM_OF_ROWS                 ( 5 )
#define MATRIX_WIDE_5               ( 5 )   // same as num of rows!! maybe duplicate i think, which has better name ??
#define MATRIX_LONG_24              ( 24 )
#define CLEAR_ARRAY                 ( 1 )
#define WRITE_ARRAY                 ( 0 )

/*
 * program state
*/
#define FINISHED_SAMPLE_AND_PROCESS ( 0 )
#define GO_SAMPLE_AND_PROCESS       ( 1 )

/*
 * Auto gain
*/
/* how is this determined? maybe better a dynamic variable would be more fun. e.g. knobs or sliders. */
#define MINMUM_MIC_RANGE            ( 200 ) // maybe just 11 as that really is the minimum value
#define INCREASE_MIN_RANGE_VALUE    ( 1 )
#define DECREASE_MAX_RANGE_VALUE    ( 5 )

/*
 * Generic
*/
#define TRUE                        ( 1 )
#define FALSE                       ( 0 )

/**************************
 *  Configure Debug macros
***************************/
#define DISABLE_MATRIX_OVER_I2C_COMMS   /* stops set-up of I2C for the Matrix. */
//#define SIGNAL_INPUT_IS_SAWTOOTH        /* instead of ADC inject a sawtooth signal. */

/* need this for serial to work */
#define DO_SERIAL_DEBUG

/* PRINT_ADCVALUE( adc_val ); */
//#define DO_PRINT_ADCVALUE

/* MATRIX_DEBUG prints out text in the matrix bit */
//#define DO_MATRIX_DEBUG

/* PRINT_MATRIXVALUE( new_matrix_peak_val ); */
//#define DO_PRINT_MATRIXVALUE

/* DO_PRINT_AUTOGAIN_VALS: prints adc-val x lowest_dcoffset x highest_input_read after these have been calculated. */
//#define DO_PRINT_AUTOGAIN_VALS

/*
 * DO_PRINT_AUTOGAIN_DISPLAY: a picture  |__*__  |
 * '|' bars indicate entire adc range, i.e. one half of Matrix led displays output
 * '__' indicate range under inspection ie lowest/highest adc range
 * the asterix is the adc val.
 * similar to DO_PRINT_AUTOGAIN_VALS( ) but fancier, also runs after these have been calculated.
*/
#define DO_PRINT_AUTOGAIN_DISPLAY

/* in the switch block prints out a msg of the current face type e.g. bender. */
//#define ENABLE_PRINT_FACE_TYPE

/*********************************************
 * Implementation of debug macros.
 * remember Serial.print( ) is not printf( );
**********************************************/
#ifdef DO_SERIAL_DEBUG
#   define SERIALPRINT( x_ )      ( Serial.print( x_ ) )
#   define SERIALPRINTLN( x_ )    ( Serial.println( x_ ) )
#   define SERIALPRINTLNDEC( x_ ) ( Serial.println( x_, DEC ) )
#else
#   define SERIALPRINT( x_ )
#   define SERIALPRINTLN( x_ )
#   define SERIALPRINTLNDEC( x_ )
#endif

#ifdef DO_MATRIX_DEBUG
#   define MATRIX_DEBUG( x_ ) ( Serial.println( x_ ) )
#else
#   define MATRIX_DEBUG( x_ )
#endif

#ifdef DO_PRINT_ADCVALUE
#   define PRINT_ADCVALUE( x_ ) ( Serial.print( "adc = \t" ), Serial.println( x_ ) )
#else
#   define PRINT_ADCVALUE( x_ )
#endif

#ifdef DO_PRINT_MATRIXVALUE
#   define PRINT_MATRIXVALUE( x_ ) ( Serial.print( "new_matrix Value = " ), Serial.println( x_ ) )
#else
#   define PRINT_MATRIXVALUE( x_ )
#endif

#ifdef DO_PRINT_AUTOGAIN_VALS
/* x == ADC value. y == lowest_dcoffset, z ==highest_input_read */
#   define PRINT_AUTOGAIN( x_ ,y_ ,z_ ) ( Serial.print( "adc " ),        Serial.print( x_ ),     \
                                          Serial.print( "\tLDc " ),      Serial.print( y_ ),     \
                                          Serial.print( "\thighRead " ), Serial.println( z_ )  )
#else
#   define PRINT_AUTOGAIN( x_ ,y_ ,z_ )
#endif

#ifdef ENABLE_PRINT_FACE_TYPE
#   define PRINT_FACE_TYPE( x_ ) ( Serial.println( x_ ) )
#else
#   define PRINT_FACE_TYPE( x_ )
#endif

/**********************
 * Types
**********************/
typedef struct
{
    uint8_t trail_start_point;  // set to 11 so, the min LHS or is it RHS
    uint8_t trail_width;
    uint8_t array_length;       // loop length
    uint8_t *trail_clear;
    uint8_t *trail_data;
} mouth_data_t;

/*
* Used by button presses and the switch block.
*/
enum faceTypes_t
{
    mrCoffeeBoldFlashMouth  = 0,
    mrCoffeeWavyMouth       = 1,
    gaussInwardsSweepMouth  = 2,
    knightRiderMouth        = 3,
    benderMouth             = 4,
};

/* some kind of led state struct ? */

#endif /* COFFEE_DEFINES_H */
