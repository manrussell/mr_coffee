#ifndef __HEADER_FILE__
#define __HEADER_FILE__

#define TRUE    1
#define FALSE   0
#define LHS     0
#define RHS     1

/*
 * Functionality 
 * 
*/
#define BACKGROUND_PWM_VAL  0

// Pulse mouth a bit flakey - hanging values
//#define PULSE_MOUTH   
#define BENDER              0
#define KNIGHT_RIDER        0

//#define MUSIC
//#define MIC_TO_SPEAKER
//#define USE_LIGHT_SENSITIVITY_KNOB

/*
 * Hardware
 * 
*/
  //Arduino
#define ISR_FREQ_HZ         60
#define CLOCK_SPEED_MHZ     16000000
#define DO_BUTTON_INTERRUPT  TRUE

#define KIT_FACE    0
#define BENDER_FACE 1

// maybe rename sample_isr>>
#define TIMER1_COUNTER    (65536-(CLOCK_SPEED_MHZ/256)/ISR_FREQ_HZ)

#define I2C_SPEED         800000L
#define SERIAL_PORT_BAUD  115200
#define MIDDLE_COLUMN_VAL 12
#define LAST_COLUMN_VAL   23
#define NUM_OF_ROWS       5
#define PWMPIN            8
#define MOSFET_GATE_PIN   7

#define BUTTON_PIN_0        3

  //Matrix
#define MATRIX_RESETPIN   2

/* 
 * Matrix Visuals
 * 
*/

//#define VU_TYPE_SINE
#define VU_TYPE_GAUSS

//better description here ... 
#define GAUSS_DIST_WIDTH  3 // one half - exclude the max value eg 1,2,3,(X),3,2,1

/*
 * Auto gain
 * 
*/

#define MINMUM_MIC_RANGE          200
#define INCREASE_MIN_RANGE_VALUE  1
#define DECREASE_MAX_RANGE_VALUE  5

#define SAMPLE_ADC_DATA   0
#define PROCESS_ADC_DATA  1

/* -need testing use a mosfet to 'clear' the state of the capacitor */
//#define USE_MOSFET

/*
 *  Debug macros
*/ 

/* need this for serial to work */
#define DO_SERIAL_DEBUG 

/* PRINT_ADVALUE(adc_val); */
#define DO_PRINT_ADVALUE

/* MATRIX_DEBUG prints out text in the matrix bit */
//#define DO_MATRIX_DEBUG

/* PRINT_MATRIXVALUE(new_matrix_peak_val); */
#define DO_PRINT_MATRIXVALUE 

/* DO_PRINT_AUTOGAIN_VALS: prints adc-val x lowest_dcoffset x hoghest_input_read */
//#define DO_PRINT_AUTOGAIN_VALS

/*
 * DO_PRINT_AUTOGAIN_DISPLAY: a picture  |__*__  |
 * '|' bars indicate entire adc range
 * '__' indicate range under inspection ie lowest/highest adc range
 * the asterix is the adc val
*/
//#define DO_PRINT_AUTOGAIN_DISPLAY // now a function

/* 
 *  DEBUG MACROS 
 */
#ifdef USE_SERIALPRINT
  #define SERIALPRINT( x ) (Serial.println(x))
  //#define SERIALPRINT( ... ) Serial.println(__VA_ARGS__);
#else
  #define SERIALPRINT(x)
#endif

#ifdef DO_MATRIX_DEBUG
  #define MATRIX_DEBUG( x ) (Serial.println(x))
#else
  #define MATRIX_DEBUG(x)
#endif

#ifdef DO_PRINT_ADVALUE
  #define PRINT_ADVALUE( x ) ( Serial.print("adc = \t"), Serial.println(x))
#else
  #define PRINT_ADVALUE(x)
#endif

#ifdef DO_PRINT_MATRIXVALUE
  #define PRINT_MATRIXVALUE( x ) (Serial.print("new_matrix Value = "),  Serial.println(x))
#else
  #define PRINT_MATRIXVALUE(x)
#endif


#ifdef DO_PRINT_AUTOGAIN_VALS
  #define PRINT_AUTOGAIN( x,y,z ) (Serial.print("adc_val \t"),              Serial.print(x),  \
                                  Serial.print("\t lowest_dcoffset \t"),    Serial.print(y),  \
                                  Serial.print("\t highest_input_read \t"), Serial.println(z)  )
#else
  #define PRINT_AUTOGAIN(x,y,z)
#endif

/* 
 * Types
 * 
 * 
*/

/*
const uint8_t pwmsetBenderMouth[] = {
0,0,0,255,0,0,0,255, 0,0,0,255,0,0,0,255, 0,0,0,255,0,0,0,0,
0,0,0,255,0,0,0,255, 0,0,0,255,0,0,0,255, 0,0,0,255,0,0,0,0,
0,0,0,255,0,0,0,255, 0,0,0,255,0,0,0,255, 0,0,0,255,0,0,0,0,
0,0,0,255,0,0,0,255, 0,0,0,255,0,0,0,255, 0,0,0,255,0,0,0,0,
0,0,0,255,0,0,0,255, 0,0,0,255,0,0,0,255, 0,0,0,255,0,0,0,0};
*/

typedef struct
{
  uint8_t trail_start_point;
  uint8_t width;
  uint8_t *trail_clear;
  uint8_t *trail_type;
  uint8_t arr_size; // loop length
}mouth_data;

#endif /* __HEADER_FILE__ */

