// HARDARE  -> PINS:
// ================

//  Analogue:
//  ========
// sck -> arduino a0 to mic opamp
// sck -> arduino a4
// sda -> arduino a5   

//  Digital:
//  =======
// pin output to matrix reset -> arduino pin d2
// button interrupt   d3
// mosfet to clear peak detector -> arduino d7
// speaker -> arduino
// power -> arduino 5V to  matrix
// power -> arduino 3.3V to .. nothing 
// ground -> mic and to matrix
//

// debounce circuit from http://www.ganssle.com/debouncing-pt2.htm
// DO_BUTTON_INTERRUPT true
// R1 = 82kOhm
// R2 = 17K7 Ohm
// C = 1uF
// button_0_func()


//=-=-=-=//
// NOTES
//=-=-=-=//
/*
 * Designed on Arduino. 
 * Assumes 10 bit ADC 0-5V
 * Different power supplies/lines for both adc and Matrix 
 * Matrix 24*5 led display
 * 
 * Software overview :
 * =-=-=-=-=-=-=-=-=-=
 * Uses a timer ISR to trigger adc read. ( ISR_FREQ_HZ )
 * 
 * this value is mapped to a value 0-11,
 * this value is
 * 
 * adc_state .. SAMPLE_ADC_DATA | PROCESS_ADC_DATA 
 * 
 * 
 * 
 * 
 * 
back ground pwm variable -knob
function to make up the sine/gauss tables; knob for decay, knob for brightness

rename lowest_dcoffset -> auto_gain_lowest
highest_input_read -> auto_gain_max

filter 
*/

// analogReference(type) EXTERNAL EXTERNAL: the voltage applied to the AREF pin (0 to 5V only) is used as the reference.
// could filter signal for smoothing now

#include "LRAS1130.h"
#include "coffee_define.h"

/***************
 * Prototypes
 * 
***************/
void set_matrix_leds(uint8_t frame, uint8_t row_val, uint8_t set_clear, uint8_t *trail_pattern_array, uint8_t lhs_or_rhs, uint8_t pattern_width, uint8_t pattern_start_point, uint8_t array_size );
void print_autogain_display( int16_t x, uint16_t y,uint16_t z );
//void make_sine_table(uint16_t trail_width, uint16_t table, uint8_t brightness);
void make_sine_table(uint16_t trail_width, mouth_data *mouthD, uint8_t brightness);
void button_0_func();

using namespace lr;
AS1130 ledDriver;


/***************
 * Globals
 * 
***************/

static volatile uint8_t adc_state = SAMPLE_ADC_DATA;             // either SAMPLE_ADC_DATA or PROCESS_ADC_DATA

// auto gain
static uint16_t lowest_dcoffset = 1024;
static uint16_t highest_input_read = 250;

#ifdef MUSIC
  #include "pitches.h"
  
  // notes in the melody:
  int melody[] = {
    NOTE_C4, NOTE_G3, NOTE_G3, NOTE_A3, NOTE_G3, 0, NOTE_B3, NOTE_C4
  };
  
  // note durations: 4 = quarter note, 8 = eighth note, etc.:
  int noteDurations[] = {
    4, 8, 8, 4, 4, 4, 4, 4
  };
#endif

static int buttonState = 0;         // variable for reading the pushbutton status


    /***************
     * Matrix
    ***************/
const uint8_t AllOnFrame[] = {
0b11111111, 0b11111111, 0b11111111,
0b11111111, 0b11111111, 0b11111111,
0b11111111, 0b11111111, 0b11111111,
0b11111111, 0b11111111, 0b11111111,
0b11111111, 0b11111111, 0b11111111};

const uint8_t background_array[] = {BACKGROUND_PWM_VAL};

    /***************
     * Matrix Gauss
    ***************/
    
const uint8_t CENTRE_OF_GAUSS_DIST = 0; // same as max gauss 

const uint8_t gauss_dist_array[] = {0,0,0,0,0,0,0,0,0,3,34,154,255,154,34,3,0,0,0,0,0,0,0,0,0};  //uint8_t GAUSS_DIST_WIDTH  = 3;
//uint8_t gauss_dist_array[] = {255,154,34,3,0,0,0,0,0,0,0,0,0};  //uint8_t GAUSS_DIST_WIDTH  = 3;
//uint8_t gauss_dist_array[] = {0,0,0,1,2,4,7,11,15,20,24,27,28,27,24,20,15,11,7,4,2,1,0,0,0}; // uint8_t GAUSS_DIST_WIDTH  = 9;

#ifdef VU_TYPE_GAUSS
  mouth_data mouth = 
  {
    .trail_start_point  = 12,
    .width              = 3,
    .trail_clear        = &background_array[0],
    .trail_type         = &gauss_dist_array[0],
    .arr_size           = 24,
  };
#endif


    /***************
     * Matrix SINE
    ***************/
//const uint8_t sine_table[] = {255,251,238,218,191,160,128,95,64,37,17,4,0,4,17,37,64,95};
uint8_t sine_table[24] = {0}; // 256,128,34,0
//uint8_t sine_table[] = {255,128,34,0,34,128}; // 256,128,34,0

// 128,191,238,255,238,191,128,64,17,0,17,64,128


#ifdef VU_TYPE_SINE
  mouth_data mouth = 
  {
    .trail_start_point  = 12, //wher in array to start looping from, was 0
    .width              = 3,       // width of lips
    .trail_clear        = &background_array[0],
    .trail_type         = &sine_table[0],
    .arr_size           = 24,    // loop length was smaller...!
  };
#endif

/***************
 * Setup
 * 
***************/

void setup() 
{
  #ifdef USE_MOSFET 
    pinMode(MOSFET_GATE_PIN, OUTPUT);
  #endif
  
  
  Wire.begin();
  Wire.setClock( I2C_SPEED );
  
  #ifdef DO_SERIAL_DEBUG || DO_PRINT_ADVALUE || DO_MATRIX_DEBUG || DO_PRINT_MATRIXVALUE || DO_PRINT_AUTOGAIN_DISPLAY || DO_PRINT_AUTOGAIN_VALS
    Serial.begin(SERIAL_PORT_BAUD);
  #endif
  
  // Wait until the chip is ready.
  delay(500); 
  
  SERIALPRINT(F("Reset chip"));
  
  // hard reset the matrix 
  digitalWrite(MATRIX_RESETPIN, LOW);
  delay(500);
  digitalWrite(MATRIX_RESETPIN, HIGH);
  
  SERIALPRINT(F("Initialize chip"));

  
  // Check if the chip is addressable.
  
  if (!ledDriver.isChipConnected()) 
  {
    SERIALPRINT(F("Communication problem with chip."));
    //Serial.flush();
    return;
  }

  /* make up sine table */
  // void make_sine_table(uint16_t trail_width, moutn *mouth, uint8_t brightness)
  make_sine_table( 3, &mouth, 128);
  
  /*  Set-up everything.
   *  Ram config 1, with 36 frames and 1 pwm set, we require 1 frame (all leds on) and 1 pwm set ( which we alter) 
   * 
  */  
  ledDriver.setRamConfiguration(AS1130::RamConfiguration1); //  ramconf 1 = 36 frames and 1 pwmset  // ram cofig 5 = 5 pwm sets and 12 frames
  ledDriver.setOnOffFrame24x5(0, AllOnFrame, 0); // void setOnOffFrame24x5(uint8_t frameIndex, const uint8_t *data, uint8_t pwmSetIndex = 0);
  
  // Set-up a blink&PWM set with values for all LEDs.
  ledDriver.setBlinkAndPwmSetAll(0, false, BACKGROUND_PWM_VAL); // void setBlinkAndPwmSetAll(uint8_t setIndex, bool doesBlink = false, uint8_t pwmValue = 0xff);

  
  ledDriver.setCurrentSource(AS1130::Current5mA);
  ledDriver.setScanLimit(AS1130::ScanLimitFull);
  ledDriver.startPicture(0, false); // startPicture(uint8_t frameIndex, bool blinkAll = false);
  
  // Enable the chip
  ledDriver.startChip();

  // initialize timer1 
  noInterrupts();
  TCCR1A = 0;
  TCCR1B = 0;  
  TCNT1 = TIMER1_COUNTER;   // preload timer
  TCCR1B |= (1 << CS12);    // 256 prescaler 
  TIMSK1 |= (1 << TOIE1);   // enable timer overflow interrupt

  #if DO_BUTTON_INTERRUPT
    pinMode(BUTTON_PIN_0, INPUT_PULLUP);
    attachInterrupt(digitalPinToInterrupt(BUTTON_PIN_0), button_0_func, FALLING );
  #endif

  interrupts();             // enable all interrupts

  #ifdef MUSIC
    // iterate over the notes of the melody:
    for (int thisNote = 0; thisNote < 8; thisNote++) {
  
      // to calculate the note duration, take one second
      // divided by the note type.
      //e.g. quarter note = 1000 / 4, eighth note = 1000/8, etc.
      int noteDuration = 1000 / noteDurations[thisNote];
      tone(8, melody[thisNote], noteDuration);
  
      // to distinguish the notes, set a minimum time between them.
      // the note's duration + 30% seems to work well:
      int pauseBetweenNotes = noteDuration * 1.30;
      delay(pauseBetweenNotes);
      // stop the tone playing:
      noTone(8);
    }
  #endif


  //pinMode(A0,INPUT_PULLUP);

  #ifdef USE_MOSFET
    // this switch enables the capacitor in the peak dector circuit so atart collecting data about the mic state
    digitalWrite(MOSFET_GATE_PIN, LOW);  
  #endif
}


ISR(TIMER1_OVF_vect)
{
  /* Acquire data */
  if ( adc_state == SAMPLE_ADC_DATA )
  {
    adc_state = PROCESS_ADC_DATA ;
  }
  
  TCNT1 = TIMER1_COUNTER;   // reload timer
}

#if DO_BUTTON_INTERRUPT
//effectively my isr for button press
  uint8_t button_state = 0;
  
  void button_0_func()
  {
    //alternate face type
    if ( button_state == KIT_FACE)
    {
       button_state = BENDER_FACE;
    }
    else
    {
      button_state = KIT_FACE;
    }
  }
#endif


void loop() 
{
  static uint16_t new_matrix_peak_val = 0;
  static uint16_t old_matrix_peak_val = 1;
  static uint8_t  max_map_val = 11; 
  
  static int16_t adcVals[3] = {0,0,0};

  /* Process Data */
  if( adc_state == PROCESS_ADC_DATA )
  {
    #if DO_BUTTON_INTERRUPT
      if ( button_state == KIT_FACE)
      {
        max_map_val = 11; // 0-11 good for KIT_FACE style
        //Serial.println("kitnface");
      }
      else // ( button_state == BENDER_FACE )
      {
        max_map_val = 2;
        //Serial.println("bender face");
      }
    #endif
    
    uint16_t adc_val = analogRead(A0);

    #ifdef USE_MOSFET
    //mosfet goes high so capacitor will reset to 0 voltage value
    digitalWrite(MOSFET_GATE_PIN, HIGH);  
    delay(5);
    digitalWrite(MOSFET_GATE_PIN, LOW);
    #endif
        
    #ifdef MIC_TO_SPEAKER
      analogWrite(PWMPIN, 100 + adc_val);
    #endif

    PRINT_ADVALUE(adc_val);


    /* AutoGain Section */
    
    // :recalibrate range of data, to max the range because polsives make a mess of the range
    // ignore low end noise
    // ignore top end space
    
    lowest_dcoffset += INCREASE_MIN_RANGE_VALUE; //always knock minimum value up
    
    if(lowest_dcoffset > adc_val)
    {      
      lowest_dcoffset = adc_val; // recalibrate min -knock back down
    }

    if( highest_input_read > (lowest_dcoffset + MINMUM_MIC_RANGE) ) // range minimum is 200 
    {
      highest_input_read -= DECREASE_MAX_RANGE_VALUE; // always knock max value down
    }
    
    if(highest_input_read < adc_val)
    {
      highest_input_read = adc_val; // recalibrate max  -knock back up
    }
    
    PRINT_AUTOGAIN(adc_val, lowest_dcoffset, highest_input_read);
    
    #ifdef DO_PRINT_AUTOGAIN_DISPLAY 
      print_autogain_display( adc_val, lowest_dcoffset, highest_input_read ); 
    #endif
    
    // Whatever the relative values are 
    //
    adcVals[0] = map( adc_val, lowest_dcoffset, highest_input_read , max_map_val, 0);   
         
    //if( (adcVals[0] > 11)  || (adcVals[0] < 0) )
    //{
      //adcVals[0] = 0;
    //}

    /* Filter section */
    //  low pass, moving average 3
    // new_matrix_peak_val =  (adcVals[0]  + adcVals[1] + adcVals[2])/3;
    // adcVals[1] = adcVals[0]; // update 
    // adcVals[2] = adcVals[1]; // update 
    
    // moving average 2
    // new_matrix_peak_val =  (adcVals[0]  + adcVals[1])/2;
    // adcVals[1] = adcVals[0]; // update 

    // no lowpass
    new_matrix_peak_val = adcVals[0];

    PRINT_MATRIXVALUE(new_matrix_peak_val);

    /*  */
    //if(new_matrix_peak_val != old_matrix_peak_val )
    {
        #ifndef PULSE_MOUTH
          static uint8_t trail_start_point = mouth.trail_start_point;
        #else
          static uint8_t trail_start_point = mouth.trail_start_point;
          trail_start_point = mouth.trail_start_point++;
        #endif
        
        uint8_t width        = mouth.width;
        uint8_t *trail_clear = mouth.trail_clear;
        uint8_t *trail_type  = mouth.trail_type;
        uint8_t arr_size     = mouth.arr_size; // mod this 
      

      #ifdef PULSE_MOUTH
         //trail_start_point = mouth.trail_start_point++;
      #endif

      /* Send values to the Matrix */
      
      // set_matrix_leds( frame,  row_val,  set_clear,  *trail_pattern_array,  lhs_or_rhs,  pattern_width,  pattern_start_point, array_size )
      // removed made change so that LHS and RHS have the same trail, but 
      // instead put in arr_size for pulse mouth 
      // set clear not used
      // 
      
      set_matrix_leds(0, old_matrix_peak_val, 0, trail_clear, LHS, width, trail_start_point, 1 );
      set_matrix_leds(0, new_matrix_peak_val, 0, trail_type,  LHS, width, trail_start_point, arr_size );
      
      //clear RHS then write new values
      set_matrix_leds(0, 24-old_matrix_peak_val, 0, trail_clear, RHS, width, trail_start_point, 1 );
      set_matrix_leds(0, 24-new_matrix_peak_val, 0, trail_type,  RHS, width, trail_start_point, arr_size );
      
      old_matrix_peak_val = new_matrix_peak_val;
    }

    /* reset state ready for next interrupt */
    adc_state = SAMPLE_ADC_DATA;
  }

} /* end of main loop */



void set_matrix_leds(uint8_t frame, uint8_t row_val, uint8_t set_clear, uint8_t *trail_pattern_array, uint8_t lhs_or_rhs, uint8_t pattern_width, uint8_t pattern_start_point, uint8_t array_size )
{
  uint8_t col=0;
  //const uint8_t CENTRE_OF_GAUSS_DIST = 12; // same as max gauss 
  uint8_t i=0;
  uint8_t lhsvalue = 0;
  uint8_t rhsvalue = 0;
  uint8_t point;

  MATRIX_DEBUG("Enter set_matrix_leds()");
  
  if(lhs_or_rhs == LHS)
  {
    MATRIX_DEBUG("LHS");
    
    lhsvalue = 0;
    rhsvalue = 11;
  }
  else
  {
    MATRIX_DEBUG(",RHS");
    
    lhsvalue = 12; //MIDDLE_COLUMN_VAL;
    rhsvalue = 23; //LAST_COLUMN_VAL;
  }
  
  // 
  for(col=0; col<=4; col++)
  {
    // centre of array is trail_pattern_array[ 12 ]
    // therefore row_val should be trail_pattern_array[12] 
    //
    
    // centre trail - bighest
    // (frame '0' , led index, vlaue)
    ledDriver.setPwmValue(frame, ledDriver.getLedIndex24x5(row_val, col), trail_pattern_array[ pattern_start_point ] );
      
    // LHS trail, if (row_val >= 1 ) ok LHS trail
    for(i=1; i<=pattern_width; i++)
    {
        if (row_val >= lhsvalue+i) 
        {
          // pattern start pointer must not go negative or greter than array size - pass in array size?? or have all same size?
          // this was [ pattern_start_point-i ] but 
          point = (pattern_start_point-i ) % array_size;
          
          ledDriver.setPwmValue(frame, ledDriver.getLedIndex24x5(row_val-i, col), trail_pattern_array[ point ] );
          MATRIX_DEBUG("LHS trail ");
        }
    }
    MATRIX_DEBUG("");
    
    // RHS trail, if (row_val <= 10) ok RHS trail
    for(i=1 ; i<=pattern_width; i++)
    {
    // 11 -1 = 10
    // 11 -2 = 9
        if (row_val <= (rhsvalue-i) )
        {
          point = (pattern_start_point+i ) % array_size;
          ledDriver.setPwmValue(frame, ledDriver.getLedIndex24x5(row_val+i, col), trail_pattern_array[ point ] );
          
          MATRIX_DEBUG("RHS trail ");
        }
    }
    MATRIX_DEBUG("");
  }

  //write picture back
  ledDriver.startPicture(0, false);
  
  MATRIX_DEBUG("Leave set_matrix_leds()");
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
void make_sine_table(uint16_t trail_width, mouth_data *mouthD, uint8_t brightness)
{
  uint8_t i=0;
  uint8_t val =0;

  mouthD->width = trail_width;
  //uint8_t *tablet = mouthD->trail_type[0];

  if( (trail_width > 12) || (trail_width < 2) )
  {
    //error
    Serial.println("Sine table error");
    return;
  }
  
  for(i=0; i<=trail_width; i++)
  {
    val = uint8_t ( (double)brightness * cos( i * ( (PI/2)/(float)trail_width)) );
    Serial.print("val = \t");
    Serial.println(val);
    
    //RHS
    //tablet[12+i] = val; // not note start at 1
    mouthD->trail_type[12 + i] = val;
    
    //LHD
    //tablet[11 - i] = val;
    mouthD->trail_type[11-i] = val;
  }

  //
  Serial.print("trail_width = "); Serial.print(trail_width); Serial.print("\t mouthD->width = \t"); Serial.println( mouthD->width); 
  for(i=0; i<24; i++)
  {
    Serial.print("i = "); Serial.print(i); Serial.print("mouthD->trail_type[i] = "); Serial.println( mouthD->trail_type[i]); 
  }
  Serial.println("");

  // rads=2*pi*f
}


#ifdef DO_PRINT_AUTOGAIN_DISPLAY
void print_autogain_display( int16_t x, uint16_t y,uint16_t z) 
{
  x=(x/64);
  y=(y/64);
  z=(z/64);
  uint8_t v=16;
  
  Serial.print("|");
  
  while(y>0)
  {
    Serial.print(" ");

    if ( x>0) x--;
    if ( y>0) y--;
    if ( z>0) z--;
    if ( v>0) v--;
  }
  
  while(x>0)
  {
    Serial.print("_");
    if ( x>0) x--;
    if ( y>0) y--;
    if ( z>0) z--;
    if ( v>0) v--;
  }
  
  Serial.print("*");
    if ( x>0) x--;
    if ( y>0) y--;
    if ( z>0) z--;
    if ( v>0) v--;
    
  while(z>0)
  {
    Serial.print("_");
    if ( x>0) x--;
    if ( y>0) y--;
    if ( z>0) z--;
    if ( v>0) v--;
  }

  while(v>0)
  {
    Serial.print(" ");
    if ( x>0) x--;
    if ( y>0) y--;
    if ( z>0) z--;
    if ( v>0) v--;
  }
  
  Serial.println("|");
  
}
#endif


