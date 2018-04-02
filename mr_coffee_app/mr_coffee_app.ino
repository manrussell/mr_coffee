// HARDARE  -> PINS:
// ================

//  Analogue:
//  ========
// sck -> arduino a0 to mic opamp
// sck -> arduino a4
// sda -> arduino a5   

//  Digital:
//  =======
// pin output to matrix reset -> arduino pin d4
// button interrupt_0   d3
// button interrupt_1   d2
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
//void set_matrix_leds(uint8_t frame, uint8_t row_val, uint8_t set_clear, uint8_t *trail_pattern_array, uint8_t lhs_or_rhs, uint8_t pattern_width, uint8_t pattern_start_point, uint8_t array_size );
void set_matrix_leds(uint8_t pwmset, uint8_t frame, uint8_t row_val, uint8_t clear, mouth_data *mouthPiece, uint8_t lhs_or_rhs );
void print_autogain_display( int16_t x, uint16_t y,uint16_t z );
//void make_sine_table(uint16_t trail_width, uint16_t table, uint8_t brightness);
void make_sine_table( mouth_data *mouthD, uint8_t brightness);
void button_0_func(); // button isr
void button_1_func(); // button isr

using namespace lr;
AS1130 ledDriver;


/***************
 * Globals
 * 
***************/

static volatile uint8_t adc_state = SAMPLE_ADC_DATA;             // either SAMPLE_ADC_DATA or PROCESS_ADC_DATA
static int buttonState = 0;         // variable for reading the pushbutton status

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

    /***************
     * Matrix
    ***************/
const uint8_t AllOnFrame[] = {
0b11111111, 0b11111111, 0b11111111,
0b11111111, 0b11111111, 0b11111111,
0b11111111, 0b11111111, 0b11111111,
0b11111111, 0b11111111, 0b11111111,
0b11111111, 0b11111111, 0b11111111};

const uint8_t MRFrame[] = {
0b00000000, 0b00000000, 0b00000000,
0b00000000, 0b10100000, 0b00000000,
0b00000000, 0b11101110, 0b00000000,
0b00000000, 0b10101000, 0b00000000,
0b00000000, 0b10101000, 0b00000000};

// writes the coffee on the mouth piece when talking
const uint8_t COFFEEFrame[] = {
0b11101110, 0b11101110, 0b11101110,
0b10001010, 0b10001000, 0b10001000,
0b10001010, 0b11101110, 0b11101110,
0b10001010, 0b10001000, 0b10001000,
0b11101110, 0b10001000, 0b11101110};

const uint8_t background_array[24] = {BACKGROUND_PWM_VAL};

    /***************
     * Matrix Gauss
    ***************/
    
const uint8_t CENTRE_OF_GAUSS_DIST = 0; // same as max gauss 

//const uint8_t gauss_dist_array[] = {0,0,0,0,0,0,0,0,0,3,34,154,255,154,34,3,0,0,0,0,0,0,0,0,0};  //uint8_t GAUSS_DIST_WIDTH  = 3;
const uint8_t gauss_dist_array[] = {0,0,0,0,0,0,0,0,0,3,34,154,255,255,255,255,255,255,255,255,255,255,255,255,255};
//uint8_t gauss_dist_array[] = {255,154,34,3,0,0,0,0,0,0,0,0,0};  //uint8_t GAUSS_DIST_WIDTH  = 3;
//uint8_t gauss_dist_array[] = {0,0,0,1,2,4,7,11,15,20,24,27,28,27,24,20,15,11,7,4,2,1,0,0,0}; // uint8_t GAUSS_DIST_WIDTH  = 9;


static  mouth_data mouth_gauss = 
  {
    .trail_start_point  = 12,
    .trail_width        = 3, //3, // width of bars excluding the peak value
    .array_length       = 24,    // loop length was smaller...!
    .trail_clear        = &background_array[0],
    .trail_data         = &gauss_dist_array[0],
  };

    /***************
     * Matrix SINE
    ***************/
static uint8_t sine_table[24] = {0};

static  mouth_data mouth_sine = 
  {
    .trail_start_point  = 12, //wher in array to start looping from, was 0
    .trail_width        = 6,     // width of bars excluding the peak value
    .array_length       = 24,    // loop length was smaller...!
    .trail_clear        = &background_array[0],
    .trail_data         = &sine_table[0],
  };

static uint8_t all_led_on_full[24] = {255};
static mouth_data *mouth = &mouth_gauss;
 

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
  make_sine_table( &mouth_sine, 255);
  
  /*  Set-up everything.
   *  Ram config 1, with 36 frames and 1 pwm set, we require 1 frame (all leds on) and 1 pwm set ( which we alter) 
   * 
  */  
  ledDriver.setRamConfiguration(AS1130::RamConfiguration3); //  ramconf 1 = 36 frames and 1 pwmset  // ram cofig 5 = 5 pwm sets and 12 frames

  ledDriver.setOnOffFrame24x5(0, MRFrame, 0); // writes the mr
  ledDriver.setOnOffFrame24x5(1, COFFEEFrame, 0); // writes the coffee on the mouth piece
  ledDriver.setOnOffFrame24x5(2, AllOnFrame, 1); // void setOnOffFrame24x5(uint8_t frameIndex, const uint8_t *data, uint8_t pwmSetIndex = 0);

  
  // Set-up a blink&PWM set with values for all LEDs.
  ledDriver.setBlinkAndPwmSetAll(0, false, 255); // void setBlinkAndPwmSetAll(uint8_t setIndex, bool doesBlink = false, uint8_t pwmValue = 0xff);
  ledDriver.setBlinkAndPwmSetAll(1, false, BACKGROUND_PWM_VAL); // void setBlinkAndPwmSetAll(uint8_t setIndex, bool doesBlink = false, uint8_t pwmValue = 0xff);
  
  ledDriver.setCurrentSource(AS1130::Current15mA);
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
    
    pinMode(BUTTON_PIN_1, INPUT_PULLUP);
    attachInterrupt(digitalPinToInterrupt(BUTTON_PIN_1), button_1_func, FALLING );
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
// isr for button press
  uint8_t button_0_count = 0;
  uint8_t button_1_count = 0;
  
  void button_0_func()
  {
    //alternate face type
    button_0_count++;
    button_0_count %= BUTTON_ZERO_MODES;
    

  }

// isr for button 
  void button_1_func()
  {
    button_1_count++;
    button_1_count %= BUTTON_ONE_MODES;
  }
    
#endif



void loop() 
{
  static uint16_t new_matrix_peak_val = 0;
  static uint16_t old_matrix_peak_val = 1;
  static uint8_t  max_map_val = 11; 
  static int16_t adcVals = {0};
    uint8_t frame=0;
    uint8_t pwmset = 0;
    uint8_t button_state = 0;
  

  

  /* Process Data */
  if( adc_state == PROCESS_ADC_DATA )
  {   
    uint16_t adc_val = analogRead(A0);
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
    adcVals = map( adc_val, lowest_dcoffset, highest_input_read , max_map_val, 0);   

    // no lowpass
    new_matrix_peak_val = adcVals;

    PRINT_MATRIXVALUE(new_matrix_peak_val);

    /*  */
    //if(new_matrix_peak_val != old_matrix_peak_val )
    {
        #ifndef PULSE_MOUTH
          static uint8_t trail_start_point = mouth->trail_start_point;
        #else
          static uint8_t trail_start_point = mouth->trail_start_point;
          trail_start_point = mouth->trail_start_point++;
        #endif
        
        //uint8_t trail_width   = mouth.trail_width;
        //uint8_t *trail_clear  = mouth.trail_clear;
        //uint8_t *trail_data   = mouth.trail_data;
        //uint8_t array_length  = mouth.array_length; // mod this 
      

      #ifdef PULSE_MOUTH
         //trail_start_point = mouth.trail_start_point++;
      #endif

      /* Send values to the Matrix */
      
      // set_matrix_leds( frame,  row_val,  set_clear,  *trail_pattern_array,  lhs_or_rhs,  pattern_trail_width,  pattern_start_point, array_size )
      // removed made change so that LHS and RHS have the same trail, but 
      // instead put in array_length for pulse mouth 
      // set clear not used
      // 
      


      // store button state
      button_state = button_1_count; // capture button state, don't turn off interrupts
      
      switch(button_state)
      {
        case 0: 

              Serial.println("MR COFFEE bold flash");
              if( old_matrix_peak_val >=11 ) // new_matrix_peak_val
              {
                //show MR
                frame = 0;            // 'mr' frame
                pwmset = 0;           // bold pwm set
                mouth = &mouth_sine;  // update mouth type
                ledDriver.setBlinkAndPwmSetAll(pwmset, false, 255); //all pwm values to max // void setBlinkAndPwmSetAll(uint8_t setIndex, bool doesBlink = false, uint8_t pwmValue = 0xff);
                
                ledDriver.startPicture(frame, false);               // draw picture
              }
              else
              {
                //show coffee 
                frame = 1;            // 'coffee' frame
                pwmset = 0;           // bold pwm set
                mouth = &mouth_sine;  // update mouth type
                ledDriver.setOnOffFrame24x5(frame, COFFEEFrame, pwmset);  // update frame with pwm set
                ledDriver.setBlinkAndPwmSetAll(pwmset, false, 255);       //all pwm values to max
                
                ledDriver.startPicture(frame, false);
              }
              break;
              
          case 1: 
          /* MR COFFEE wavy */
              Serial.println("MR COFFEE strobed");
              if( old_matrix_peak_val >=11 ) // new_matrix_peak_val
              {
                //show MR
                frame = 0;            // 'mr' frame
                pwmset = 0;           // bold pwm set
                mouth = &mouth_sine;  // update mouth type
                
                ledDriver.startPicture(frame, false);
              }
              else
              {
                //show coffee 
                frame = 1;            // 'coffee' frame
                pwmset = 1;           // wavy pwmset
                mouth = &mouth_gauss; // update mouth type
                ledDriver.setOnOffFrame24x5(frame, COFFEEFrame, pwmset);  // update frame with pwm set
                
                set_matrix_leds(pwmset, frame, old_matrix_peak_val, CLEAR,          mouth, LHS);
                set_matrix_leds(pwmset, frame, new_matrix_peak_val, WRITE_ARRAY,    mouth, LHS);
                set_matrix_leds(pwmset, frame, 24-old_matrix_peak_val, CLEAR,       mouth, RHS);
                set_matrix_leds(pwmset, frame, 24-new_matrix_peak_val, WRITE_ARRAY, mouth, RHS);
              }
              break;
               
        case 2: 
        /* knight rider */
              Serial.println("bars");
              frame = 2;              // 'all on' frame
              pwmset = 1;             // wavy pwmset
              mouth = &mouth_gauss;   // update mouth type
              ledDriver.setOnOffFrame24x5(frame, AllOnFrame, pwmset); // update frame with pwm set // void setOnOffFrame24x5(uint8_t frameIndex, const uint8_t *data, uint8_t pwmSetIndex = 0);
  
              set_matrix_leds(pwmset, frame, old_matrix_peak_val, CLEAR,          mouth, LHS);
              set_matrix_leds(pwmset, frame, new_matrix_peak_val, WRITE_ARRAY,    mouth, LHS);
              set_matrix_leds(pwmset, frame, 24-old_matrix_peak_val, CLEAR,       mouth, RHS);
              set_matrix_leds(pwmset, frame, 24-new_matrix_peak_val, WRITE_ARRAY, mouth, RHS);
              break;   
        default:
              Serial.println("default button_state");
              
      } // end of switch(button_state)
      
      old_matrix_peak_val = new_matrix_peak_val;
    } // end of update matrix

    /* reset state ready for next interrupt */
    adc_state = SAMPLE_ADC_DATA;
  }

} /* end of main loop */


void set_matrix_leds_bender(uint8_t pwmset, uint8_t new_row_val, uint8_t old_row_val, uint8_t set_clear, uint8_t *trail_pattern_array, uint8_t lhs_or_rhs )
{
  /*
uint8_t col=0;
uint8_t i=0;
  for(col=0; col<=4; col++)
  {
  // do teeth
    for(i=0; i<=pattern_width; i++)
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
  }
  
  
  //draw teeth
  
  */
}



void set_matrix_leds(uint8_t pwmset, uint8_t frame, uint8_t row_val, uint8_t clear, mouth_data *mouthPiece, uint8_t lhs_or_rhs)
{
  uint8_t col=0;
  uint8_t i=0;
  uint8_t lhsvalue = 0;
  uint8_t rhsvalue = 0;
  uint8_t point;

  MATRIX_DEBUG("Enter set_matrix_leds()");
  
  if(lhs_or_rhs == LHS)
  {
    MATRIX_DEBUG(",LHS");
    
    lhsvalue = 0;
    rhsvalue = 11;
  }
  else
  {
    MATRIX_DEBUG(",RHS");
    
    lhsvalue = 12; //MIDDLE_COLUMN_VAL;
    rhsvalue = 23; //LAST_COLUMN_VAL;
  }
  
  uint8_t mod = 0;
  uint8_t *data = NULL;
  mod = mouthPiece->array_length;
  
  if(clear)
  {
    //mod = mouthPiece->array_length; // 1;
    data = mouthPiece->trail_clear;
  }
  else
  {
    data = mouthPiece->trail_data;
  }

  for(col=0; col<=4; col++)
  {
    // centre of array is trail_pattern_array[ 12 ]
    // therefore row_val should be trail_pattern_array[12] 
    //
    
    // centre trail - bighest
    // (pwmset '0' , led index, vlaue)
    //ledDriver.setPwmValue(pwmset, ledDriver.getLedIndex24x5(row_val, col), trail_pattern_array[ pattern_start_point ] );
    MATRIX_DEBUG("LHS trail ");
    
    // LHS trail, if (row_val >= 1 ) ok LHS trail
    for(i=0; i<=mouthPiece->trail_width; i++)
    {
        if (row_val >= lhsvalue+i) 
        {
          // pattern start pointer must not go negative or greter than array size - pass in array size?? or have all same size?
          // this was [ pattern_start_point-i ] but 
          point = (mouthPiece->trail_start_point-i ) % mod;
          
          ledDriver.setPwmValue(pwmset, ledDriver.getLedIndex24x5(row_val-i, col), data[ point ] );
          //Serial.print("LHS data[ point ] = "); Serial.println(data[ point ]);
        }
    }
    MATRIX_DEBUG("");
    MATRIX_DEBUG("RHS trail ");
    // RHS trail, if (row_val <= 10) ok RHS trail
    for(i=1 ; i<=mouthPiece->trail_width; i++)
    {
    // 11 -1 = 10
    // 11 -2 = 9
        if (row_val <= (rhsvalue-i) )
        {
          point = (mouthPiece->trail_start_point+i ) % mod;
          ledDriver.setPwmValue(pwmset, ledDriver.getLedIndex24x5(row_val+i, col), data[ point ] );
          //Serial.print("RHS data[ point ] = "); Serial.println(data[ point ]);
        }
    }
    MATRIX_DEBUG("");
  }
  
  // #ifdef COFFEE_BACKPLATE
    // ledDriver.setOnOffFrame24x5(0, COFFEEFrame, 0); // writes the coffee on the mouth piece
  // #else
    // ledDriver.setOnOffFrame24x5(0, AllOnFrame, 0); // void setOnOffFrame24x5(uint8_t frameIndex, const uint8_t *data, uint8_t pwmSetIndex = 0);
  // #endif
  // MRFrame

  //write picture back
  ledDriver.startPicture(frame, false);
  
  
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
void make_sine_table( mouth_data *mouthD, uint8_t brightness)
{
  uint8_t i=0;
  uint8_t val =0;

  if( (mouthD->trail_width > 12) || (mouthD->trail_width < 2) )
  {
    //error
    mouthD->trail_width = 11;
  }
  
  for(i=0; i<=mouthD->trail_width; i++)
  {
    val = uint8_t ( (double)brightness * cos( i * ( (PI/2)/(float)mouthD->trail_width)) );
    Serial.print("val = \t");
    Serial.println(val);
    
    //RHS
    //tablet[12+i] = val; // not note start at 1
    mouthD->trail_data[12 + i] = val;
    
    //LHD
    //tablet[11 - i] = val;
    mouthD->trail_data[11-i] = val;
  }

  //
  Serial.print("\t mouthD->trail_width = \t"); Serial.println( mouthD->trail_width); 
  for(i=0; i<24; i++)
  {
    Serial.print("i = "); Serial.print(i); Serial.print("mouthD->trail_data[i] = "); Serial.println( mouthD->trail_data[i]); 
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



