// HARDARE  -> PINS (Arduino Uno / pro trinket )
// ================

//  Analogue:
//  ========
// sck -> arduino a0 to mic opamp
// sck -> arduino a4
// sda -> arduino a5   

//  Digital:
//  =======
// pin output to matrix reset -> arduino pin d4
// button interrupt_0   d2 // -> there is no D2 on protrinket!! just uno so use 'pin-change interrupt' see http://playground.arduino.cc/Main/PinChangeInterrupt
// button interrupt_1   d3
// mosfet to clear peak detector -> arduino d7 - now removed
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
 * Basically  a Foreground/background architecture
 * 
 * Uses a timer ISR to trigger adc read. ( ISR_FREQ_HZ )
 * 
 * this value is mapped to a value 0-11,
 * this value is 
 * 
 * adc_state .. SAMPLE_ADC_DATA | PROCESS_ADC_DATA 
 * 
 * 
*/



#include "LRAS1130.h"
#include "coffee_define.h"

/***************
 * Prototypes
 * 
***************/
void set_matrix_leds(uint8_t pwmset, uint8_t frame, uint8_t row_val, uint8_t clear, mouth_data *mouthPiece, uint8_t lhs_or_rhs );
void set_matrix_leds_bender(uint8_t pwmset, uint8_t frame, uint8_t row_val, uint8_t clear, mouth_data *mouthPiece);

void print_autogain_display( int16_t x, uint16_t y,uint16_t z );
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
const uint8_t gauss_dist_array[] =  {0,0,0,0,0,0,0,0,0,3,34,154,255,154,34,3,0,0,0,0,0,0,0,0,0};  // .trail_width = 3
//uint8_t gauss_dist_array[] = {0,0,0,1,2,4,7,11,15,20,24,27,28,27,24,20,15,11,7,4,2,1,0,0,0};    // .trail_width = 9


static  mouth_data mouth_gauss = 
  {
    .trail_start_point  = 12,// peak brightness
    .trail_width        = 3, // width of bars excluding the peak value
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

  make_sine_table( &mouth_sine, 255);
  
  /*  Set-up everything.
   *  Ram config 1, with 36 frames and 1 pwm set, we require 1 frame (all leds on) and 1 pwm set ( which we alter) 
  */  
  ledDriver.setRamConfiguration(AS1130::RamConfiguration3); //  ramconf 1 = 36 frames and 1 pwmset  // ram cofig 5 = 5 pwm sets and 12 frames

  ledDriver.setOnOffFrame24x5(0, MRFrame, 0); // writes the mr
  ledDriver.setOnOffFrame24x5(1, COFFEEFrame, 0); // writes the coffee on the mouth piece
  ledDriver.setOnOffFrame24x5(2, AllOnFrame, 1); // void setOnOffFrame24x5(uint8_t frameIndex, const uint8_t *data, uint8_t pwmSetIndex = 0);

  
  // Set-up a blink&PWM set with values for all LEDs.
  ledDriver.setBlinkAndPwmSetAll(0, false, 255); // void setBlinkAndPwmSetAll(uint8_t setIndex, bool doesBlink = false, uint8_t pwmValue = 0xff);
  ledDriver.setBlinkAndPwmSetAll(1, false, BACKGROUND_PWM_VAL); // void setBlinkAndPwmSetAll(uint8_t setIndex, bool doesBlink = false, uint8_t pwmValue = 0xff);
  
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

  // button interrupts
  pinMode(BUTTON_PIN_0, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(BUTTON_PIN_0), button_0_func, FALLING );
  
  pinMode(BUTTON_PIN_1, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(BUTTON_PIN_1), button_1_func, FALLING );

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

uint8_t button_0_count = 0;
uint8_t button_1_count = 0;
  
// isr for button press, changes brightness of leds
void button_0_func()
{
  button_0_count++;
  button_0_count %= BUTTON_ZERO_MODES;
}

// isr for button 1 - changes the face type
void button_1_func()
{
  button_1_count++;
  button_1_count %= BUTTON_ONE_MODES;
}


void loop() 
{
  static uint16_t new_matrix_peak_val = 0;
  static uint16_t old_matrix_peak_val = 1;
  static uint8_t  max_map_val = 11; 
  static int16_t adcVals = {0};
  uint8_t frame=0;
  uint8_t pwmset = 0;
  uint8_t button_face_state = 0;
  uint8_t button_brightness_state = 0;
  
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
    
    //optional serial print out, set DO_PRINT_AUTOGAIN_DISPLAY
    print_autogain_display( adc_val, lowest_dcoffset, highest_input_read ); 

    // capture button state, don't turn off interrupts, because of i2c 
    button_face_state = button_1_count; 
    
    // map to range
    if(button_face_state != 4)
    {
      // not bender mouth
      adcVals = map( adc_val, lowest_dcoffset, highest_input_read , max_map_val, 0);   
    }
    else
    {
      //bender mouth
      adcVals = map( adc_val, lowest_dcoffset, highest_input_read , 0, 2);  
    }
    
    // capture button state, 
    button_brightness_state = button_0_count;
    
    // set brightness from brightness button
    ledDriver.setCurrentSource(button_brightness_state);
      
    // no lowpass
    new_matrix_peak_val = adcVals;

    PRINT_MATRIXVALUE(new_matrix_peak_val);

    /*  */
    //if(new_matrix_peak_val != old_matrix_peak_val )
    {
      /* Send values to the Matrix 
        depends on button_face_state (1)
        0) Serial.println("MR COFFEE bold flash");
        1) Serial.println("MR COFFEE strobed");
        2) Serial.println("gauss sweeper");
        3) Serial.println("bars");
        4) Serial.println("bender");
        5)
      */    

      
      switch(button_face_state)
      {
        case 0: 
        /* MR COFFEE BOLD FLASH */
              //Serial.println("MR COFFEE bold flash");
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
              //Serial.println("MR COFFEE wavy");             
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
        /* gauss inwards to centre sweep */
              //Serial.println("gauss sweeper");              
              frame = 2;              // 'all on' frame
              pwmset = 1;             // wavy pwmset
              mouth = &mouth_gauss;   // update mouth type
              ledDriver.setOnOffFrame24x5(frame, AllOnFrame, pwmset); // update frame with pwm set // void setOnOffFrame24x5(uint8_t frameIndex, const uint8_t *data, uint8_t pwmSetIndex = 0);
  
              static uint8_t cnt =0;
              cnt++;
              cnt %= 12;
              static uint8_t offset = 12;
              offset += cnt;
              offset %= 24;
  
              set_matrix_leds(pwmset, frame, cnt-1, CLEAR,          mouth, LHS);
              set_matrix_leds(pwmset, frame, cnt, WRITE_ARRAY,      mouth, LHS);
              set_matrix_leds(pwmset, frame, 24-cnt-1, CLEAR,       mouth, RHS);
              set_matrix_leds(pwmset, frame, 24-cnt, WRITE_ARRAY, mouth, RHS);
              break;  
        case 3:
        /* knight rider */
              //Serial.println("bars");        
              frame = 2;              // 'all on' frame
              pwmset = 1;             // wavy pwmset
              mouth = &mouth_gauss;   // update mouth type
              ledDriver.setOnOffFrame24x5(frame, AllOnFrame, pwmset); // update frame with pwm set // void setOnOffFrame24x5(uint8_t frameIndex, const uint8_t *data, uint8_t pwmSetIndex = 0);
              
              set_matrix_leds(pwmset, frame, old_matrix_peak_val, CLEAR,          mouth, LHS);
              set_matrix_leds(pwmset, frame, new_matrix_peak_val, WRITE_ARRAY,    mouth, LHS);
              set_matrix_leds(pwmset, frame, 24-old_matrix_peak_val, CLEAR,       mouth, RHS);
              set_matrix_leds(pwmset, frame, 24-new_matrix_peak_val, WRITE_ARRAY, mouth, RHS);
              break;  
        case 4:
        /* Bender */
              //Serial.println("bender");              
              frame = 2;              // 'all on' frame
              pwmset = 1;             // wavy pwmset
              mouth = &mouth_gauss;   // update mouth type
              ledDriver.setOnOffFrame24x5(frame, AllOnFrame, pwmset); // update frame with pwm set // void setOnOffFrame24x5(uint8_t frameIndex, const uint8_t *data, uint8_t pwmSetIndex = 0);
              //ledDriver.setBlinkAndPwmSetAll(pwmset, false, BACKGROUND_PWM_VAL);//reset all leds to off no too flashy on display bad effect
              set_matrix_leds_bender(pwmset, frame, old_matrix_peak_val, CLEAR,       mouth);
              set_matrix_leds_bender(pwmset, frame, new_matrix_peak_val, WRITE_ARRAY, mouth);
              break;  

        default:
              Serial.println("default button_face_state");
              
      } // end of switch(button_face_state)
      
      //capture old value which will be erased next time
      old_matrix_peak_val = new_matrix_peak_val;
      
    } // end of update matrix

    /* reset state ready for next interrupt */
    adc_state = SAMPLE_ADC_DATA;
  }

} /* end of main loop */

/*
* function to make benders face
*
*/
void set_matrix_leds_bender(uint8_t pwmset, uint8_t frame, uint8_t row_val, uint8_t clear, mouth_data *mouthPiece)
{
  uint8_t col=0;
  uint8_t i=0;
  uint8_t point =12;
  uint8_t *data = NULL;
  //Serial.print("\t row_val=\t"); Serial.println(row_val);
 
  if(clear)
  {
    data = mouthPiece->trail_clear;
  }
  else
  {
    data = mouthPiece->trail_data;
  }

  for(col=5; col<24; col +=5 )
  {
    // do teeth
    //Serial.print("col=\t"); Serial.print(col); 
    for(i=0; i<5; i++)
    {
      ledDriver.setPwmValue(pwmset, ledDriver.getLedIndex24x5(col,i), data[point]);
    }
  }
        
  switch(row_val)
  {
    case 0: 
    // mouth closed
        for(i=0; i<24; i++)
        {
          // middle lip
          ledDriver.setPwmValue(pwmset, ledDriver.getLedIndex24x5(i,row_val+2), data[point]);
        }
        break;
        
    case 1: 
    // mouth mid open
        for(i=0; i<24; i++)
        {
          // top lip
          ledDriver.setPwmValue(pwmset, ledDriver.getLedIndex24x5(i,2-row_val), data[point]);
        }
        
        for(i=0; i<24; i++)
        {
          //bottom lip
          ledDriver.setPwmValue(pwmset, ledDriver.getLedIndex24x5(i,row_val+2), data[point]);
        }
        break;
        
    case 2: 
    // mouth open
        for(i=0; i<24; i++)
        {
          // top lip
          ledDriver.setPwmValue(pwmset, ledDriver.getLedIndex24x5(i,2-row_val), data[point]);
        }
        
        for(i=0; i<24; i++)
        {
          //bottom lip
          ledDriver.setPwmValue(pwmset, ledDriver.getLedIndex24x5(i,row_val+2), data[point]);
        }
        break;

  }
    //Serial.println();  
}


/*
* function to light up matrix
*
*/
void set_matrix_leds(uint8_t pwmset, uint8_t frame, uint8_t row_val, uint8_t clear, mouth_data *mouthPiece, uint8_t lhs_or_rhs)
{
  /*
   * Error if row_val goes higher than 11/12 still prints the LHS trail
   * Error if row_val goes higher than 23 RHS trail goes over 9/10/11 
  */
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
//Serial.print("col=\t"); Serial.print(col); Serial.print("\t row_val=\t"); Serial.println(row_val);
    // centre of array is trail_pattern_array[ 12 ]
    // therefore row_val should be trail_pattern_array[12] 
    //
    
    // centre trail - bighest
    // (pwmset '0' , led index, vlaue)
    //ledDriver.setPwmValue(pwmset, ledDriver.getLedIndex24x5(row_val, col), trail_pattern_array[ pattern_start_point ] );
    MATRIX_DEBUG("LHS trail ");
    
    // LHS trail and peak value, if (row_val >= 1 ) ok LHS trail
    for(i=0; i<=mouthPiece->trail_width; i++)
    {
        if (row_val >= lhsvalue+i) 
        {
          // pattern start pointer must not go negative or greter than array size - pass in array size?? or have all same size?
          // this was [ pattern_start_point-i ] but 
          point = (mouthPiece->trail_start_point-i ) % mod;
          
          ledDriver.setPwmValue(pwmset, ledDriver.getLedIndex24x5(row_val-i, col), data[ point ] );
//Serial.print(" LHS data[ "); Serial.print(point);  Serial.print(" ] = "); Serial.print(data[ point ]); Serial.print("\ti="); Serial.println(i);

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
//Serial.print(" RHS data[ "); Serial.print(point);  Serial.print(" ] = "); Serial.print(data[ point ]); Serial.print("\ti="); Serial.println(i);
        }
    }
    MATRIX_DEBUG("");
  }
  
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
    mouthD->trail_data[12 + i] = val;
    
    //LHD
    mouthD->trail_data[11-i] = val;
  }

  //
  Serial.print("\t mouthD->trail_width = \t"); Serial.println( mouthD->trail_width); 
  for(i=0; i<24; i++)
  {
    Serial.print("i = "); Serial.print(i); Serial.print("mouthD->trail_data[i] = "); Serial.println( mouthD->trail_data[i]); 
  }
  Serial.println("");
}



void print_autogain_display( int16_t x, uint16_t y,uint16_t z) 
{
  #ifdef DO_PRINT_AUTOGAIN_DISPLAY
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
  
  #endif
}




