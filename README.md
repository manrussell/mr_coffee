# MR COFFEE

Project that uses a I2C controled led array to make a faces for a D and D robot costume.
It's bascially a foreground/background loop

## Getting Started

Install Arduino IDE.

Install the LRSA1130 driver into the Ardunio IDE from the .zip file.

  Harware  -> Pins (Arduino Uno / pro trinket )
  ================

  Analogue:

  sck -> arduino a0 to mic opamp.
  
  sck -> arduino a4.
  
  sda -> arduino a5 .  

  Digital:

  pin output to matrix reset -> arduino pin d4.
  
  button interrupt_0   d2 // -> there is no D2 on protrinket!! just uno so use 'pin-change interrupt' see http://playground.arduino.cc/Main/PinChangeInterrupt.
  
  button interrupt_1   d3.
  
  power -> arduino 5V to electret mic circuit.
  
  power -> arduino 3.3V to matrix.
  
  ground -> mic and to matrix.
  
  For the rest of the circuit see blog post https://wordpress.com/post/diyelectronicsandmusic


## Running the tests

To test the I2C matrix led's run the driver test 'led test' 


## License

This project is licensed under the MIT License - see the [LICENSE.md](LICENSE.md) file for details

## Acknowledgments

Thanks to Boldport for the hardare        https://www.boldport.com/

Thanks to lucky resistor for the driver   https://luckyresistor.me/  https://github.com/LuckyResistor/LRAS1130
