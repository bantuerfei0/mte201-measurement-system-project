/*
 * MTE 201 Measurement System Project Code
 * 10/31/2024
 * Code mostly taken from https://github.com/mo-thunderz/RotaryEncoder and https://docs.arduino.cc/learn/electronics/lcd-displays/
 */

// include the library code:
#include <LiquidCrystal.h>

// initialize the library by associating any needed LCD interface pin
// with the arduino pin number it is connected to
const int rs = 12, en = 11, d4 = 5, d5 = 4, d6 = 3, d7 = 2;
LiquidCrystal lcd(rs, en, d4, d5, d6, d7);

// macro definition for the rotatary encoder, see: https://components101.com/sites/default/files/component_datasheet/KY-04-Rotary-Encoder-Datasheet.pdf
// TODO: set to correct values
#define CLK_PIN 2 // "A" pin A ideally can handle an interrupt
#define DT_PIN 3 // "B" POSSIBLY REMOVE. REDUNDANT
#define SW_PIN 4 // the built in switch

// volatile unsigned int foo; <= see: https://gammon.com.au/interrupts

unsigned long _lastIncReadTime = micros(); 
unsigned long _lastDecReadTime = micros(); 
int _pauseLength = 25000;
int _fastIncrement = 1; // We actually don't need this functionality - remove

volatile int counter = 0;

void setup() {
  //Serial.begin(9600); // TODO: set to correct baudrate
  // encoder pin definition
  pinMode(CLK_PIN, INPUT_PULLUP);
  pinMode(DT_PIN, INPUT_PULLUP);
  pinMode(SW_PIN, INPUT_PULLUP);
  // interrupts the microcontroller and tells it to run a function on the rising edge of the CLK_PIN
  // using this so we don't miss any cycles that the rotary encoder is outputting
  // see: https://docs.arduino.cc/language-reference/en/functions/external-interrupts/attachInterrupt/
  // RISING is a built-in constant. see above
  attachInterrupt(digitalPinToInterrupt(CLK_PIN), handlePulse, CHANGE);
  attachInterrupt(digitalPinToInterrupt(DT_PIN), handlePulse, CHANGE);

  // Start the serial monitor to show output
  Serial.begin(115200);

  // set up the LCD's number of columns and rows:
  lcd.begin(16, 2);
  // Print a message to the LCD.
  //lcd.print("hello, world!");

}

void loop() {
    static int lastCounter = 0;

  // set the cursor to column 0, line 1
  // (note: line 1 is the second row, since counting begins with 0):
  lcd.setCursor(0, 1);

  // If count has changed print the new value to serial
  if(counter != lastCounter){
    //Serial.println(counter);
    // print the the counter
    lcd.print(counter);
    lastCounter = counter;
  }

  // If button on encoder (SW_PIN) is pressed, reset
  if(digitalRead(SW_PIN)){
    counter = 0;
  }
}

/*
 * ISR to handle a pulse from the rotary encoder
 */
void handlePulse() {
  // TODO: impl
  // Taken from https://github.com/mo-thunderz/RotaryEncoder
  // Encoder interrupt routine for both pins. Updates counter
  // if they are valid and have rotated a full indent
 
  static uint8_t old_AB = 3;  // Lookup table index
  static int8_t encval = 0;   // Encoder value  
  static const int8_t enc_states[]  = {0,-1,1,0,1,0,0,-1,-1,0,0,1,0,1,-1,0}; // Lookup table

  old_AB <<=2;  // Remember previous state

  if (digitalRead(CLK_PIN)) old_AB |= 0x02; // Add current state of pin A
  if (digitalRead(DT_PIN)) old_AB |= 0x01; // Add current state of pin B
  
  encval += enc_states[( old_AB & 0x0f )];

  // Update counter if encoder has rotated a full indent, that is at least 4 steps
  if( encval > 3 ) {        // Four steps forward
    int changevalue = 1;
    if((micros() - _lastIncReadTime) < _pauseLength) {
      changevalue = _fastIncrement * changevalue; 
    }
    _lastIncReadTime = micros();
    counter = counter + changevalue;              // Update counter
    encval = 0;
  }
  else if( encval < -3 ) {        // Four steps backward
    int changevalue = -1;
    if((micros() - _lastDecReadTime) < _pauseLength) {
      changevalue = _fastIncrement * changevalue; 
    }
    _lastDecReadTime = micros();
    counter = counter + changevalue;              // Update counter
    encval = 0;
  }

}
