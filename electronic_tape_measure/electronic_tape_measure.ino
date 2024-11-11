/*
 * MTE 201 Measurement System Project Code
 * 10/31/2024
 * Code snippets from https://github.com/mo-thunderz/RotaryEncoder
 */

// include the library code:
#include <LiquidCrystal.h>

// initialize the library by associating any needed LCD interface pin
// with the arduino pin number it is connected to
#define RS_PIN 5
#define EN_PIN 6
#define D4_PIN 7
#define D5_PIN 8
#define D6_PIN 9
#define D7_PIN 10

LiquidCrystal lcd(RS_PIN, EN_PIN, D4_PIN, D5_PIN, D6_PIN, D7_PIN);

// macro definition for the rotatary encoder, see: https://components101.com/sites/default/files/component_datasheet/KY-04-Rotary-Encoder-Datasheet.pdf
// TODO: set to correct values
#define CLK_PIN 2  // "A" pin A ideally can handle an interrupt
#define DT_PIN 3   // "B" POSSIBLY REMOVE. REDUNDANT
#define SW_PIN 4   // the built in switch

// volatile unsigned int foo; <= see: https://gammon.com.au/interrupts

volatile uint16_t pulse_count = 0;
volatile bool update_lcd = true;

void setup() {
  Serial.begin(115200);
  lcd.begin(16, 2);  // begin LCD object with (columns, rows)
  lcd.clear();       // clear the lcd
  // encoder pin definition
  pinMode(CLK_PIN, INPUT_PULLUP);
  pinMode(DT_PIN, INPUT_PULLUP);
  pinMode(SW_PIN, INPUT_PULLUP);
  // interrupts the microcontroller and tells it to run a function on the changing edge of the CLK_PIN
  // using this so we don't miss any cycles that the rotary encoder is outputting
  // see: https://docs.arduino.cc/language-reference/en/functions/external-interrupts/attachInterrupt/
  // CHANGE is a built-in constant. see above
  attachInterrupt(digitalPinToInterrupt(CLK_PIN), handlePulse, CHANGE);
  attachInterrupt(digitalPinToInterrupt(DT_PIN), handlePulse, CHANGE);
}

void loop() {
  static uint8_t prev_button_state = HIGH;
  // If count has changed print the new value to serial
  if (update_lcd) {
    update_lcd = false;
    //Serial.println(counter);
    // print the the counter
    lcd.clear();
    lcd.print(pulse_count);
    lcd.setCursor(0, 2);
    lcd.print("d = ");
    lcd.print((pulse_count * 0.4265) - 0.2487);
    lcd.print(" mm");
    Serial.println(pulse_count);
  }
  // If button on encoder (SW_PIN) is pressed, reset
  int button_state = digitalRead(SW_PIN);
  //Serial.println(zbutton_state);
  if (button_state != prev_button_state) {
    if (button_state == LOW) {
      pulse_count = 0;
      update_lcd = true;
      Serial.println("RESET");
    }
    // ignore case for LOW button
  }
  prev_button_state = button_state;
}

/*
 * ISR to handle a pulse from the rotary encoder
 */
void handlePulse() {
  // Taken from https://github.com/mo-thunderz/RotaryEncoder
  // Encoder interrupt routine for both pins. Updates counter
  // if they are valid and have rotated a full indent
  static uint8_t old_AB = 3;                                                                  // Lookup table index
  static int8_t encval = 0;                                                                   // Encoder value
  static const int8_t enc_states[] = { 0, -1, 1, 0, 1, 0, 0, -1, -1, 0, 0, 1, 0, 1, -1, 0 };  // Lookup table
  old_AB <<= 2;                                                                               // Remember previous state
  if (digitalRead(CLK_PIN)) old_AB |= 0x02;                                                   // Add current state of pin A
  if (digitalRead(DT_PIN)) old_AB |= 0x01;                                                    // Add current state of pin B
  encval += enc_states[(old_AB & 0x0f)];
  // Update counter if encoder has rotated a full indent, that is at least 4 steps
  if (encval > 3) {
    // four steps forward
    pulse_count++;  // increment pulse_count
    update_lcd = true;
    encval = 0;
  } else if (encval < -3) {
    // four steps backwards
    if (pulse_count > 0) {
      pulse_count--;
      update_lcd = true;
    }
    encval = 0;
  }
  // any other case is invalid, best decision is said to ignore this
}
