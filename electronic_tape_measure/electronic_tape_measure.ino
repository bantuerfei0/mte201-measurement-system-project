/*
 * MTE 201 Measurement System Project Code
 * 10/31/2024
 */

// macro definition for the rotatary encoder, see: https://components101.com/sites/default/files/component_datasheet/KY-04-Rotary-Encoder-Datasheet.pdf
// TODO: set to correct values
#define CLK_PIN 0 // "A" pin A ideally can handle an interrupt
#define DT_PIN 1 // "B" POSSIBLY REMOVE. REDUNDANT
#define SW_PIN 2 // the built in switch

// volatile unsigned int foo; <= see: https://gammon.com.au/interrupts

void setup() {
  Serial.begin(9600); // TODO: set to correct baudrate
  // encoder pin definition
  pinMode(CLK_PIN, INPUT);
  pinMode(DT_PIN, INPUT);
  pinMode(SW_PIN, INPUT);
  // interrupts the microcontroller and tells it to run a function on the rising edge of the CLK_PIN
  // using this so we don't miss any cycles that the rotary encoder is outputting
  // see: https://docs.arduino.cc/language-reference/en/functions/external-interrupts/attachInterrupt/
  // RISING is a built-in constant. see above
  attachInterrupt(digitalPinToInterrupt(CLK_PIN), handlePulse, RISING);
}

void loop() {
}

/*
 * ISR to handle a pulse from the rotary encoder
 */
void handlePulse() {
  // TODO: impl
}
