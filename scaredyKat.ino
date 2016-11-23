/*
The idea is to run your sumobot slowly so that it does not leave the ring. 
It helps if it makes small circles in the ring. Once your bump detector hits another sumobot, 
your sumobot should push the other robot out with full power. When you build a sumobot, this is important.

code borrowed from http://www.robots-and-androids.com/Build-a-Sumobot.html
modified by James Dinsmore for the Melbourne MakerSpace SumoBot Competition August 2016
http://www.MelbourneMakerSpace.org
*/

#include <Servo.h> 

#define DEBUG 0

// Pin Assignments
#define PORT_PIN 9
#define STAR_PIN 10
#define TRACK_PIN 0
#define FRONT_PIN 1
#define GO_PIN 2

// Servo Tuning Values
#define PORT_STOP_VALUE 1480
#define STAR_STOP_VALUE 1435
#define SLOW_OFFSET 70
#define MED_OFFSET 100
#define FAST_OFFSET 800

// Sensor Tuning Values
#define FRONT_THRESHOLD 400
#define TRACK_THRESHOLD_LOW 50
#define TRACK_THRESHOLD_HI 800

// Directional Enumeration
#define PORT 0
#define STARBOARD 1

Servo port;
Servo star;
// Volatile since they can change in the ISR.
volatile boolean bGo;
volatile boolean bWait;
volatile boolean bLed;

void halt() {
  port.writeMicroseconds(PORT_STOP_VALUE);
  star.writeMicroseconds(STAR_STOP_VALUE);
}

void forward(int offset) {
  if (!bWait) {
    port.writeMicroseconds(PORT_STOP_VALUE + offset);
    star.writeMicroseconds(STAR_STOP_VALUE - offset);
  }
}

void backward(int offset) {
  if (!bWait) {
    port.writeMicroseconds(PORT_STOP_VALUE - offset);
    star.writeMicroseconds(STAR_STOP_VALUE + offset);
  }
}

void turn(int side) {
  if (!bWait) {
    if (PORT == side) {
      port.writeMicroseconds(PORT_STOP_VALUE);
      star.writeMicroseconds(STAR_STOP_VALUE - SLOW_OFFSET);    
    }
    else {    
      port.writeMicroseconds(PORT_STOP_VALUE + SLOW_OFFSET);
      star.writeMicroseconds(STAR_STOP_VALUE);
    }
  }
}
  
void spin(int side) {
  if (!bWait) {
    if (PORT == side) {
      port.writeMicroseconds(PORT_STOP_VALUE - SLOW_OFFSET);
      star.writeMicroseconds(STAR_STOP_VALUE - SLOW_OFFSET);    
    }
    else {    
      port.writeMicroseconds(PORT_STOP_VALUE + SLOW_OFFSET);
      star.writeMicroseconds(STAR_STOP_VALUE + SLOW_OFFSET);
    }
  }
}

void ledDelay(int onDelay = 0, int offDelay = 0) {
  if (0 < onDelay) {
    digitalWrite(LED_BUILTIN, HIGH);
    delay(onDelay);
  }
  // We will ALWAYS turn the LED off.
  digitalWrite(LED_BUILTIN, LOW);
  if (0 < offDelay) {
    delay(offDelay);
  }
}

// The ISR for whan the user presses the button.
void goIsr() {
  static unsigned long ulLastEntry = 0;
  unsigned long ulThisEntry = millis();
  // 200 ms debounce test.
  if (200 < (ulThisEntry - ulLastEntry)) {
    bGo = !bGo;
    if (!bGo) {
      halt(); // Halt immediately upon press if running.
      bWait = true;
      digitalWrite(LED_BUILTIN, LOW); // Turn the LED off.
    }
  }
  ulLastEntry = ulThisEntry;
}

// ISR to flash the LED rapidly if we are not waiting.
SIGNAL(TIMER0_COMPA_vect) {
  static unsigned long ulLastEvent = 0;
  unsigned long ulThisEntry = millis();
  if (bGo && !bWait) {
    if (100 < (ulThisEntry - ulLastEvent)) {
      // Toggle the LED
      bLed = !bLed;
      digitalWrite(LED_BUILTIN, (bLed ? HIGH : LOW));
      ulLastEvent = ulThisEntry;
    }
  }
}

#if (DEBUG != 0)
void deadpoint_detect() {
  for (int i = 1300; i <= 1700; i += 5) {
    port.writeMicroseconds(i);
    Serial.println(i);
    delay(1000); 
  }
}
#endif

void setup() {
  bGo = false;
  bWait = true;
  bLed = false;
  pinMode(GO_PIN, INPUT_PULLUP);
  pinMode(LED_BUILTIN, OUTPUT);
  digitalWrite(LED_BUILTIN, LOW);
#if (DEBUG != 0)
  Serial.begin(9600);
#endif
  // Begin with servos stopped.
  halt();
  // Attach the servo pins.
  port.attach(PORT_PIN);
  star.attach(STAR_PIN);
  // Attach the button interrupt.
  attachInterrupt(digitalPinToInterrupt(GO_PIN), goIsr, FALLING);
  // Timer0 is already used for millis(). Just interrupt somewhere
  // in the middle and call the SIGNAL function.
  OCR0A = 0x74;
  TIMSK0 |= _BV(OCIE0A);
} 
 
void loop() {
  while (!bGo) {
    // Wait a short time to see if the button gets pressed.
    delay(10);
  }
  if (bWait) {
    // Flash LED during 5 second start up delay.
    for (int i = 0; bGo && (i < 5); i++) {
      ledDelay(500, 500);
    }
    if (!bGo) {
      // In case the user cancelled.
      return;
    }
    bWait = false; // Signal that we are running.
  }
  // Read edge of track sensor (facing down).
  int trackSensor = analogRead(TRACK_PIN);
  delay(1);
  // Read front proximity sensor (facing forward).
  int frontSensor = analogRead(FRONT_PIN);
  delay(1);
  
#if (DEBUG != 0)
//  Serial.print(frontSensor);
//  Serial.print("\t");
//  Serial.println(trackSensor);
//  delay(500);
  deadpoint_detect();
#else

  if (trackSensor < TRACK_THRESHOLD_HI) {
    // On the mat, go for a bit.
    forward(MED_OFFSET);
    delay (1);           // Run the motors for a bit.
  }    
  else {
    // At the edge, stop, back, and turn.
    halt();
    delay(10);            // Pause for a bit.
    backward(SLOW_OFFSET);
    delay(200);          // Keep backing up for a bit.
    // turn a little and proceed
    spin(STARBOARD);
    delay(300);          // Turn for a bit, 400 == 180. 
  }
  
  // See if our opponent is in front of us.
  while (frontSensor > FRONT_THRESHOLD && trackSensor < TRACK_THRESHOLD_LOW) {
    // Somehting is in front of us, FULL SPEED AHEAD!
    forward(FAST_OFFSET);
    delay (1);           // Run the motors for a bit.
    // Read sensors.
    trackSensor = analogRead(TRACK_PIN);
    delay(1);
    frontSensor = analogRead(FRONT_PIN);
    delay(1);
  }

#endif
}

