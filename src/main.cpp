#include <Arduino.h>
#include <RFM69OOK.h>
#include <RFM69OOKregisters.h>

#define DEFAULT_OOK_FREQ 433.9
#define MAX_PROTOCOL_SIZE 500 
#define BUFF_SIZE (MAX_PROTOCOL_SIZE + 2)


//RFM69 - OOK Config
static RFM69OOK radio(9);
float ookFreq;

//RF Scanner Config
static unsigned int timings[BUFF_SIZE];
static unsigned int pos = 0;
static unsigned long lastTime = 0;

//interruptPin Variables
static int receiverPin = 3;
static int interruptPin = 0;

int repeats = 0;     // long sync timing repeats
int changeCount = 0; //interruptPin change counter

//RF Scanner Func Prototypes 
void scanner_begin();
void scanner_loop();


//Init RFM69 Module
void rfm69_begin();

//Interrupt Func Prototype
void handleInterrupt();

void setup() {
  Serial.begin(9600);
  rfm69_begin();
  scanner_begin();
  Serial.println("Setup Finished");
}

void loop() {
  scanner_loop();
}


void rfm69_begin()
{
  ookFreq = DEFAULT_OOK_FREQ;
  radio.initialize();
  radio.setBandwidth(OOK_BW_250_0);
  radio.setFixedThreshold(30);
  radio.setFrequencyMHz(ookFreq);
  radio.setBitrate(2000);
  radio.receiveBegin();
}

void scanner_begin()
{
  interruptPin = digitalPinToInterrupt(receiverPin);
  attachInterrupt(interruptPin, handleInterrupt, CHANGE);
}

void scanner_loop()
{
  for (int i = 5; i>0; i--) {
    Serial.print(i);
    Serial.print("... ");
    delay(900);
    digitalWrite(13, HIGH);
    delay(100);
    digitalWrite(13, LOW);
  }
  Serial.println();
    
  detachInterrupt(interruptPin);

  int finalstate = digitalRead(receiverPin);
  
  for (unsigned int i = pos + finalstate; i < BUFF_SIZE; i++) {
    Serial.print( timings[i] );
    Serial.print(",");
  }

  for (unsigned int i = 0; i < pos; i++) {
    Serial.print( timings[i] );
    Serial.print(",");
  }

  Serial.println("");
  Serial.println("Reset your Arduino to scan again...");

  while(true) {} 
}

void handleInterrupt() {
  const long time = micros();
  timings[pos] = time - lastTime;
  lastTime = time;
  if (++pos > BUFF_SIZE - 1) {
    pos = 0;
  }
}
