

//LCD
//A4 A5 to SDA SLC
//Power 5v + GND

//DAC
//SPI configurations
#define PIN_MISO 4
#define PIN_CS   10
#define PIN_SCK  6
#define PIN_MOSI 7
//#define SPI_PORT spi0

#include <Wire.h> 
#include <SPI.h>
#include "LiquidCrystal_I2C.h"

//for DAC
//#include "MCP48xx.h"

//currently 30 hz max with TIMERSPEED 2000 and stepMax 1000
#define TIMERSPEED 2500 //this matches well with 99 in timer calc and reading still works ok.


LiquidCrystal_I2C lcd(0x27,20,4);  // set the LCD address to 0x27 for a 16 chars and 2 line display
// Define the MCP4822 instance, giving it the SS (Slave Select) pin
// The constructor will also initialize the SPI library
// We can also define a MCP4812 or MCP4802


// Define the MCP4822 instance, giving it the SS (Slave Select) pin
// The constructor will also initialize the SPI library
// We can also define a MCP4812 or MCP4802
//MCP4822 dac(PIN_CS);
// We define an int variable to store the voltage in mV so 100mV = 0.1V
int voltage = 100;


////
//int F = 2;                                                   //frequency of the signal
#define SINE_WAVE_TABLE_LEN 512
//int Fs = 500;                                                //sampling frequency
//int n = 500;                                                 //number of samples
//float t;                                                     //Time instance
//int sampling_interval;
int16_t sine_wave_table[SINE_WAVE_TABLE_LEN];                                           // to store the samples
//////

float stepA = 0.05;     //jumps of 32 through the sine table... 0.01 is Very smooth byt 5hz - 0.05 is ok at 25hz - 0.1 is dotty but 50Hz
float stepB = 0.1;     //jumps of 32 through the sine table...
uint16_t volume = 2048-1;
float angleA = 0;
float angleB = 0;
#define TWOPI PI*2

void setupDac()
{
    Serial.println("SetupDac");
    pinMode(PIN_CS, OUTPUT);
    digitalWrite(PIN_CS, HIGH);
    SPI.begin();

    
    //noInterrupts();
    /*
    while (true) {
      uint16_t sampleA = volume * cosf(angleA) + 2048; //2048 to make midpoint
      uint16_t sampleB = volume * sinf(angleB) + 2048; //2048 to make midpoint

      angleA += stepA;
      angleB += stepB;
      if (angleA > TWOPI) angleA -= TWOPI;
      if (angleB > TWOPI) angleB -= TWOPI;

      dac_updateDAC(sampleA, sampleB);

      //Serial.print("A"); Serial.print(angleA); Serial.print(" "); Serial.println(sampleA);
      //Serial.print("B"); Serial.print(angleB); Serial.print(" "); Serial.println(sampleB);

      //delay(400);
    }*/

    /*///set up sine
    for (int n = 0; n < MAXSAMPLES; n++)
    {
      float t = (float) n / Fs;                                       //creating time isntance to find the 500 samples
      samples[n] = (byte) (127.0 * sin(2 * 3.14 * t) + 127.0 ); //calculating the sin value at each time instance
    }
    sampling_interval = 1000000 / (F * n);                      
    //sampling interval Ts = 1/frequency x number of sample (Ts = 1/Fn or Ts = T/n)x1000000 to convert it in µS
    */

    /*
    for (int i = 0; i < SINE_WAVE_TABLE_LEN; i++) {
        sine_wave_table[i] = 32767u * cosf(i * 2 * (float) (PI / SINE_WAVE_TABLE_LEN)); //multiply by 32767u to preserve decimals
        //Serial.println(sine_wave_table[i]);
    }
    uint32_t step = 0x200000;     //jumps of 32 through the sine table...
    uint32_t pos = 0;//step-1;
    uint32_t pos_max = 0x10000 * SINE_WAVE_TABLE_LEN;
    uint16_t vol = 1;
    while (true) {
        int16_t sampleA = (vol * sine_wave_table[pos >> 16u]) >> 8u;
        int16_t sampleB = sampleA;
        pos += step;
        if (pos >= pos_max) pos -= pos_max;

        char buffer [64]; // must be large enough for your whole string!
        sprintf (buffer, "%d %d %d\n", pos, sampleA, sampleB);
        Serial.print(buffer);
        //Serial.print(pos >> 16u); Serial.print(", ");
        //Serial.print(sampleA);Serial.print(", ");
        //Serial.println(sampleB);
        dac_updateDAC(sampleA, sampleB);
        delay(100);
    }
    */
}


void setup()
{
  Serial.begin(115200);
  Serial.println("Start");

  //Freq Selectors
  pinMode(A0, INPUT);
  pinMode(A1, INPUT);

  // initialize the lcd 
  lcd.init();                      
  lcd.backlight();
  lcd.setCursor(0,0); //col,row
  lcd.print("Wobblevision");
  lcd.setCursor(0,1);
  lcd.print("Dublin Maker '23");
  setupDac();
  startTimers();
}

void dac_updateDAC(uint16_t a, uint16_t b)
{
    #define BITS_RES 12
    if (a > (1u << BITS_RES) - 1) 
      a = (1u << BITS_RES) - 1;  //4095
    if (b > (1u << BITS_RES) - 1)
      b = (1u << BITS_RES) - 1;  //4095
    
    //13u for Gain... 15u for ChanB 12u for Enabled?
    uint16_t commandA = a | (1u << 12u) | (1u << 13u);
    //commandB &= 0xF000u;
    uint16_t commandB  = (1u << 12u) | (1u << 15u) | (1u << 13u) | b;

    /* begin transaction using maximum clock frequency of 20MHz */
    SPI.beginTransaction(SPISettings(20000000, MSBFIRST, SPI_MODE0));

    //A
    digitalWrite(PIN_CS, LOW); //select device
    SPI.transfer16(commandA); // sent command for the A channel
    digitalWrite(PIN_CS, HIGH); //deselect device
    //B
    digitalWrite(PIN_CS, LOW);
    SPI.transfer16(commandB); // sent command for the B channel
    digitalWrite(PIN_CS, HIGH);
    SPI.endTransaction();
}

int olda, oldb;
unsigned long millisA = 0; //millis to complete a cycle A
unsigned long millisAStart = 0; //millis at start of a cycle
unsigned long millisB = 0; //millis to complete a cycle B
unsigned long millisBStart = 0; //millis at start of a cycle
int hzA = 0; //(1/(millisA/1000.0))
int hzB = 0; //(1/(millisB/1000.0))
int oldHzA, oldHzB;
unsigned long lastUpate = 0;

void updateLCD(){
  //unsigned long tiStart = millis();
  //if(oldHzA != hzA || oldHzB != hzB)
  {
    //oldHzA = hzA;
    //oldHzB = hzB;

    float hzA = TIMERSPEED/(2*PI/stepA);  //+0.5;
    float hzB = TIMERSPEED/(2*PI/stepB);  //+0.5);

    lcd.setCursor(0,1);
    char buffer[17];
    sprintf(buffer, "%3.1fHz  %3.1fHz", hzA, hzB);
    sprintf(buffer, "%4dHz    %4dHz", (int)(hzA*10), (int)(hzB*10));
    lcd.print(buffer);
    Serial.println(buffer);
  }

  //above can be slow - abjust timers so wont be out on frequency calcs
  //int timeTaken = millis() - tiStart;
  //Serial.println(timeTaken);
  //millisAStart += timeTaken;
  //millisBStart += timeTaken;
}

void startTimers(){
  pinMode(8, OUTPUT);

  cli();//stop interrupts

  //set timer0 interrupt at 2kHz
  TCCR0A = 0;// set entire TCCR0A register to 0
  TCCR0B = 0;// same for TCCR0B
  TCNT0  = 0;//initialize counter value to 0
  // set compare match register for 2khz increments
  OCR0A = 99;//124; //(16*10^6) / (TIMERSPEED*64) - 1; // ie. 124 (must be <256)
  // turn on CTC mode
  TCCR0A |= (1 << WGM01);
  // Set CS01 and CS00 bits for 64 prescaler
  TCCR0B |= (1 << CS01) | (1 << CS00);   
  // enable timer compare interrupt
  TIMSK0 |= (1 << OCIE0A);

  /*//set timer1 interrupt at 1Hz
  TCCR1A = 0;// set entire TCCR1A register to 0
  TCCR1B = 0;// same for TCCR1B
  TCNT1  = 0;//initialize counter value to 0
  // set compare match register for 1hz increments
  OCR1A = 15624;// = (16*10^6) / (1*1024) - 1 (must be <65536)
  // turn on CTC mode
  TCCR1B |= (1 << WGM12);
  // Set CS10 and CS12 bits for 1024 prescaler
  TCCR1B |= (1 << CS12) | (1 << CS10);  
  // enable timer compare interrupt
  TIMSK1 |= (1 << OCIE1A);

  //set timer2 interrupt at 8kHz
  TCCR2A = 0;// set entire TCCR2A register to 0
  TCCR2B = 0;// same for TCCR2B
  TCNT2  = 0;//initialize counter value to 0
  // set compare match register for 8khz increments
  OCR2A = 249;// = (16*10^6) / (8000*8) - 1 (must be <256)
  // turn on CTC mode
  TCCR2A |= (1 << WGM21);
  // Set CS21 bit for 8 prescaler
  TCCR2B |= (1 << CS21);   
  // enable timer compare interrupt
  TIMSK2 |= (1 << OCIE2A);
  */
  sei();//allow interrupts
}

ISR(TIMER0_COMPA_vect)
{//here 2000 per second
  //angle A is updated by stepA each time
  //  it will take:  (2*PI/stepA) ticks to complete one revolution
  //  in 2000 ticks (one second) it have done  2000/(2*PI/stepA) revolutions
  //  so Hz = 2000/(2*PI/stepA)

  uint16_t sampleA = volume * cosf(angleA) + 2048; //2048 to make midpoint
  uint16_t sampleB = volume * sinf(angleB) + 2048; //2048 to make midpoint

  angleA += stepA;
  angleB += stepB;
  if (angleA > TWOPI){
    angleA -= TWOPI;
  }
  if (angleB > TWOPI){
    angleB -= TWOPI;
  }
  dac_updateDAC(sampleA, sampleB);
  digitalWrite(8, !digitalRead(8));
}


void loop()
{
  int a = analogRead(A0); //0..2023  10 bits
  int b = analogRead(A1); //0..2023  10 bits
  /*if (abs(a-olda) > 2 || abs(b-oldb)> 2){
    //0.01 is Very smooth byt 5hz - 0.05 is ok at 25hz - 0.1 is dotty but 50Hz
    stepA = map(a, 0 , 1023, 1, 100) / 1000.0;
    stepB = map(b, 0 , 1023, 1, 100) / 1000.0;
    olda=a;oldb=b;
    Serial.print(stepA); Serial.print(", "); Serial.println(stepB);
  }*/

  //0.01 is Very smooth byt 5hz - 0.05 is ok at 25hz - 0.1 is dotty but 50Hz
  #define descreteTicksOnDial 100

  float _stepA = map(a, 0 , 1023, 1, descreteTicksOnDial) / 500.0;
  float _stepB = map(b, 0 , 1023, 1, descreteTicksOnDial) / 500.0;
  if (stepA != _stepA || stepB != _stepB){
    stepA = _stepA;
    stepB = _stepB;
    Serial.print(stepA); Serial.print(", "); Serial.println(stepB);
    updateLCD();
  }

  int diff = millis() - lastUpate;
  if (diff > 1000) 
  {
    //Serial.println("2Second");
    lastUpate = millis();
  }
}


