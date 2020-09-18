#include <math.h>
#include <avr/interrupt.h>
#define twopi 6.28318531
#define encoderPinA 2
#define encoderPinB 11


int pwn_pin = 9;
int in1 = 6;
int in2 = 4;

int speed;

int sw_pin = 10;
int ISRstate;
int nISR;
int stop_n = 250;
unsigned long t0;
unsigned long t;
unsigned long t1;
int dt_micro;
unsigned long prevt;
float dt_ms;
float dt_sec;
float t_ms;
float t_sec;
float y;
int inByte;
int ISR_Happened;
int state;
int width; 
int amp;
int printamp; //Global variable to print amp value 
int theta = 0; //Angle of Motor 

void setup()
{
  Serial.begin(115200);
  Serial.print("Lab 3: Open-Loop DC Motor Control");
  Serial.print("\n");

  pinMode(sw_pin, OUTPUT);

  pinMode(pwn_pin, OUTPUT);
  pinMode(in1, OUTPUT);
  pinMode(in2, OUTPUT);  
 
  pinMode(encoderPinA, INPUT); 
  pinMode(encoderPinB, INPUT);
  
  attachInterrupt(0, doEncoder, RISING);
  
  //=======================================================
  // set up the Timer2 interrupt
  //=======================================================
  cli();// disable global interrupts temporarily
  //set timer2 interrupt at 8kHz
  TCCR2A = 0;// set entire TCCR2A register to 0
  TCCR2B = 0;// same for TCCR2B
  TCNT2  = 0;//initialize counter value to 0
  // set compare match register for 8khz increments
  OCR2A = 61;// = (16*10^6) / (8000*8) - 1 (must be <256)
  // turn on CTC mode
  TCCR2A |= (1 << WGM21);
  TCCR2B = (TCCR2B & 0b11111000) | 0x07;// set prescaler to 1024
  // taken from here: https://playground.arduino.cc/Main/TimerPWMCheatsheet
  //        scroll down to "Pins 11 and 3: controlled by timer 2"
  //TCCR2B = (TCCR2B & 0b11111000) | 0x06;
  // enable timer compare interrupt
  TIMSK2 |= (1 << OCIE2A);
  sei();// re-enable global interrupts
  //=======================================================

}

void doEncoder(){
  //Do something cool
  //When A's rising edge is triggered, check if B LOW or HIGH to see if motor is spinning in positive or negative direction  
  if(digitalRead(encoderPinB) == 0)
  {
    theta++; 
  }
  else 
  {
    theta--;
  }
}

//COMMAND MOTOR SPEED
void command_motor_speed(int speed){
if ((speed > 0) || (speed == 0)){
// go forward
  analogWrite(pwn_pin, speed);
  digitalWrite(in1, HIGH); 
  digitalWrite(in2, LOW); 
  }
else if (speed < 0){
// go backwards
  analogWrite(pwn_pin, abs(speed));
  digitalWrite(in2, HIGH); 
  digitalWrite(in1, LOW); 
  }
}


void mynewline(){
  Serial.print('\n');
}

int get_int(){
  int out_int;
  out_int = 0;
  while (out_int == 0){
    while (Serial.available() == 0){
      delay(10);
    }
    out_int = Serial.parseInt();
  }
  return(out_int);  
}

void menu(){
  Serial.println("Enter width:");//-255 is full speed backwards, 255 is full speed forward
  width = get_int();
  Serial.print("Width entered:");
  Serial.println(width);
  Serial.println("Enter Amplitude:"); 
  amp = get_int();
  Serial.print("Amplitude Entered:");
  Serial.println(amp); 

  Serial.println("enter s to start a new test");
  mynewline();
  state = 0;// If something goes wrong, set code to return to menu

  while ( state == 0){
    while (Serial.available() == 0){
      delay(10);
    }
    inByte = Serial.read();
    if ( inByte == 's' ){
      nISR = 0;
      state = 1;
      Serial.print("#start");
      mynewline();
      Serial.print("#n, t (ms), y(t)");
      mynewline();
      delay(50);
      t0 = micros();
    }
    else{
      Serial.print("input not regonzided: ");
      Serial.print(inByte);
      mynewline();
    }
  }
}


void loop()
{
  if ( state == 0){
    menu();
  }
    
  else if (state == 1){
    if (( ISR_Happened == 1) && (nISR < stop_n)){
      ISR_Happened = 0;
      t = micros()-t0;
      nISR++;
      t_ms = t/1000.0;
      t_sec = t_ms/1000.0;
  
      // do other stuff here
      if (nISR < 10){
        command_motor_speed(0);
        //global variable set to zero 
        printamp = 0;
      }
      else if (nISR >= 10 && nISR < (10 + width)) {
        command_motor_speed(amp);
        //gloabal variable set to amp 
        printamp = amp; 
      }
      else if (nISR >= (10 + width)) {
        command_motor_speed(0);
        //global variable set to zero
        printamp = 0; 
      }

    
      Serial.print(nISR);
      Serial.print(",");
      Serial.print(t_ms);
      Serial.print(",");
      Serial.print(printamp);
      Serial.print(",");
      Serial.print(theta);
   
      mynewline();
    }
 
    else if (nISR >= stop_n){
      Serial.println("#end");
      Serial.println("#====================");
      Serial.println("");
      state = 0;
      analogWrite(pwn_pin, 0);
    }
  }
}


ISR(TIMER2_COMPA_vect)
{
  ISR_Happened = 1;

  // generate square wave to verify timing
  if (ISRstate == 1){
    ISRstate = 0;
    digitalWrite(sw_pin, LOW);
  }
  else{
    ISRstate = 1;
    digitalWrite(sw_pin, HIGH);
  }
}
