//Startup code for arduino cruise control module Porsche 944
//see wiring pinout at https://rennlist.com/forums/944-and-944s-forum/1180331-cruise-control-saga.html
//Written by: John H.

//USE AT YOUR OWN RISK

#include <FreqMeasure.h>
#include <PID_v1.h>

//Inputs
int speed_signal = 49;     //analog signal (sinusoid) requires pin 49 for FreqCount lib (pin 8 for UNO)--- pin 3 for FreqPeriodCounter
int brake_signal = 3;      //0v when off 12v when brake pressed (A1)
bool brake_signal_state;
int cancel_signal = 2;     //12v when off 0v when cancel pressed (A2)
bool cancel_signal_state;
int resume_signal = 19;     //0v until 12v when resume pressed
bool resume_signal_state;
int set_signal = 20;        //0v until 12v when pressed
bool set_signal_state;
int clutch_signal = 18;     //0v until 12v when clutch pressed
bool clutch_signal_state;

//Outputs (not sure how these work yet)
int clutch_1 = 23;   //First of two safety magnetic clutches in servo (?)
int clutch_2 = 25;   //Second of two saferty magnetic clutches in servo (?)
int pot_1 = 27;
int pot_2 = 29;
int motor_up = 31;
int motor_down = 33;

//speed information in hz
double set_speed_freq = 0;
double cur_speed_freq = 0;

//other
bool canceled = true; //system "power switch"
bool clutch_pressed = false;
double speed_range = 0.04;

//PID Controller
//Define Variables we'll be connecting to
double Setpoint, Input, Output;

//Define the aggressive and conservative Tuning Parameters for PID
double aggKp=4, aggKi=0.2, aggKd=1;
double consKp=1, consKi=0.05, consKd=0.25;
int WindowSize = 1000;
unsigned long windowStartTime;
#define RELAY_UP 3
#define RELAY_DOWN 2

PID myPID(&Input, &Output, &Setpoint, consKp, consKi, consKd, DIRECT);

void setup() {
  Serial.begin(57600);
  Serial.println("Started Setup");

  //Frequency measuring setup
  FreqMeasure.begin();

  //PID setup
  windowStartTime = millis();
  Input = cur_speed_freq;
  //tell the PID to range between 0 and the full window size
  myPID.SetOutputLimits(0, WindowSize);

  //input pinmodes
  pinMode(speed_signal, INPUT);
  pinMode(brake_signal, INPUT);
  pinMode(cancel_signal, INPUT);
  pinMode(resume_signal, INPUT);
  pinMode(set_signal, INPUT);
  pinMode(clutch_signal, INPUT);
  pinMode(A0, INPUT);
  pinMode(A1, INPUT);
  pinMode(A2, INPUT);

  //output pinmodes
  pinMode(clutch_1, OUTPUT);
  pinMode(clutch_2, OUTPUT);
  pinMode(pot_1, OUTPUT);
  pinMode(pot_2, OUTPUT);
  pinMode(motor_up, OUTPUT);
  pinMode(motor_down, OUTPUT);

  //define interrupts
  attachInterrupt(digitalPinToInterrupt(brake_signal), cancel, RISING);
  attachInterrupt(digitalPinToInterrupt(cancel_signal), cancel, FALLING);
  attachInterrupt(digitalPinToInterrupt(resume_signal), resume, RISING);
  attachInterrupt(digitalPinToInterrupt(clutch_signal), clutchCheck, RISING);

}

void loop() {
  set_signal_state = digitalRead(set_signal);
  if(set_signal_state == HIGH){           //checks if set button is pressed
    Serial.println("Set Button Pressed");
    set_speed_freq = getCurrentFreq();  //read freq for given speed
    canceled = false;     //disables the cancel variable
    Setpoint = set_speed_freq;
    cur_speed_freq = set_speed_freq;
    myPID.SetMode(AUTOMATIC);
  }

  if(set_speed_freq != 0 && canceled == false) {     //checks to see if cruise has been set
    //enables servo (not sure how this works yet)
    digitalWrite(clutch_1, HIGH);
    digitalWrite(clutch_2, HIGH);
    //Serial.println("Running");
    cur_speed_freq = getCurrentFreq();
    Input = cur_speed_freq;
    Serial.println(Input);

    if (cur_speed_freq > (set_speed_freq + (speed_range * set_speed_freq))){  //if current speed is above a predefined offset of set speed
      //slow_down();
      myPID.SetTunings(aggKp, aggKi, aggKd);
    }
    else if (cur_speed_freq < (set_speed_freq - (speed_range * set_speed_freq))){  //if current speed is below a predefined offset of set speed
      //speed_up();
      myPID.SetTunings(aggKp, aggKi, aggKd);
    } else{
      myPID.SetTunings(consKp, consKi, consKd);
    }
  }

  cancel_signal_state = digitalRead(cancel_signal);
  if (cancel_signal_state == LOW){    //checks for cancel button being pressed
    Serial.println("Cancel Button Pressed");
    cancel();
  }

  resume_signal_state = digitalRead(resume_signal);
  if (resume_signal_state == HIGH){     //checks for resume button being pressed
    Serial.println("Resume Button Pressed");
    resume();
  }

  brake_signal_state = digitalRead(brake_signal);
  if (brake_signal_state == HIGH){    //checks for brake pedal being pressed
    Serial.println("Brake Pressed");
    cancel();
  }

  clutch_signal_state = digitalRead(clutch_signal);
  clutchCheck();

  if(myPID.GetMode() == AUTOMATIC){
    myPID.Compute();

    if(millis() - windowStartTime > WindowSize){
      windowStartTime += WindowSize;
    }
    if(Output < millis() - windowStartTime){
      //Serial.println("Arrived at change output");
      if(cur_speed_freq > set_speed_freq + (0.01 * set_speed_freq)){
        digitalWrite(motor_down, HIGH);
        digitalWrite(motor_up, LOW);
      } else if (cur_speed_freq < set_speed_freq - (0.01 * set_speed_freq)){
        digitalWrite(motor_down, LOW);
        digitalWrite(motor_up, HIGH);
      } else{
        digitalWrite(motor_down, LOW);
        digitalWrite(motor_up, LOW);
      }
    }

  }
  //delay(500);   //timing test
}

double getCurrentFreq(){
  double sum=0;
  int count=0;
  while(1){
    if (FreqMeasure.available()) {
      // average several reading together
      sum = sum + FreqMeasure.read();
      count = count + 1;
      if (count > 30) {
        float frequency = FreqMeasure.countToFrequency(sum / count);
        //Serial.println(frequency);
        sum = 0;
        count = 0;
        return frequency;
      }
    }
  }
}

void resume(){    //re-enables clutches (Havent tested this yet)
  digitalWrite(clutch_1, HIGH);
  digitalWrite(clutch_2, HIGH);
  myPID.SetMode(AUTOMATIC);
  canceled = false;
}

void clutchCheck(){
  if (clutch_signal_state == HIGH){     //checks for clutch pedal being pressed
    Serial.println("Clutch Pressed");
    cancel();
    clutch_pressed = true;
  }

  if(clutch_signal_state == LOW && clutch_pressed){
    Serial.println("Clutch no longer pressed");
    clutch_pressed = false;
    resume();
  }
}

void cancel(){ //Disables all outputs and sets canceled to true
  //Serial.print("Canceling: Line ");
  //Serial.println(lineNum);
  digitalWrite(clutch_1, LOW);
  digitalWrite(clutch_2, LOW);
  digitalWrite(pot_1, LOW);
  digitalWrite(pot_2, LOW);
  digitalWrite(motor_up, LOW);
  digitalWrite(motor_down, LOW);
  myPID.SetMode(0);
  canceled = true;
}
