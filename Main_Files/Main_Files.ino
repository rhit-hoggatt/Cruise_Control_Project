//Startup code for arduino cruise control module Porsche 944
//see pinout at https://rennlist.com/forums/944-and-944s-forum/1180331-cruise-control-saga.html
//Written by: John Hoggatt

//Libraries
#include <FreqCount.h>
#include <FreqMeasure.h>
#include <KickMath.h>
#include <List.hpp>
#include <Math.h>
#include <FreqPeriodCounter.h>
#include <Albert.h>
#include <Streaming.h>
#include <FreqCount.h>

//Inputs
int speed_signal = 49;     //analog signal (sinusoid) requires pin 49 for FreqCount lib (pin 8 for UNO)--- pin 3 for FreqPeriodCounter
int brake_signal = 24;      //0v when off 12v when brake pressed
bool brake_signal_state;
int cancel_signal = 26;     //12v when off 0v when cancel pressed
bool cancel_signal_state;
int resume_signal = 28;     //0v until 12v when resume pressed
bool resume_signal_state;
int set_signal = 30;        //0v until 12v when pressed
bool set_signal_state;
int clutch_signal = 32;     //0v until 12v when clutch pressed
bool clutch_signal_state;

//Outputs (not sure how these work yet)
int clutch_1 = 23;
int clutch_2 = 25;
int pot_1 = 27;
int pot_2 = 29;
int motor_up = 31;
int motor_down = 33;

//Other Variables
unsigned long set_speed_freq = 0;
unsigned long current_speed_freq = 0;
unsigned long acc_speed_freq = 0;     //used to measure acceleration
double speed_range = 0.015;  //sets acceptable range from set speed before taking action
double stop_speed_change = 0.005;     //sets range at which speed up/slow down are active
bool canceled = true;
bool clutch_pressed = false;

const int counterPin = 49; 
FreqPeriodCounter counter(counterPin, micros, 0);

void setup() {
  Serial.begin(57600);
  Serial.println("Begin Serial");
  FreqMeasure.begin();
  pinMode(9, OUTPUT);
  
  //input pinmodes
  pinMode(speed_signal, INPUT);
  pinMode(brake_signal, INPUT);
  pinMode(cancel_signal, INPUT);
  pinMode(resume_signal, INPUT);
  pinMode(set_signal, INPUT);
  pinMode(clutch_signal, INPUT);
  pinMode(A0, INPUT);

  //output pinmodes
  pinMode(clutch_1, OUTPUT);
  pinMode(clutch_2, OUTPUT);
  pinMode(pot_1, OUTPUT);
  pinMode(pot_2, OUTPUT);
  pinMode(motor_up, OUTPUT);
  pinMode(motor_down, OUTPUT);
}

void loop() {
  analogWrite(9, 8);
  double tempFreq = get_speed();
  Serial.println(tempFreq);
  set_signal_state = digitalRead(set_signal);
  //Serial.print(set_signal_state);
  if(set_signal_state == HIGH){           //checks if set button is pressed
    set_speed_freq = get_speed();  //read freq for given speed
    canceled = false;     //disables the cancel variable
    if(set_speed_freq == 0){
      cancel();
    }
  }

  if (set_speed_freq != 0 && canceled == false) {     //checks to see if cruise has been set
    //enables servo (not sure how this works yet)
    digitalWrite(clutch_1, HIGH);
    digitalWrite(clutch_2, HIGH);
    current_speed_freq = get_speed();  //reads current speed freq
    if(current_speed_freq == 0){
      cancel();
    }
    if (current_speed_freq > (set_speed_freq + (speed_range * set_speed_freq))){  //if current speed is above a set offset of set speed
      slow_down();
    }
    if (current_speed_freq < (set_speed_freq - (speed_range * set_speed_freq))){  //if current speed is below a set offset of set speed
      speed_up();
    }
  }

  cancel_signal_state = digitalRead(cancel_signal);
  if (cancel_signal_state == LOW){    //checks for cancel button being pressed
    cancel();
  }

  resume_signal_state = digitalRead(resume_signal);
  if (resume_signal_state == HIGH){     //checks for resume button being pressed
    resume();
  }

  brake_signal_state = digitalRead(brake_signal);
  if (brake_signal_state == HIGH){    //checks for brake pedal being pressed
    cancel();
  }

  clutch_signal_state = digitalRead(clutch_signal);
  if (clutch_signal_state == HIGH){     //checks for clutch pedal being pressed
    cancel();
    clutch_pressed = true;
  }

  if(clutch_signal_state == LOW && clutch_pressed){
    clutch_pressed = false;
    resume();
  }

  Serial.print(get_speed());
  Serial.print('\n');

}

double get_speed(){

  unsigned long start_time = 0;
  unsigned long end_time = 0;

  int sensorValue = 0;
  for(int i = 0; i < 100; i++){
    int tempSensorValue = analogRead(A0);
    if(tempSensorValue > sensorValue){
      sensorValue = tempSensorValue;
      start_time = micros();
    }
    delay(0.02);
  }

  float pkVoltage = sensorValue * (5.0 / 1023.0);
  if(pkVoltage < 0.25) return 0;

  int sensorValue1 = 0;
  for(int i = 0; i < 100; i++){
    int tempSensorValue1 = analogRead(A0);
    if(tempSensorValue1 > sensorValue1){
      sensorValue1 = tempSensorValue1;
      end_time = micros();
    }
    delay(0.02);
  }

  double period = end_time - start_time;

  double period_sec = period / 1000000;

  // Serial.println(period_sec);

  double freq = 1 / period_sec;

  return freq;






  // unsigned long temp = 0;
  // int check = 0;
  // for(int i = 0; i < 50; i++){
  //   counter.poll();
  //   int per = counter.period;
  //   if(per > 10000){
  //     per = 0;
  //   }
  //   if(per != 0){
  //     temp += per;
  //     check++;
  //   }
  // }
  // if(check != 0){
  //   unsigned long fin = (temp / check);
  //   if(fin > 10000) fin = 0;
  //   return fin;
  // }
  
  // //if(counter.poll()) Serial << endl << counter.level, counter.period, counter.pulseWidth, counter.pulseWidthLow; 
  // if(counter.elapsedTime > 50000){
  //   Serial.println("No signal from input");
  // }

  // return 0;

}

void speed_up() {
//  delay(500);
  acc_speed_freq = get_speed();  //reads current speed freq
  if(acc_speed_freq == 0){
    cancel();
  }
  if (acc_speed_freq < (current_speed_freq - (stop_speed_change * current_speed_freq))){
    digitalWrite(motor_up, HIGH);
    delay(250);
    digitalWrite(motor_up, LOW);
  }
  
}

void slow_down() {
//  delay(500);
  acc_speed_freq = get_speed();  //reads current speed freq
  if(acc_speed_freq == 0){
    cancel();
  }
  if (acc_speed_freq > (current_speed_freq + (stop_speed_change * current_speed_freq))){
    digitalWrite(motor_down, HIGH);
    delay(250);
    digitalWrite(motor_down, LOW);
  }
  
}

int speed_derivative(){ //0 for speed equal, 1 for speed increasing, -1 for speed decreasing

  if(FreqMeasure.available()){
    List<double> list1(false);
    List<double> list2(false);

    for(int i = 0; i < 20; i++){
      double temp = get_speed();
      list1.addAtIndex(i, temp);
      delay(10);
    }

    delay(100);

    for(int i = 0; i < 20; i++){
      double temp = get_speed();
      list2.addAtIndex(i, temp);
      delay(10);
    }

    double leading = 0;
    double trailing = 0;
    double cur_slope = 0;
    for(int i = 0; i < list1.getSize(); i++){
      if(i == 0){
        leading = list1.getValue(i);
      }
      else{
        trailing = leading;
        leading = list1.getValue(i);
        if(cur_slope == 0){
          cur_slope = leading - trailing;
        }
        else{
          cur_slope = (leading - trailing) + cur_slope;
        }
      }
    }

    double leading1 = 0;
    double trailing1 = 0;
    double cur_slope1 = 0;
    for(int i = 0; i < list2.getSize(); i++){
      if(i == 0){
        leading1 = list2.getValue(i);
      }
      else{
        trailing1 = leading1;
        leading1 = list2.getValue(i);
        if(cur_slope == 0){
          cur_slope1 = leading1 - trailing1;
        }
        else{
          cur_slope1 = (leading1 - trailing1) + cur_slope1;
        }
      }
    }

    double combines = cur_slope + cur_slope1;

    if(combines == 0){
      return 0;
    }
    else if(combines < 0){
      return -1;
    }
    else{
      return 1;
    }

  }
  else{
    cancel();
    return 0;
  }

}

void resume(){
  digitalWrite(clutch_1, HIGH);
  digitalWrite(clutch_2, HIGH);
  canceled = false;
}

void cancel(){
  digitalWrite(clutch_1, LOW);
  digitalWrite(clutch_2, LOW);
  digitalWrite(pot_1, LOW);
  digitalWrite(pot_2, LOW);
  digitalWrite(motor_up, LOW);
  digitalWrite(motor_down, LOW);
  canceled = true;
}
