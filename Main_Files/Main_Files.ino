//Startup code for arduino cruise control module Porsche 944
//see pinout at https://rennlist.com/forums/944-and-944s-forum/1180331-cruise-control-saga.html
//Written by: John H.

//USE AT YOUR OWN RISK

//Libraries
#include <FreqCount.h>
#include <FreqMeasure.h>
#include <KickMath.h>
#include <List.hpp>
#include <Math.h>
#include <FreqPeriodCounter.h>
// #include <Albert.h>
// #include <Streaming.h>
#include <FreqCount.h>

//Inputs
int speed_signal = 8;     //analog signal (sinusoid) requires pin 49 for FreqCount lib (pin 8 for UNO)--- pin 3 for FreqPeriodCounter
int brake_signal = 15;      //0v when off 12v when brake pressed (A1)
bool brake_signal_state;
int cancel_signal = 19;     //12v when off 0v when cancel pressed
bool cancel_signal_state;
int resume_signal = 11;     //0v until 12v when resume pressed
bool resume_signal_state;
int set_signal = 10;        //0v until 12v when pressed
bool set_signal_state;
int clutch_signal = 9;     //0v until 12v when clutch pressed
bool clutch_signal_state;

//Outputs (not sure how these work yet)
int clutch_1 = 7;
int clutch_2 = 6;
int pot_1 = 5;
int pot_2 = 4;
int motor_up = 3;
int motor_down = 2;

//Other Variables
long set_speed_freq = 0;
long current_speed_freq = 0;
long acc_speed_freq = 0;     //used to measure acceleration
double speed_range = 0.015;  //sets acceptable range from set speed before taking action
double stop_speed_change = 0.005;     //sets range at which speed up/slow down are active
bool canceled = true;
bool clutch_pressed = false;

double current_frequency = 0;
double newCurFreq = 0;
double previous_frequency = 0;
List<double> last100(true);

long total_cycles = 0;

//get_speed_2 globals
long long freqList_start = 0;
long long freqList_current = 0;
int freq_list_count = 0;
List<int> freqList(false);
int num = 0;
int prevNum = 0;
bool numCheck = true;

const int counterPin = 49; 
FreqPeriodCounter counter(counterPin, micros, 0);

void setup() {
  Serial.begin(57600);
  Serial.println("Begin Serial");
  FreqMeasure.begin();
  // pinMode(9, OUTPUT);
  
  //input pinmodes
  pinMode(speed_signal, INPUT);
  pinMode(brake_signal, INPUT);
  pinMode(cancel_signal, INPUT);
  pinMode(resume_signal, INPUT);
  pinMode(set_signal, INPUT);
  pinMode(clutch_signal, INPUT);
  pinMode(A0, INPUT);
  pinMode(A1, INPUT);

  //output pinmodes
  pinMode(clutch_1, OUTPUT);
  pinMode(clutch_2, OUTPUT);
  pinMode(pot_1, OUTPUT);
  pinMode(pot_2, OUTPUT);
  pinMode(motor_up, OUTPUT);
  pinMode(motor_down, OUTPUT);
}

void loop() {
  if(total_cycles == 0){
    digitalWrite(clutch_1, LOW);
    digitalWrite(clutch_2, LOW);
    digitalWrite(pot_1, LOW);
    digitalWrite(pot_2, LOW);
    digitalWrite(motor_up, LOW);
    digitalWrite(motor_down, LOW);
  }
  total_cycles++;
  // current_frequency = get_speed();
  int tempSpeed = get_speed_2();
  current_frequency = tempSpeed;

  if(tempSpeed != -1){
    newCurFreq = tempSpeed;
  }
  set_signal_state = digitalRead(set_signal);
  //Serial.print(set_signal_state);
  if(set_signal_state == HIGH){           //checks if set button is pressed
    // set_speed_freq = get_speed();  //read freq for given speed
    set_speed_freq = current_frequency;
    canceled = false;     //disables the cancel variable
    if(set_speed_freq == 0){
      cancel(99);
    }
  }

  if (set_speed_freq != 0 && canceled == false) {     //checks to see if cruise has been set
    //enables servo (not sure how this works yet)
    digitalWrite(clutch_1, HIGH);
    digitalWrite(clutch_2, HIGH);

    Serial.println("running");
    current_speed_freq = current_frequency;
    if(current_speed_freq == 0){
      cancel(111);
    }
    if (current_speed_freq > (set_speed_freq + (speed_range * set_speed_freq))){  //if current speed is above a predefined offset of set speed
      slow_down();
    }
    if (current_speed_freq < (set_speed_freq - (speed_range * set_speed_freq))){  //if current speed is below a predefined offset of set speed
      speed_up();
    }
  }

  cancel_signal_state = digitalRead(cancel_signal);
  Serial.print("Cancel state: ");
  Serial.println(cancel_signal_state);
  if (cancel_signal_state == LOW){    //checks for cancel button being pressed
    cancel(123);
  }

  resume_signal_state = digitalRead(resume_signal);
  if (resume_signal_state == HIGH){     //checks for resume button being pressed
    resume();
  }

  brake_signal_state = digitalRead(brake_signal);
  if (brake_signal_state == HIGH){    //checks for brake pedal being pressed
    cancel(133);
  }

  clutch_signal_state = digitalRead(clutch_signal);
  if (clutch_signal_state == HIGH){     //checks for clutch pedal being pressed
    cancel(138);
    clutch_pressed = true;
  }

  if(clutch_signal_state == LOW && clutch_pressed){
    clutch_pressed = false;
    resume();
  }

  previous_frequency = current_frequency;

  if(current_frequency != -1){
    last100.add(current_frequency);
  }

  if(last100.getSize() > 10){
    last100.removeFirst();
    Serial.print("Freq List Size: ");
    Serial.println(last100.getSize());
  }
}

double get_speed(){

  List<int> freqList(false);

  for(int i = 0; i < 5; i++){
    long long start_time = millis();
    long long cur_time = 0;
    int count = 0;
    int previous = 0;
    bool check = true;
    while(cur_time < start_time + 100){
      cur_time = millis();
      int curWave = analogRead(A0);
      if(curWave < previous && check){
        float pkVoltage = curWave * (5.0 / 1023.0);
        if(pkVoltage > 1){
          count++;
          check = false;
        }
        
      }
      else if(curWave > previous){
        check = true;
      }
      previous = curWave;
    }
    if(10 * count < 50){
      freqList.add(0);
    }
    else{
      freqList.add(10 * count);
    }
  }

  int avg_sum = 0;
  for(int i = 0; i < freqList.getSize(); i++){
    avg_sum += freqList.getValue(i);
  }

  int freq = (avg_sum / freqList.getSize());

  Serial.print("Method 1: ");
  Serial.println(freq);

  if(freq == 0){
    return 1;
  }
  return freq;

}

double get_speed_2(){
  int curVal = helper();
  if(curVal != -1){
    if(10 * curVal < 50){
      freqList.add(0);
    }
    else{
      freqList.add(curVal);
    }
  }

  if(freqList.getSize() >= 5){
    int avg_sum = 0;
    for(int i = 0; i < freqList.getSize(); i++){
      avg_sum += freqList.getValue(i);
    }

    int freq = (avg_sum / freqList.getSize());

    Serial.print("Method 2: ");
    Serial.println(freq);

    freqList.removeAll();

    if(freq == 0){
      return 1;
    }
    // Serial.print("Method 2: ");
    // Serial.println(freq);
    return freq;
  }

  return -1;
}

int helper(){

  if(freq_list_count == 0){
    freqList_start = millis();
    freq_list_count++;
  }

  freqList_current = millis();
  if(freqList_current > freqList_start + 100){
    int cur = 10 * num;
    num = 0;
    prevNum = 0;
    numCheck = true;
    freq_list_count = 0;
    return cur;
  }
  else{
    int curWave = analogRead(A0);
    if(curWave < prevNum && numCheck){
      float pkVoltage = curWave * (5.0 / 1023.0);
      if(pkVoltage > 0){
        num++;
        numCheck = false;
      }
    }
    else if(curWave > prevNum){
      numCheck = true;
    }
  }

  return -1;

}

int speed_slope(){
  long sum = 0;
  if(last100.getSize() > 2){
    for(int i = last100.getSize() - 2; i < last100.getSize(); i++){
      int check = last100.getValue(i) - last100.getValue(i - 1);
      sum += check;
    }
  }

  if(sum > 0) return 1;
  else if(sum < 0) return -1;
  else return 0;

}

void speed_up() {
//  delay(500);
  // acc_speed_freq = get_speed();  //reads current speed freq
  acc_speed_freq = current_frequency;
  if(acc_speed_freq == 0){
    cancel(230);
  }
  if (acc_speed_freq < (current_speed_freq - (stop_speed_change * current_speed_freq))){
    digitalWrite(motor_up, HIGH);
    delay(250);
    digitalWrite(motor_up, LOW);
  }

  if(speed_slope() == -1){
    digitalWrite(motor_up, HIGH);
    delay(250);
    digitalWrite(motor_up, LOW);
  }
  
}

void slow_down() {
//  delay(500);
  // acc_speed_freq = get_speed();  //reads current speed freq
  acc_speed_freq = current_frequency;
  if(acc_speed_freq == 0){
    cancel(251);
  }
  if (acc_speed_freq > (current_speed_freq + (stop_speed_change * current_speed_freq))){
    digitalWrite(motor_down, HIGH);
    delay(250);
    digitalWrite(motor_down, LOW);
  }

  if(speed_slope() == 1){
    digitalWrite(motor_down, HIGH);
    delay(250);
    digitalWrite(motor_down, LOW);
  }
  
}

void resume(){
  digitalWrite(clutch_1, HIGH);
  digitalWrite(clutch_2, HIGH);
  canceled = false;
}

void cancel(int lineNum){
  Serial.print("Canceling: Line ");
  Serial.println(lineNum);
  digitalWrite(clutch_1, LOW);
  digitalWrite(clutch_2, LOW);
  digitalWrite(pot_1, LOW);
  digitalWrite(pot_2, LOW);
  digitalWrite(motor_up, LOW);
  digitalWrite(motor_down, LOW);
  canceled = true;
}

void flip_values(){
  bool state1 = digitalRead(clutch_1);
  if(state1 == 1){
    digitalWrite(clutch_1, LOW);
  }
  else{
    digitalWrite(clutch_1, HIGH);
  }

  bool state2 = digitalRead(clutch_2);
  if(state2 == 1){
    digitalWrite(clutch_2, LOW);
  }
  else{
    digitalWrite(clutch_2, HIGH);
  }

  bool state3 = digitalRead(pot_1);
  if(state3 == 1){
    digitalWrite(pot_1, LOW);
  }
  else{
    digitalWrite(pot_1, HIGH);
  }

  bool state4 = digitalRead(pot_1);
  if(state4 == 1){
    digitalWrite(pot_2, LOW);
  }
  else{
    digitalWrite(pot_2, HIGH);
  }

  bool state5 = digitalRead(motor_up);
  if(state5 == 1){
    digitalWrite(motor_up, LOW);
  }
  else{
    digitalWrite(motor_up, HIGH);
  }

  bool state6 = digitalRead(motor_up);
  if(state6 == 1){
    digitalWrite(motor_down, LOW);
  }
  else{
    digitalWrite(motor_down, HIGH);
  }
}
