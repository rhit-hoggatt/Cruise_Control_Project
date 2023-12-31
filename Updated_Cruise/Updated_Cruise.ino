//Startup code for arduino cruise control module Porsche 944
//see pinout at https://rennlist.com/forums/944-and-944s-forum/1180331-cruise-control-saga.html
//Written by: John H.

//USE AT YOUR OWN RISK

//Libraries
#include <FreqCount.h>
#include <FreqMeasure.h>
#include <List.hpp>

//Inputs
int speed_signal = 8;     //analog signal (sinusoid) requires pin 49 for FreqCount lib (pin 8 for UNO)--- pin 3 for FreqPeriodCounter
int brake_signal = 15;      //0v when off 12v when brake pressed (A1)
bool brake_signal_state;
int cancel_signal = 16;     //12v when off 0v when cancel pressed (A2)
bool cancel_signal_state;
int resume_signal = 11;     //0v until 12v when resume pressed
bool resume_signal_state;
int set_signal = 10;        //0v until 12v when pressed
bool set_signal_state;
int clutch_signal = 9;     //0v until 12v when clutch pressed
bool clutch_signal_state;

//Outputs (not sure how these work yet)
int clutch_1 = 7;   //First of two safety magnetic clutches in servo
int clutch_2 = 6;   //Second of two saferty magnetic clutches in servo
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
bool clutch_pressed = false;  //Used in resume function for when clutch is no longer pressed

//list of previous speeds to reference
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

//speed up and slow down globals to eliminate using delay()
bool SO = false;
bool SD = false;
long long speed_up_initial = 0;
long long speed_up_current = 0;
long long slow_down_initial = 0;
long long slow_down_current = 0;

// AList AListTest = new AList();

//just for testing
const int counterPin = 49; 
// FreqPeriodCounter counter(counterPin, micros, 0);
int speedSlopeAvg = -1;
float speedSlopeSum = 0;


//system time globals for reference
long long current_system_time = 0;
long long prev_system_time = 0;

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
  pinMode(A2, INPUT);

  //output pinmodes
  pinMode(clutch_1, OUTPUT);
  pinMode(clutch_2, OUTPUT);
  pinMode(pot_1, OUTPUT);
  pinMode(pot_2, OUTPUT);
  pinMode(motor_up, OUTPUT);
  pinMode(motor_down, OUTPUT);
}

void loop(){
  current_system_time = millis(); //number of milliseconds since system started
  Serial.print("Loop time: ");
  Serial.println((double) current_system_time - (double) prev_system_time);  //check "heartbeat" i.e. is how fast is loop running

  int tempSpeed = get_speed_2();  //Gets current freq if avaliable
  if(tempSpeed != -1){  //Checks if current speed was avaliable
    current_frequency = tempSpeed;
  }

  set_signal_state = digitalRead(set_signal);
  if(set_signal_state == HIGH){           //checks if set button is pressed
    Serial.println("Set Button Pressed");
    set_speed_freq = current_frequency;  //read freq for given speed
    canceled = false;     //disables the cancel variable
  }

    if (set_speed_freq != 0 && canceled == false) {     //checks to see if cruise has been set
    //enables servo (not sure how this works yet)
    digitalWrite(clutch_1, HIGH);
    digitalWrite(clutch_2, HIGH);
    Serial.println("Running");
    current_speed_freq = current_frequency;
    if (current_speed_freq > (set_speed_freq + (speed_range * set_speed_freq)) || SD){  //if current speed is above a predefined offset of set speed
      slow_down();
    }
    if (current_speed_freq < (set_speed_freq - (speed_range * set_speed_freq)) || SO){  //if current speed is below a predefined offset of set speed
      speed_up();
    }
  }

  cancel_signal_state = digitalRead(cancel_signal);
  if (cancel_signal_state == LOW){    //checks for cancel button being pressed
    Serial.println("Cancel Button Pressed");
    cancel(116);
  }

  resume_signal_state = digitalRead(resume_signal);
  if (resume_signal_state == HIGH){     //checks for resume button being pressed
    Serial.println("Resume Button Pressed");
    resume();
  }

  brake_signal_state = digitalRead(brake_signal);
  if (brake_signal_state == HIGH){    //checks for brake pedal being pressed
    Serial.println("Brake Pressed");
    cancel(128);
  }

  clutch_signal_state = digitalRead(clutch_signal);
  if (clutch_signal_state == HIGH){     //checks for clutch pedal being pressed
    Serial.println("Clutch Pressed");
    cancel(133);
    clutch_pressed = true;
  }

  if(clutch_signal_state == LOW && clutch_pressed){
    Serial.println("Clutch no longer pressed");
    clutch_pressed = false;
    resume();
  }

  if(last100.getSize() > 1000){  //ensures list of previous speed values doesnt get too large
    speedSlopeSum = last100.getValue(0);
    last100.removeFirst();
    Serial.print("Freq List Size: ");
    Serial.println(last100.getSize());
  }

  prev_system_time = current_system_time; //system time reference
  previous_frequency = current_frequency; //changes previous frequency value for next loop
}

void resume(){    //re-enables clutches (Havent tested this yet)
  digitalWrite(clutch_1, HIGH);
  digitalWrite(clutch_2, HIGH);
  canceled = false;
}

void cancel(int lineNum){ //Disables all outputs and sets canceled to true
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

void speed_up(){
  if(!SO){
    SO = true;
    speed_up_initial = millis();
  }
  speed_up_current = millis();

  if(speed_up_current <= speed_up_initial + 250){
    digitalWrite(motor_up, HIGH);
  }
  else{
    digitalWrite(motor_up, LOW);
    SO = false;
  }

}

void slow_down(){
  if(!SD){
    SD = true;
    slow_down_initial = millis();
  }
  slow_down_current = millis();

  if(slow_down_current <= slow_down_initial + 250){
    digitalWrite(motor_down, HIGH);
  }
  else{
    digitalWrite(motor_down, LOW);
    SD = false;
  }
}

double get_speed_2(){ //returns average of 5 frequency readings over time
  int curVal = helper();  //calls helper method to get one freq to be averaged
  if(curVal != -1){ //if curVal is avaliable
    if(10 * curVal < 50){ //arbitrary offset as low freqs are not reliable to read
      freqList.add(0);  //considers frequency to be essentially zero
    }
    else{ //assuming freq is valid
      freqList.add(curVal);
    }
  }

  if(freqList.getSize() >= 5){  //averages values when list size gets to 5
    int avg_sum = 0;
    for(int i = 0; i < freqList.getSize(); i++){
      avg_sum += freqList.getValue(i);
    }

    int freq = (avg_sum / freqList.getSize());

    Serial.print("Method 2: ");
    Serial.println(freq);

    freqList.removeAll();

    if(freq == 0){  //this is for testing- ignore
      return 0;
    }
    // Serial.print("Method 2: ");
    // Serial.println(freq);
    return freq;  //returns final averaged value
  }

  return -1;  //Returns -1 to show that freq reading not current avaliable
}

int helper(){   //get speed helper method to return current freq (not averaged)

  if(freq_list_count == 0){ //checks if this is the first run of this "cycle"
    freqList_start = millis();
    freq_list_count++;  //makes this no longer the first run
  }

  freqList_current = millis();
  if(freqList_current > freqList_start + 100){  //only returns value after 100ms of reading
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

int speed_slope(){  //not currently used but useful information
  long sum = 0;
  if(speedSlopeAvg == -1){
    if(last100.getSize() > 2){
      for(int i = last100.getSize() - 2; i < last100.getSize(); i++){
        int check = last100.getValue(i) - last100.getValue(i - 1);
        sum += check;
      }
    }
    speedSlopeAvg = sum;
  }
  else{
    speedSlopeAvg -= speedSlopeSum;
    speedSlopeAvg += last100.getValue(last100.getSize() - 1);
  }

  if(sum > 0) return 1; //returns 1 if positive slope
  else if(sum < 0) return -1; //returns -1 is negative slope
  else return 0;  //zero is slope is zero

}