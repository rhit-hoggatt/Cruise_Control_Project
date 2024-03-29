
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

//speed information in hz(?)
double set_speed_freq = 0;
double cur_speed_freq = 0;

//timing information for each loop
long start_sys_time = 0;
long loop_start_time = 0;
long speedUpRef = 0;
long slowDownRef = 0;

//other
bool canceled = true; //system "power switch"
bool clutch_pressed = false;
double speed_range = 0.02;

void setup() {
  Serial.begin(57600);
  Serial.println("Started Setup");

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

void loop() {
  long current_sys_time = millis();

  set_signal_state = digitalRead(set_signal);
  if(set_signal_state == HIGH){           //checks if set button is pressed
    Serial.println("Set Button Pressed");
    set_speed_freq = getCurrentFreq();  //read freq for given speed
    canceled = false;     //disables the cancel variable
  }

    if (set_speed_freq != 0 && canceled == false) {     //checks to see if cruise has been set
    //enables servo (not sure how this works yet)
    digitalWrite(clutch_1, HIGH);
    digitalWrite(clutch_2, HIGH);
    Serial.println("Running");
    double current_speed_freq = getCurrentFreq();
    if (current_speed_freq > (set_speed_freq + (speed_range * set_speed_freq))){  //if current speed is above a predefined offset of set speed
      slow_down();
    }
    if (current_speed_freq < (set_speed_freq - (speed_range * set_speed_freq))){  //if current speed is below a predefined offset of set speed
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
  long curTime = millis();
  if(curTime > speedUpRef + 50){
    speedUpRef = curTime;
    digitalWrite(motor_up, HIGH);
  }
  digitalWrite(motor_up, HIGH);
  delay(50);
  digitalWrite(motor_up, LOW);
}

void slow_down(){
  digitalWrite(motor_down, HIGH);
  delay(50);
  digitalWrite(motor_down, LOW);
}

double getCurrentFreq(){
  #define SAMPLES 4096
  double curFreq = 0;
  for(int j = 0; j < SAMPLES; j++){
    curFreq+= 500000/pulseIn(speed_signal, HIGH, 250000);
  }
  return curFreq / SAMPLES;
}
