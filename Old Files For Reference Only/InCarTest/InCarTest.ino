//Inputs
int speed_signal = 49;     //analog signal (sinusoid) requires pin 49 for FreqCount lib (pin 8 for UNO)
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
int clutch_1 = 35;   //First of two safety magnetic clutches in servo (?)
int clutch_2 = 37;   //Second of two saferty magnetic clutches in servo (?)
int pot_1 = 27;
int pot_2 = 29;
int motor_up = 31;
int motor_down = 33;

int mains = 23;

void setup() {
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
  pinMode(mains, OUTPUT);

  digitalWrite(mains, LOW);
  digitalWrite(clutch_1, LOW);
  digitalWrite(clutch_2, LOW);
  digitalWrite(pot_1, LOW);
  digitalWrite(pot_2, LOW);
  digitalWrite(motor_up, LOW);
  digitalWrite(motor_down, LOW);
}

void loop() {
  // put your main code here, to run repeatedly:
  // digitalWrite(motor_up, LOW);

  // delay(1000);

  // digitalWrite(motor_up, HIGH);
  // digitalWrite(motor_down, LOW);

  // delay(1000);

  // digitalWrite(motor_down, HIGH);

}
