/********************************************
   Functions for pid configuration
 ********************************************/
#include <PID_v1.h>

double Setpoint, Input, Output;
double Kp=0.0015, Ki=0.04, Kd=0.005;

PID myPID(&Input, &Output, &Setpoint, Kp, Ki, Kd, DIRECT);
int PWM = 11; // assigns pin 11 to variable pwm
int DIR = 8;

/********************************************
   Variables for prcessing RPM and direction
 ********************************************/
int hall_effect = 2;
volatile byte half_revolutions;
unsigned int rpm;
volatile int sig;
volatile int flag;
volatile int _dir;
unsigned long timeold;
String str;

// This code will be executed every time the interrupt 0 (pin2) gets low.
void counter()
{
  // For each rotation, this interrupt function runs twice
  half_revolutions++;
}

void setup()  // setup loop
{
  //initialize the serial link with processing
  Serial.begin(9600);

  pinMode(hall_effect, INPUT);
  pinMode(PWM, OUTPUT); // declares pin 12 as output
  pinMode(DIR, OUTPUT);

  attachInterrupt(0, counter, RISING);
  half_revolutions = 0;
  rpm = 0;
  timeold = 0;
  Setpoint = 0;
  
  //turn the PID on
  myPID.SetMode(AUTOMATIC);
}

void loop()
{
  // Write direction of motor(0 - forward, 1 - backwards)
  digitalWrite(DIR, _dir);
  
  // Hall Effect Sensor RPM calculation
  if (millis() - timeold >= 1000) { 
    //Don't process interrupts during calculations
    detachInterrupt(0);
   
    rpm = (30 * 1000) / (millis() - timeold) * half_revolutions;
    timeold = millis();
    half_revolutions = 0;
    
    //Restart the interrupt processing
    attachInterrupt(0, counter, RISING);
    Serial.print("RPM = ");
    Serial.println(rpm, DEC);
  }

  // Update RPM and compute PWM output
  Input = rpm;
  myPID.Compute();
  if(Setpoint == 0){
    Output = 0;
  }
  analogWrite(PWM, Output);
}

// Read serial input for RPM or direction
void serialEvent() {
  str = Serial.readStringUntil(':');
  Serial.println(str);
  flag = Serial.parseInt();
  if(str == "rpm"){
    Setpoint = flag;
  }
  else if(str == "dir"){
    _dir = flag;
  }
}
