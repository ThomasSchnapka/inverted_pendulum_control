#include "FastAccelStepper.h"
#include "AVRStepperPins.h"
#include <Encoder.h>

#define PULSE_PER_REV 800
#define MAX_STEP_ACCELERATION 60000
#define MAX_STEP_VELOCITY 12000
#define GEAR_DIAMETER 12.73//in mm
#define UPDATE_TIME 10 //in ms

#define ENA 5
#define DIR 6
#define PUL 7
#define ENDSTOP 21
#define ENCODER_A 2
#define ENCODER_B 3

//control parameters
const float M = 1;  // mass of cart for translation from force to acceleration
float alpha = 0.75; //filter value for encoder

//other variables
float state[] = {0, 0, 0, 0};
float x_last = 0;
float x_max = 0;
float theta_last = 0;
float u = 0;
float last_update = 0;
bool led_state = HIGH;

FastAccelStepperEngine engine = FastAccelStepperEngine();
FastAccelStepper *stepper = NULL;
Encoder EncoderPendulum(ENCODER_A, ENCODER_B);

void setup() {
  pinMode(ENDSTOP, INPUT_PULLUP);
  pinMode(LED_BUILTIN, OUTPUT);
  attachInterrupt(digitalPinToInterrupt(ENDSTOP), endstop_touched, LOW);
  // initialisation of stepper
  engine.init();
  stepper = engine.stepperConnectToPin(PUL);
  if (stepper) {
    stepper->setDirectionPin(DIR);
    stepper->setEnablePin(ENA);
    stepper->setAutoEnable(false);
    stepper->enableOutputs();
    stepper->setSpeedInHz(MAX_STEP_VELOCITY);  // the parameter is us/step !!! //MAX SPEED
  }
  Serial.begin(115200);
  Serial.setTimeout(1);
  // set state (including derivatives) to initial value
  update_state();
  update_state();
  
  // continue only if angle is zero --> start position
  /*
  do{
    update_state();
    delay(10);
  }while(abs(state[0]) > 1.0);
  */
}

void loop() {
  u = get_mpc_output(u);
  if((millis() - last_update) > UPDATE_TIME){
    update_state();
    if(abs(state[0]) < 50){
      //set_u(u);
      set_a(u);
    }else{
      stepper->stopMove();
    }
    //Serial.println(String(state[0]) + " " + String(state[2]) + " " + String(u));
    last_update = millis();
  }
}

void update_state(){
  state[0] = get_theta();
  state[1] = (state[0] - theta_last)*1000.0/UPDATE_TIME;
  theta_last = state[0];
  state[2] = get_x();
  state[3] = get_x_dot();// (state[2] - x_last)*1000/UPDATE_TIME;
  //x_last = state[2];
}

float get_mpc_output(float u){
  // exchange data with MPC controller. Return old u, if no data is available
  while (Serial.available() > 0){
    u = Serial.readString().toFloat();
    Serial.print(String(state[0]*(2*PI)/360.0, 3) + " " + String(state[1]*(2*PI)/360.0, 3) + " " + String(state[2], 3) + " " + String(state[3], 3) + " " + String(u));
  }
  return u;
}

void set_u(float u){
  // set force u pushing the cart
  // force is translated to angular acceleration in stepper ticks
  float a = u/M;                  //translational acceleration
  a = a/(2*PI*0.5*GEAR_DIAMETER*1000); //rotational acceleration
  a = a*PULSE_PER_REV;     //acceleretion in steps
  a = limit_value_to(a, MAX_STEP_ACCELERATION);
  //Serial.println("Setting a to " + String(a));
  stepper->moveByAcceleration(a);
}

void set_a(float a){
  a = limit_value_to(a, MAX_STEP_ACCELERATION);
  stepper->moveByAcceleration(a);
}

float get_x(){
  float x = stepper->getCurrentPosition(); // in steps
  x = x/PULSE_PER_REV;                     // in rev
  x = x*(2*PI*0.5*GEAR_DIAMETER*1000);     // in m
  return x;
}

float get_x_dot(){
  float x = stepper->getCurrentSpeedInMilliHz()*1000; // in steps/s
  x = x/PULSE_PER_REV;                                // in rev/s
  x = x*(2*PI*0.5*GEAR_DIAMETER*1000);                // in m/s
  return x;
}

float get_theta(){
  int32_t ticks = EncoderPendulum.read();
  ticks = ticks%(4*600);
  float theta = 360.0*ticks/(4*600);
  //initial positin is rod facing ground, but zero deg is defined as rod facing upwards
  theta = theta-180.0;
  //filter
  theta = alpha*theta + (1.0-alpha)*theta_last;
  return theta;
}


void endstop_touched(){
  digitalWrite(LED_BUILTIN, HIGH);
  stepper->disableOutputs();
}

float limit_value_to(float val, float max_val){
  // perform min() considering the value's sign
  int sign = abs(val)/val;
  val = min(abs(val), max_val);
  val *= sign;
  return val;
}

