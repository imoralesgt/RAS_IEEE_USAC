//Controlador PID discreto
//Implementado sin punto flotante

#include <Ultrasonic.h>

const byte PWM1 = 3;
const byte PWM2 = 4;
const byte TRIGGER_PIN = 8;
const byte ECHO_PIN = 7;

const int MAX_PWM = 120;

const unsigned int SP = 50;


float Kp, Ki, Kd, minInt, maxInt;
int setPoint;
float derivator = 0, integrator = 0;
int error = 0;
float pValue = 0, iValue = 0, dValue = 0;

Ultrasonic ping(TRIGGER_PIN, ECHO_PIN); //Ultrasonic HC-SR04 distance sensor

void pidInit(float p, float i, float d, float minI, float maxI){
   Kp = p; Ki = i; Kd = d; //Coeficientes del controlador PID
   minInt = minI; maxInt = maxI; //Limites del integrador
   
   derivator = 0; integrator = 0; //Condiciones iniciales
   error = 0; //Condiciones iniciales
}

void pidSetPoint(int set){
  setPoint = set; //Establecer valor deseado
  //derivator = 0; //Inicializar condiciones iniciales
  //integrator = 0; //Eliminar todo error acumulado al reiniciar
}

int pidUpdate(float currentValue){
  float PID;
  error = setPoint - currentValue; 

  //Parte proporcional del controlador
  pValue = Kp*error;
  
  //Integral discreta
  integrator = integrator + error;

  if (integrator > maxInt){
    integrator = maxInt;
  }else if(integrator < minInt){
    integrator = minInt;
  }
  
  //Parte integral del controlador
  iValue = integrator*Ki;
  
  
  //Derivada discreta
  dValue = Kd*(error - derivator);
  derivator = error; //Diferencia del valor anterior con el valor actual
/*
  Serial.print("\n\nError: ");
  Serial.println(error);
  Serial.println("P,  I,  D");
  Serial.print(pValue);
  Serial.print(", ");
  Serial.print(iValue);
  Serial.print(", ");
  Serial.println(dValue);
*/
  PID = pValue + iValue + dValue;

  if(PID > (MAX_PWM - 1)){
    PID = MAX_PWM - 1;
  }else if(PID < ((-1)*MAX_PWM + 1)){
    PID = ((-1)*MAX_PWM + 1);
  }

  
  Serial.print("\n\nPID Value: ");
  Serial.println(PID);
  
  return (int)PID;
}


void setup()
{
  
  pinMode(PWM1, OUTPUT);
  pinMode(PWM2, OUTPUT);  

  digitalWrite(PWM1, 0);
  digitalWrite(PWM2, 0);
  
  ping.init(); //Initialize Ultrasonic sensor
  
  Serial.begin(115200);
  
  
  //Extremadamente importante calibrar adecuadamente los coeficiente
  pidInit(-8, -0.5, -1, (-1)*MAX_PWM, MAX_PWM); //Coeficientes y limites de integrador
  pidSetPoint(SP); //Posicion deseada
}

uint16_t leePosicion(){
  return ping.distance(1);
}

void driveMotors(int pwm){
  if(pwm >=0){
    analogWrite(PWM1, pwm);
    digitalWrite(PWM2, 0);
    analogWrite(GREEN_LED, pwm);
    digitalWrite(RED_LED, 0);
  }else{
    pwm = 255 + pwm;
    digitalWrite(PWM2, 1);
    analogWrite(PWM1, pwm);    
    digitalWrite(GREEN_LED, 0);
    analogWrite(RED_LED, pwm);
  }
  Serial.print("\n\nPWM MOTORES:  ");
  Serial.println(pwm);
}

void loop()
{
  uint16_t pos;
  int16_t pwmOut;
  pos = leePosicion();
  Serial.print("Posicion: ");
  Serial.println(pos);

  uint16_t sp;
  sp = SP;
  pidSetPoint(sp); //Actualizar set point  
  
  Serial.print("Set point: ");
  Serial.println(sp);
  
  pwmOut = pidUpdate(pos); //Actualizar el sistema de control
                            //y obtener nuevo valor

  driveMotors(pwmOut);
/*
  Serial.println(" ");  
  Serial.print("PIDval: ");
  Serial.println(pwmOut);
*/
  delay(5);
  
}  
