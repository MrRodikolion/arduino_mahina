#include <QTRSensors.h>
#include <Servo.h> 

#define MOTOR_SPED 5
#define MOTOR_DIR 4
#define BASESPED 42
#define STOP 0
#define TRIGGER 6
#define ECHO 7

QTRSensors qtr;
Servo myServo;

const uint8_t SensorCount = 10;
uint16_t sensorValues[SensorCount];
int pos = 0;
int perecrestok;
int perecrestok2;
int dist;
int angle;  //угол поворота
int centerAngle = 89; //центральный угол
//float k = 0.9; //коэффицент пропорциональной состовляющей
//float kDif = 7;
float u; //управляющее воздействие
//float d;// дифференциальная, интегральная составляющая
int sped = 0; // скорость движения мотора
//int error = 0, errorOld;
int porog = 250;
int sens = 0;
int sensor = 0;
int lastFiltr = 0;
int filtr = 0;

bool flag = true;//флаг для знака стоп
bool flagPoint = false;//флаг для setpoint
bool light = true;//флаг для светофора который реагирует 1 раз
bool stopLine = false;//флаг для стоп линии
bool smes = false;

int irDa() {
  if (Serial3.available() > 0) {
    lastFiltr = Serial3.read();
    if (lastFiltr > -1 && lastFiltr < 7) {
      sensor = lastFiltr;
    }
  }
  return sensor;
}

void setup() {
  Serial3.begin(115200);
  pinMode (MOTOR_SPED, OUTPUT);
  pinMode (MOTOR_DIR, OUTPUT);
  pinMode(TRIGGER, OUTPUT);
  pinMode(ECHO, OUTPUT);
  myServo.attach(11);
  // configure the sensors
  qtr.setTypeAnalog();
  qtr.setSensorPins((const uint8_t[]) {
    A15, A14, A13, A12, A11, A10, A9, A8, A7, A6
  }, SensorCount);


  pinMode(LED_BUILTIN, OUTPUT);
  digitalWrite(LED_BUILTIN, HIGH); // turn on Arduino's LED to indicate we are in calibration mode
  for (uint16_t i = 0; i < 200; i++)
  {
    qtr.calibrate();
    delay (10);
  }
  digitalWrite(LED_BUILTIN, LOW); // turn off Arduino's LED to indicate we are through with calibration
  Serial.begin(9600);
  for (uint8_t i = 0; i < SensorCount; i++)
  {
    Serial.print(qtr.calibrationOn.minimum[i]);
    Serial.print(' ');
  }
  Serial.println();

  for (uint8_t i = 0; i < SensorCount; i++)
  {
    Serial.print(qtr.calibrationOn.maximum[i]);
    Serial.print(' ');
  }
  Serial.println();
  Serial.println();
  delay(1000);
  //sped = BASESPED;// put your setup code here, to run once:
}

int distant(unsigned long limit)// функция считывания датчика расстояния
{
  unsigned long lim = limit * 58;
  int cm;
  digitalWrite(TRIGGER, LOW);
  delayMicroseconds(2);
  digitalWrite(TRIGGER, HIGH);
  delayMicroseconds(10);
  digitalWrite(TRIGGER, LOW);

  cm =  int(pulseIn(ECHO, HIGH, lim) / 58);
  //delay(10);
  if (cm == 0) {
    return int(limit);
  }
  return cm;
}

void loop() {
  sens = irDa();
  dist = distant(100);
  perecrestok = analogRead(A6);
  perecrestok2 = analogRead(A10);
  uint16_t position = qtr.readLineBlack(sensorValues);


  pos = maprror = pos - centrLine;
//  Serial.print("u  ");(position, 0, 9000, -35, 35);
 Serial.println(pos);
  //int centrLine = 40;
  //e
//  Serial.println(u);

  if (angle > 123) angle = 123;
  else if (angle < 57) angle = 57; //серво чтоб не сгорел
  
  myServo.write(angle);
  analogWrite(MOTOR_DIR, LOW);
  analogWrite(MOTOR_SPED, sped);
  //  errorOld = error;
  delay(5);  
}
