#include <QTRSensors.h>
#include <Servo.h>

#define MOTOR_SPED 5
#define MOTOR_DIR 4
#define BASESPED 25
#define STOP 0
#define TRIGGER 6
#define ECHO 7

//------------ forward2back
bool is_back = false;
bool is_done = false;

int perecrestok, perecrestok2;
int porog = 250;

//------------ svet
bool is_red = true;
bool is_stop_lin = true;

//------------ irda
int sensor = 2;
int lastFilter = 0;

//------------ qtr sensor forvard
QTRSensors qtr_forward;

const uint8_t forward_SensorCount = 10;
uint16_t sensorValues[forward_SensorCount];
int pos = 0;
int old_pos = 0;

uint16_t fposition, bposition;

int lines[3][2];

size_t line_n = 0;
int line_len = 0;

//------------ qtr sensor back
QTRSensors qtr_back;

const uint8_t back_SensorCount = 2;
uint16_t back_sensorValues[back_SensorCount];

//------------ servo
Servo myServo;

//------------ motor
int speed = BASESPED;

//------------ pid
class PID {

  float p, i, d;
  float old_error = 0;
  float integral = 0;

  float start_time;

  float fp, fi, fd;
  float old_ferror = 0;

public:
  PID(float kp, float ki, float kd, float kfp, float kfd) {
    p = kp;
    i = ki;
    d = kd;
    fp = kfp;
    fd = kfd;

    start_time = millis();
  }

  int get_angle(int bcentr, int fcentr) {
    float error = bcentr - 500;

    float ferror = fcentr - 4500;

    integral += error * i;
    float integ = integral / (start_time - millis());

    float derivative = error - old_error;

    float fderivative = ferror - old_ferror;

    int angle = (p * error + integ + d * derivative + ferror * fp + fderivative * fd) + 90;

    old_error = error;

    return min(max(angle, 57), 123);
  }
};

class PID_F {

  float p, i, d;
  float old_error = 0;
  float integral = 0;

  float start_time;

public:
  PID_F(float kp, float ki, float kd) {
    p = kp;
    i = ki;
    d = kd;

    start_time = millis();
  }

  int get_angle(int centr) {
    int error = centr - 4500;

    integral += error * i;
    float integ = integral / (start_time - millis());

    int derivative = error - old_error;

    int angle = (p * error + integ + d * derivative) + 90;

    old_error = error;

    return min(max(angle, 57), 123);
  }
};

PID back_pid{ 1, 0.5, 1, 0.015, 0.03 };
PID_F pid{ 0.015, 0.003, 0.03 };

int angle = 90;

void setup() {
  Serial.begin(9600);
  // setup pins
  pinMode(MOTOR_SPED, OUTPUT);
  pinMode(MOTOR_DIR, OUTPUT);
  pinMode(TRIGGER, OUTPUT);
  pinMode(ECHO, INPUT);
  myServo.attach(11);

  pinMode(LED_BUILTIN, OUTPUT);
  // forward
  // qtr setup
  qtr_forward.setTypeAnalog();
  qtr_forward.setSensorPins((const uint8_t[]){
                              A15, A14, A13, A12, A11, A10, A9, A8, A7, A6 },
                            forward_SensorCount);

  // qtr calibration
  digitalWrite(LED_BUILTIN, HIGH);
  for (uint16_t i = 0; i < 100; i++) {
    qtr_forward.calibrate();
    delay(10);
  }
  digitalWrite(LED_BUILTIN, LOW);

  delay(500);

  // back
  // qtr setup
  qtr_back.setTypeAnalog();
  qtr_back.setSensorPins((const uint8_t[]){
                           A3, A2 },
                         back_SensorCount);

  // qtr calibration
  digitalWrite(LED_BUILTIN, HIGH);
  for (uint16_t i = 0; i < 100; i++) {
    qtr_back.calibrate();
    delay(10);
  }
  digitalWrite(LED_BUILTIN, LOW);

  delay(500);
}

void loop() {
  if (is_back) {
    // if (is_done) {
    //   return;
    // }
    back_line();

    // perecrestok = analogRead(A6);
    // perecrestok2 = analogRead(A10);
    // if ((perecrestok > porog && perecrestok2 > porog) || (back_sensorValues[0] + back_sensorValues[1] > 850)) {
    //   is_done = true;
    //   speed = 0;
    // }
  } else {
    forward_line();
    if (distant(100) < 20) {
      is_back = true;
    }
  }
}

void back_line() {
  // get centr line
  bposition = qtr_back.readLineBlack(back_sensorValues);

  fposition = qtr_forward.readLineBlack(sensorValues);

  // for (auto val : back_sensorValues) {
  //     Serial.print(val); Serial.print(", ");
  // }
  // Serial.println(bposition);

  // get angle
  angle = back_pid.get_angle(bposition, fposition);

  Serial.println(bposition);

  // write
  myServo.write(angle);
  analogWrite(MOTOR_DIR, speed);
  analogWrite(MOTOR_SPED, LOW);

  delay(5);
}

void forward_line() {
  irDa();
  speed = BASESPED;
  if (is_red) {
    speed = 0;
  }

  // get centr line
  // qtr.readLineBlack
  qtr_forward.readCalibrated(sensorValues, QTRReadMode::On);

  find_centers();

  pos = get_cool_line();
  old_pos = pos;

  // get angle
  angle = pid.get_angle(pos);

  // write
  myServo.write(angle);
  analogWrite(MOTOR_DIR, LOW);
  analogWrite(MOTOR_SPED, speed);

  delay(5);
}

int distant(unsigned long limit)  // функция считывания датчика расстояния
{
  unsigned long lim = limit * 58;
  int cm;
  digitalWrite(TRIGGER, LOW);
  delayMicroseconds(2);
  digitalWrite(TRIGGER, HIGH);
  delayMicroseconds(10);
  digitalWrite(TRIGGER, LOW);

  cm = int(pulseIn(ECHO, HIGH, lim) / 58);
  // delay(10);
  if (cm == 0) {
    return int(limit);
  }
  return cm;
}

void irDa() {
  if (Serial3.available() == 0) {
    lastFilter = Serial3.read();
    if (lastFilter > -1 && lastFilter < 7) {
      sensor = lastFilter;
    }
  }
  if (sensor == 0 || sensor == 1 || sensor == 4) {
    is_red = true;
  } else if (sensor == 2 || sensor == 3) {
    is_red = false;
  }
}

void find_centers() {
  size_t line_start = 0, line_end = 0;

  line_n = 0;
  line_len = 0;
  for (size_t i = 0; i < forward_SensorCount; ++i) {
    if (line_n >= 3) {
      break;
    }
    if (sensorValues[i] > qtr_forward.calibrationOn.maximum[i]) {
      line_len++;
      if (!line_start) {
        line_start = i + 1;
      }
    } else if (sensorValues[i] < qtr_forward.calibrationOn.maximum[i] && line_start) {
      line_end = i;

      uint32_t avg = 0;
      uint16_t sum = 0;

      for (uint8_t i = line_start - 1; i < line_end; i++) {
        uint16_t value = sensorValues[i];

        if (value > 50) {
          avg += (uint32_t)value * (i * 1000);
          sum += value;
        }
      }
      lines[line_n][0] = line_len;
      lines[line_n][1] = min(avg / sum, 9000);
      // Serial.println(avg / sum);

      line_n++;
      line_start = 0;
      line_end = 0;

      line_len = 0;
    }
  }
  if (line_start) {
    line_end = 9;

    uint32_t avg = 0;
    uint16_t sum = 0;

    for (uint8_t i = line_start - 1; i < line_end; i++) {
      uint16_t value = sensorValues[i];

      if (value > 50) {
        avg += (uint32_t)value * (i * 1000);
        sum += value;
      }
    }
    lines[line_n][0] = line_len;
    lines[line_n][1] = min(avg / sum, 9000);
    // Serial.println(avg / sum);
    line_n++;
  }
}

int get_cool_line() {
  int cool_line = old_pos;
  for (size_t i = line_n - 1; i >= 0; --i) {
    auto lin = lines[i];

    if (lin[0] <= 3) {
      cool_line = lin[1];
      break;
    }
    is_stop_lin = true;
  }

  return cool_line;
}