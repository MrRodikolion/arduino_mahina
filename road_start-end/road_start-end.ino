#include <QTRSensors.h>
#include <Servo.h>

#define MOTOR_SPED 5
#define MOTOR_DIR 4
#define BASESPED 50
#define STOP 0
#define TRIGGER 6
#define ECHO 7

//------------ svet
bool is_red = true;
bool is_stop_lin = true;

int dist = 0;

//------------ irda
int sensor = 0;
int lastFilter = 0;

//------------ qtr sensor
QTRSensors qtr;

const uint8_t SensorCount = 10;
uint16_t sensorValues[SensorCount];
int sensetivity = 1000;
int pos = 0;

bool is_line_passing = false;
bool is_line_passed = false;

uint16_t position;

struct line_data {
  uint32_t pos = 0;
  size_t start = 0, end = 0;

  int len() {
    return end - start;
  }
};

line_data lines[2];
line_data old_line;

size_t line_n = 0;
int line_len = 0;

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

public:
  PID(float kp, float ki, float kd) {
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

PID pid{ 0.015, 0.003, 0.03 };  //{15, 15, 30};

int angle = 90;

void setup() {
  Serial3.begin(115200);

  Serial.begin(9600);
  Serial.println("ok");
  // setup pins
  pinMode(MOTOR_SPED, OUTPUT);
  pinMode(MOTOR_DIR, OUTPUT);
  pinMode(TRIGGER, OUTPUT);
  pinMode(ECHO, INPUT);
  myServo.attach(11);

  // qtr setup
  qtr.setTypeAnalog();
  qtr.setSensorPins((const uint8_t[]){
                      A15, A14, A13, A12, A11, A10, A9, A8, A7, A6 },
                    SensorCount);

  // qtr calibration
  pinMode(LED_BUILTIN, OUTPUT);
  digitalWrite(LED_BUILTIN, HIGH);
  for (uint16_t i = 0; i < 200; i++) {
    qtr.calibrate();
    delay(10);
  }
  digitalWrite(LED_BUILTIN, LOW);

  Serial.println("start");

  delay(500);
}

void loop() {
  irDa();
  speed = BASESPED;
  // dist = distant(100);

  // Serial.println(is_red);

  if ((is_red && is_stop_lin)) {
    speed = 0;
  } else if (is_red) {
    speed /= 2;
  }

  // get centr line
  // qtr.readLineBlack
  qtr.readCalibrated(sensorValues, QTRReadMode::On);

  is_stop_lin = false;
  find_centers();

  pos = get_cool_line().pos;

  if (!is_line_passed) {
    if (line_n <= 1 && is_line_passing) {
      is_line_passed = true;
    } else if (line_n > 1) {
      is_line_passing = true;
    }
  }

  // get angle
  angle = pid.get_angle(pos);

  // write
  myServo.write(angle);
  analogWrite(MOTOR_DIR, LOW);
  analogWrite(MOTOR_SPED, speed);

  delay(5);
}

void irDa() {
  if (Serial3.available() > 0) {
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
  if (cm == 0) {
    return int(limit);
  }
  return cm;
}

void find_centers() {
  size_t line_start = 0, line_end = 0;

  line_n = 0;
  line_len = 0;
  for (size_t i = 0; i < SensorCount; ++i) {
    if (line_n >= 2) {
      break;
    }
    if (sensorValues[i] > qtr.calibrationOn.maximum[i]) {
      line_len++;
      if (!line_start) {
        line_start = i + 1;
      }
    } else if (sensorValues[i] < qtr.calibrationOn.maximum[i] && line_start) {
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
      lines[line_n].pos = min(avg / sum, 9000);
      lines[line_n].start = line_start;
      lines[line_n].end = line_end;

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
    lines[line_n].pos = min(avg / sum, 9000);
    lines[line_n].start = line_start;
    lines[line_n].end = line_end;
    line_n++;
  }
}

line_data get_cool_line() {
  line_data cool_line = old_line;
  if (!is_line_passed) {
    for (size_t i = line_n - 1; i >= 0; --i) {
      line_data lin = lines[i];

      if (lin.len() <= 3) {
        cool_line = lin;
        break;
      }
      is_stop_lin = true;
    }
  } else {
    for (size_t i = 0; i < line_n; ++i) {
      line_data lin = lines[i];

      if (lin.len() <= 3) {
        cool_line = lin;
        break;
      }
      is_stop_lin = true;

      // if (lin.len() > 3)
      // {
      //     is_stop_lin = true;
      //     continue;
      // }

      // if (abs(cool_line.pos - 4500) > abs(lin.pos - 4500))
      // {
      //     cool_line = lin;
      // }
    }
  }
  old_line = cool_line;
  return cool_line;
}