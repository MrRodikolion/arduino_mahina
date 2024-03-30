#include <QTRSensors.h>
#include <Servo.h>

#define MOTOR_SPED 5
#define MOTOR_DIR 4
#define BASESPED 30
#define STOP 0
#define TRIGGER 6
#define ECHO 7

//------------ qtr sensor
QTRSensors qtr;

const uint8_t SensorCount = 10;
uint16_t sensorValues[SensorCount];
int pos = 0;

uint16_t position;

long lines[3][2];

//------------ servo
Servo myServo;

//------------ motor
int speed = BASESPED;

//------------ pid
class PID
{

    float p, i, d;
    float old_error = 0;
    float integral = 0;

    float start_time;

public:
    PID(float kp, float ki, float kd)
    {
        p = kp;
        i = ki;
        d = kd;

        start_time = millis() / 1000;
    }

    int get_angle(int centr)
    {
        int error = centr - 4500;

        integral += error * i;
        float integ = integral / (start_time - millis() / 1000);

        int derivative = error - old_error;

        int angle = (p * error + integ + d * derivative) + 90;

        old_error = error;

        return min(max(angle, 57), 123);
    }
};

PID pid{0.02, 0.002, 0.02};

int angle = 90;

void setup()
{
    Serial.begin(9600);
    // setup pins
    pinMode(MOTOR_SPED, OUTPUT);
    pinMode(MOTOR_DIR, OUTPUT);
    pinMode(TRIGGER, OUTPUT);
    pinMode(ECHO, OUTPUT);
    myServo.attach(11);

    // qtr setup
    qtr.setTypeAnalog();
    qtr.setSensorPins((const uint8_t[]){
                          A15, A14, A13, A12, A11, A10, A9, A8, A7, A6},
                      SensorCount);

    // qtr calibration
    pinMode(LED_BUILTIN, OUTPUT);
    digitalWrite(LED_BUILTIN, HIGH);
    for (uint16_t i = 0; i < 200; i++)
    {
        qtr.calibrate();
        delay(10);
    }
    digitalWrite(LED_BUILTIN, LOW);

    Serial.println("start");

    delay(500);
}

void loop()
{
    // get centr line
    position = qtr.readLineBlack(sensorValues);

    // size_t line_start = 0, line_end = 0;

    // size_t line_n = 0;
    // int line_len = 0;
    // for (size_t i = 0; i < SensorCount; ++i)
    // {
    //     if (line_n >= 3)
    //     {
    //         break;
    //     }
    //     if (sensorValues[i] > qtr.calibrationOn.maximum[i])
    //     {
    //         line_len++;
    //         if (!line_start)
    //         {
    //             line_start = i + 1;
    //         }
    //     }
    //     else if (sensorValues[i] < qtr.calibrationOn.maximum[i] && line_start)
    //     {
    //         line_end = i;

    //         // long upsum = 0;
    //         // long btmsum = 0;

    //         // for (size_t j = line_start - 1; j < line_end; ++j)
    //         // {
    //         //     upsum += sensorValues[j] * (1000 * j);
    //         //     btmsum += sensorValues[j];
    //         // }
    //         lines[line_n][0] = line_len;
    //         lines[line_n][1] = (line_start + line_end) / 2;

    //         line_n++;
    //         line_start = 0;
    //         line_end = 0;

    //         line_len = 0;
    //     }
    // }
    // if (line_start)
    // {
    //     line_end = 10;

    //     // long upsum = 0;
    //     // long btmsum = 0;

    //     // for (size_t j = line_start - 1; j < line_end; ++j)
    //     // {
    //     //     upsum += sensorValues[j] * (1000 * j);
    //     //     btmsum += sensorValues[j];
    //     // }
    //     lines[line_n][0] = line_len;
    //     lines[line_n][1] = (line_start + line_end) / 2;
    //     line_n++;
    // }

    // // for (auto i : sensorValues)
    // // {
    // //     Serial.print(i);
    // //     Serial.print(' ');
    // // }
    // // Serial.println();

    // int cool_line = 0;
    // for (size_t i = 0; i < line_n; ++i)
    // {
    //     auto lin = lines[i];

    //     if (abs(cool_line - 5) > abs(lin[1] - 5))
    //     {
    //         cool_line = lin[1];
    //     }
    //     // Serial.print("(");
    //     // Serial.print(lin[0]);
    //     // Serial.print(", ");
    //     // Serial.print(lin[1]);
    //     // Serial.print(") ");
    // }
    // // Serial.print("    (");
    // // Serial.print(cool_line[0]);
    // // Serial.print(", ");
    // // Serial.print(cool_line[1]);
    // // Serial.print(") ");
    // // Serial.println();

    // pos = cool_line;


    // get angle
    angle = pid.get_angle(position);

    // write
    myServo.write(angle);
    analogWrite(MOTOR_DIR, LOW);
    analogWrite(MOTOR_SPED, speed);

    delay(50);
}

Serial3.begin(115200);//это в void setup()

//эти переменные обозначать вне функций
int sensor = 0;
int lastFilter = 0;


void irDa() {//это просто функция никуда не надо
    if(Serial3.available() > 0) {
        lastFilter = Serial3.read();
        if(lastFilter > -1 && lastFilter < 7) {
            sensor = lastFilter;
        }
    }
}
