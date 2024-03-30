#include <QTRSensors.h>
#include <Servo.h>

#define MOTOR_SPED 5
#define MOTOR_DIR 4
#define BASESPED 40
#define STOP 0
#define TRIGGER 6
#define ECHO 7

//------------ irda
int sensor = 0;
int lastFilter = 0;

void setup()
{
    Serial3.begin(115200);
    Serial.begin(115200);
}

void loop()
{
    Serial.println(Serial3.read());
}
