/* Подключение библиотек */
#include <GyverPID.h>   // Библиотека для реализации PID-регулятора
#include <QTRSensors.h> // Библиотека для работы с QTR-датчиками
#include <Servo.h>      // Библиотека для управления сервоприводом

/* Определение пинов */
#define MOTOR_SPED 5 // Пин для управления скоростью двигателя
#define MOTOR_DIR 4  // Пин для управления направлением двигателя
#define BASESPED 42  // Базовая скорость движения
#define STOP 0       // Сигнал для остановки двигателя
#define TRIGGER 6    // Пин для подачи сигнала на триггер дальномера
#define ECHO 7       // Пин для приема сигнала от дальномера

/* Инициализация объектов */
GyverPID regulator(1.8, 0.4, 1.6, 10); // Объект PID-регулятора
QTRSensors qtr;                        // Объект QTR-датчиков
Servo myServo;                         // Объект сервопривода

/* Константы и переменные */
const uint8_t SensorCount = 10;     // Количество используемых датчиков
uint16_t sensorValues[SensorCount]; // Массив для хранения значений с датчиков
int pos = 0;                        // Позиция сервопривода
int perecrestok;                    // Значение с аналогового датчика
int perecrestok2;                   // Значение с аналогового датчика
int dist;                           // Расстояние, измеренное датчиком
int angle;                          // Угол поворота сервопривода
int centerAngle = 89;               // Центральный угол сервопривода
float u;                            // Управляющее воздействие
int sped = 30;                      // Скорость движения мотора                                                            //МЕНЯТЬ
int porog = 250;                    // Пороговое значение для аналогового датчика
int sens = 0;                       // Значение с ИК-датчика
int sensor = 0;                     // Значение с ИК-датчика
int lastFiltr = 0;                  // Последнее считанное значение с коммуникационного порта
int filtr = 0;                      // Значение фильтра

bool flag = true;       // Флаг для сигнала стоп
bool flagPoint = false; // Флаг для задания установочной точки
bool light = true;      // Флаг для светофора, реагирующего один раз
bool stopLine = false;  // Флаг для стоп-линии
bool smes = false;      // Флаг для смещения

/* Функция считывания значения с ИК-датчика */
int irDa()
{
    if (Serial3.available() > 0)
    {
        lastFiltr = Serial3.read();
        if (lastFiltr > -1 && lastFiltr < 7)
        {
            sensor = lastFiltr;
        }
    }
    return sensor;
}

void setup()
{
    /* Инициализация сериального порта */
    Serial3.begin(115200); // Настройка коммуникации с ИК-датчиком
    Serial.begin(9600);    // Настройка сериального порта для отладки

    /* Настройка пинов */
    pinMode(MOTOR_SPED, OUTPUT); // Установка пина на вывод
    pinMode(MOTOR_DIR, OUTPUT);  // Установка пина на вывод
    pinMode(TRIGGER, OUTPUT);    // Установка пина на вывод
    pinMode(ECHO, OUTPUT);       // Установка пина на вывод
    myServo.attach(11);          // Привязка сервопривода к пину 11

    /* Настройка QTR-датчиков */
    qtr.setTypeAnalog();                                                                             // Установка типа QTR-датчиков
    qtr.setSensorPins((const uint8_t[]){A15, A14, A13, A12, A11, A10, A9, A8, A7, A6}, SensorCount); // Задание пинов для QTR-датчиков

    /* Настройка PID-регулятора */
    regulator.setDirection(REVERSE); // Задание направления работы PID-регулятора
    regulator.setLimits(-35, 35);    // Задание границ задания для PID-регулятора
    regulator.setpoint = 0;          // Задание установочной точки для PID-регулятора

    /* Калибровка датчиков */
    digitalWrite(LED_BUILTIN, HIGH); // Включение светодиода Arduino для индикации режима калибровки
    for (uint16_t i = 0; i < 200; i++)
    {
        qtr.calibrate();
        delay(10);
    }
    digitalWrite(LED_BUILTIN, LOW); // Выключение светодиода Arduino после калибровки
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
}

/* Функция измерения расстояния */
int distant(unsigned long limit)
{
    unsigned long lim = limit * 58; // Преобразование лимита из времени в расстояние
    int cm;
    digitalWrite(TRIGGER, LOW);
    delayMicroseconds(2);
    digitalWrite(TRIGGER, HIGH);
    delayMicroseconds(10);
    digitalWrite(TRIGGER, LOW);

    cm = int(pulseIn(ECHO, HIGH, lim) / 58); // Измерение времени прохождения звука и преобразование его в расстояние

    if (cm == 0)
    {
        return int(limit); // Возвращение лимита, если измерение не удалось
    }
    return cm;
}

void loop()
{
    sens = irDa();                                       // Считывание значения с ИК-датчика
    dist = distant(100);                                 // Измерение расстояния
    perecrestok = analogRead(A6);                        // Считывание значения с аналогового датчика
    perecrestok2 = analogRead(A10);                      // Считывание значения с аналогового датчика
    uint16_t position = qtr.readLineBlack(sensorValues); // Считывание значений с QTR-датчиков

    /* Обработка сигналов от ИК-датчика и светофора */
    if ((sens == 0 || sens == 1 || sens == 4) && light == true)
    {
        if (perecrestok > porog && perecrestok2 > porog)
        {
            sped = STOP;
            stopLine = false;
        }
        else if (stopLine == true)
            sped = BASESPED - 11;
    }
    else if ((sens == 2 || sens == 3) && light == true)
    {
        sped = BASESPED + 7;
        stopLine = true;
    }

    pos = map(position, 0, 9000, -35, 35); // Преобразование значения позиции из диапазона [0, 9000] в диапазон [-35, 35]
    regulator.input = pos;                 // Задание входного сигнала для PID-регулятора
    u = regulator.getResultTimer();        // Получение управляющего воздействия от PID-регулятора
    u = int(u);
    angle = centerAngle + u; // Расчет угла поворота сервопривода

    if (angle > 123)
        angle = 123;
    else if (angle < 57)
        angle = 57; // Ограничение угла поворота сервопривода, чтобы не повредить его

    myServo.write(angle);          // Управление сервоприводом
    analogWrite(MOTOR_DIR, LOW);   // Управление направлением двигателя
    analogWrite(MOTOR_SPED, sped); // Управление скоростью двигателя
    delay(5);
}
