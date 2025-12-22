#include <QTRSensors.h>
#include <SoftwareSerial.h>

#define USE_BLUETOOTH_SERIAL

const int M1A = 3;
const int M1B = 9;
const int M2A = 10;
const int M2B = 11;

int maximum = 58;

int error = 0;
unsigned int last_proportional = 0;
long integral = 0;

float Kp = 1.6;
unsigned long Ki = 8500.0;
float Kd = 4.3;


const int NUM_SENSORS = 8;
const int NUM_SAMPLES_PER_SENSOR = 4;
const int EMITTER_PIN = 2;

QTRSensorsAnalog qtra((unsigned char[]) {
  A0, A1, A2, A3, A4, A5, A6, A7
}, NUM_SENSORS, NUM_SAMPLES_PER_SENSOR, EMITTER_PIN);

unsigned int sensorValues[NUM_SENSORS];

unsigned long timer = 0;
SoftwareSerial mySerial(4, 5);

void setup() {
  Serial.begin(9600);
  Serial.println("hello");

  mySerial.begin(9600);
  mySerial.println("hello");


  pinMode(M1A, OUTPUT);
  pinMode(M1B, OUTPUT);
  pinMode(M2A, OUTPUT);
  pinMode(M2B, OUTPUT);
  digitalWrite(M1A, HIGH);
  digitalWrite(M2A, HIGH);
  digitalWrite(M1B, HIGH);
  digitalWrite(M2B, HIGH);

  pinMode(LED_BUILTIN, OUTPUT);
  digitalWrite(LED_BUILTIN, HIGH);
  for (int i = 0; i < 400; i++)
  {
    qtra.calibrate();
  }
  digitalWrite(LED_BUILTIN, LOW);

  for (int i = 0; i < NUM_SENSORS; i++)
  {
    Serial.print(qtra.calibratedMinimumOn[i]);
    Serial.print(' ');
  }
  Serial.println();

  for (int i = 0; i < NUM_SENSORS; i++)
  {
    Serial.print(qtra.calibratedMaximumOn[i]);
    Serial.print(' ');
  }
  Serial.println();
  Serial.println();
  delay(1000);
}


void loop()
{
  unsigned int position = qtra.readLine(sensorValues);

  int proportional = (int)position - 3500;

  int derivative = proportional - last_proportional;
  integral += proportional;

  last_proportional = proportional;

  int power_difference = proportional / Kp + integral / Ki + derivative * Kd;

  if (power_difference > maximum)
    power_difference = maximum;
  if (power_difference < -maximum)
    power_difference = -maximum;


  if (power_difference < 0)
    set_motors(maximum + power_difference, maximum);
  else
    set_motors(maximum, maximum - power_difference);

#ifndef USE_BLUETOOTH_SERIAL
  if ((millis() - timer) > 2000) {
    mySerial.print(proportional);
    mySerial.print(',');
    mySerial.print(power_difference);
    mySerial.print(' ');
    mySerial.print(' ');
    mySerial.print(Kp);
    mySerial.print(',');
    mySerial.print(Ki);
    mySerial.print(',');
    mySerial.print(Kd);
    mySerial.print(',');
    mySerial.println(maximum);

    timer = millis();
  }
  if (mySerial.available() > 0) {
    int yyy = mySerial.read();
    mySerial.println(char(yyy));
    switch (yyy) {
      case 112:
        Kp = mySerial.parseFloat();
        mySerial.print("Kp set to: ");
        mySerial.println(Kp);
        break;
      case 105:
        Ki = mySerial.parseFloat();
        mySerial.print("Ki set to: ");
        mySerial.println(Ki);
        break;
      case 100:
        Kd = mySerial.parseFloat();
        mySerial.print("Kd set to: ");
        mySerial.println(Kd);
        break;
      case 109:
        maximum = mySerial.parseInt();
        mySerial.print("Speed set to: ");
        mySerial.println(maximum);
        break;
    }
  }
#endif
}

// speed1/speed2: -255..255
void set_motors(int speed1, int speed2) {
  byte M1ASpeed = speed1 > 0 ? speed1 : 0;
  byte M1BSpeed = speed1 > 0 ? 0 : speed1 * -1;
  analogWrite(M1A, M1ASpeed);
  analogWrite(M1B, M1BSpeed);

  byte M2ASpeed = speed2 > 0 ? speed2 : 0;
  byte M2BSpeed = speed2 > 0 ? 0 : speed2 * -1;
  analogWrite(M2A, M2ASpeed);
  analogWrite(M2B, M2BSpeed);
}
