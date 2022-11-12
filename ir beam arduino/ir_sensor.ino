#include <Arduino.h>

int sensor = 15;
int value = 0;
bool isOverSensor = false;
int count = 0;

void setup() {
  //start COM over USB for debug data
  Serial.begin(115200);

  pinMode(sensor, INPUT);
  pinMode(LED_BUILTIN, OUTPUT);
}

// the loop function runs over and over again forever
void loop() {
  //recieve the GPS packet
  value = analogRead(sensor);
  Serial.println(value);

    // this value will need to be fine tuned so we don't get false positives
    // seems like it peaks to about 1000 when the blade is completely over the sensor
    // at around 5ish mm

  if (value > 900)
  {
    digitalWrite(LED_BUILTIN, HIGH);
    if (isOverSensor == false)
    {
        isOverSensor = true;
        count++;
        // this might be important. I noticed we get less false positives when we add a delay
        // might be good to figure out a Niquist sampling rate so we can keep the processing
        // power needed low
        delay(20);
    }
  }
  else
  {
    digitalWrite(LED_BUILTIN, LOW);
    isOverSensor = false;
    Serial.println(count);
  }
    
}
