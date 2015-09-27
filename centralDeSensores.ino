
#include <Wire.h>
#include <Adafruit_BMP085.h>
Adafruit_BMP085 bmp;

#include "DHT.h"

#define DHTTYPE DHT11
int dht_pin=2; //Pino DATA do Sensor ligado no pino 2
DHT dht(dht_pin, DHTTYPE);

int til_pin = A1;

int mq4_pin = A7;

int tilt_pin = A6;

int buzzer = 3;

unsigned long currentTime, startTime, lastDisplayTime, displayInterval;

unsigned char i;

int t[256];

void setup()
{
  Serial.begin(9600);

  dht.begin();

  pinMode(til_pin, INPUT);
  pinMode(mq4_pin, INPUT);
  pinMode(tilt_pin, INPUT);
  pinMode(buzzer, OUTPUT);
  analogWrite(buzzer,0);

  if (!bmp.begin()) {
    Serial.println("Could not find a valid BMP085 sensor, check wiring!");
    while (1) {
    }
  }
  startTime=millis();
  currentTime=startTime;
  lastDisplayTime=startTime;
  displayInterval=2000;
  i=0;
}

void loop()
{
  currentTime=millis();
  if ((currentTime-lastDisplayTime)>displayInterval) {
    lastDisplayTime=currentTime;
  float umidade = dht.readHumidity();
  float temperatura = dht.readTemperature();
  int luz = analogRead(til_pin);
  int metano = analogRead(mq4_pin);

  // Mostra os valores lidos, na serial
  Serial.print("elapsedTime=");
  Serial.print(currentTime-startTime);
  Serial.println ("ms");  
  Serial.print("Temp(DHT11) = ");
  Serial.print(temperatura);
  Serial.println(" C ");
  Serial.print("Um. = ");
  Serial.print(umidade);
  Serial.println(" % ");
  Serial.print("luz = ");
  Serial.println(luz);
  Serial.print("metano = ");
  Serial.println(metano);


  Serial.print("Temp(BMP180) = ");
  Serial.print(bmp.readTemperature());
  Serial.println(" *C");

  Serial.print("Pressure = ");
  Serial.print(bmp.readPressure());
  Serial.println(" Pa");

  Serial.print("Tilt = ");
  Serial.println(analogRead(tilt_pin));
  if (shaking()) {
    Serial.println ("EARTHQUAKE!!!");
    analogWrite (buzzer, 5);
  } else {
    analogWrite (buzzer, 0);
  }
  Serial.println(" ");
  } else{
    t[i]=analogRead(tilt_pin);
    i++;
  }
  // em 08.09.2015, 16h30 a pressao em santos eh 1005hPa(http://www.cptec.inpe.br/cidades/tempo/4748).
  // em 08.09.2015, 18h06 a pressao na minha sala eh 923.12 (nao sei se o sensor eh tao preciso)
  // a altitude calculada com base nas medidas de pressao eh 710.53.


  // Nao diminuir muito o valor abaixo
  // O ideal e a leitura a cada 2 segundos
  // delay(2000);
}


int shaking (void) {
  int j;
  int min=t[0];
  int max=t[0];
  for (j=1;j<256;j++) {
     if (t[j]>max) max=t[j];
     if (t[j]<min) min=t[j];
  }
  return (max-min)>100;
}


