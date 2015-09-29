// Central de sensores fundido com ...
// NeoPixel Ring simple sketch de Shae Erisson

#include <Adafruit_NeoPixel.h>
#ifdef __AVR__
#include <avr/power.h>
#endif
// Which pin on the Arduino is connected to the NeoPixels?
// On a Trinket or Gemma we suggest changing this to 1
#define PIN            6

// How many NeoPixels are attached to the Arduino?
#define NUMPIXELS      10

int delayval = 500; // delay for half a second

// When we setup the NeoPixel library, we tell it how many pixels, and which pin to use to send signals.
// Note that for older NeoPixel strips you might need to change the third parameter--see the strandtest
// example for more information on possible values.
Adafruit_NeoPixel strand = Adafruit_NeoPixel(NUMPIXELS, PIN, NEO_GRB + NEO_KHZ800);

#include <Wire.h>
#include <Adafruit_BMP085.h>
Adafruit_BMP085 bmp;

#include "DHT.h"

#define DHTTYPE DHT11
int dht_pin = 2; //Pino DATA do Sensor ligado no pino 2
DHT dht(dht_pin, DHTTYPE);

int til_pin = A1;

int mq4_pin = A7;

int tilt_pin = A6;

int buzzer = 3;

unsigned long currentTime, startTime, lastDisplayTime, displayInterval;

unsigned char i, kp;

int t[256];

void setup()
{
  Serial.begin(9600);

  dht.begin();

  strand.begin(); // This initializes the NeoPixel library.

  pinMode(til_pin, INPUT);
  pinMode(mq4_pin, INPUT);
  pinMode(tilt_pin, INPUT);
  pinMode(buzzer, OUTPUT);
  analogWrite(buzzer, 0);

  if (!bmp.begin()) {
    Serial.println("Could not find a valid BMP085 sensor, check wiring!");
    while (1) {
    }
  }
  startTime = millis();
  currentTime = startTime;
  lastDisplayTime = startTime;
  displayInterval = 2000;
  i = 0;
  kp = 0;
}

void loop()
{

  currentTime = millis();
  if ((currentTime - lastDisplayTime) > displayInterval) {
    lastDisplayTime = currentTime;
    float umidade = dht.readHumidity();
    float temperatura = dht.readTemperature();
    int luz = analogRead(til_pin);
    int metano = analogRead(mq4_pin);

    // Mostra os valores lidos, na serial
    Serial.print("elapsedTime=");
    Serial.print(currentTime - startTime);
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
    Serial.print("kp= ");
    Serial.println(kp);
  } else {
    t[i] = analogRead(tilt_pin);
    i++;

    // Some example procedures showing how to display to the pixels:
    switch (kp) {
      case 0: colorWipe(strand.Color(255, 0, 0), 50); break;
      case 1: colorWipe(strand.Color(0, 255, 0), 50); break;
      case 2: colorWipe(strand.Color(0, 0, 255), 50); break;
      // Send a theater pixel chase in...
      case 3: theaterChase(strand.Color(127, 127, 127), 50); break;
      case 4: theaterChase(strand.Color(127, 0, 0), 50); break;
      case 5: theaterChase(strand.Color(0, 0, 127), 50); break;

      case 6: rainbow(20); break;
      case 7: rainbowCycle(20); break;
      case 8: theaterChaseRainbow(50); break;
    }
    //pixels.show(); // This sends the updated pixel color to the hardware.

    //delay(delayval); // Delay for a period of time (in milliseconds).

    kp = (kp + 1) % 8;
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
  int min = t[0];
  int max = t[0];
  for (j = 1; j < 256; j++) {
    if (t[j] > max) max = t[j];
    if (t[j] < min) min = t[j];
  }
  return (max - min) > 100;
}

// Fill the dots one after the other with a color
void colorWipe(uint32_t c, uint8_t wait) {
  for(uint16_t i=0; i<strand.numPixels(); i++) {
    strand.setPixelColor(i, c);
    strand.show();
    delay(wait);
  }
}

void rainbow(uint8_t wait) {
  uint16_t i, j;

  for(j=0; j<256; j++) {
    for(i=0; i<strand.numPixels(); i++) {
      strand.setPixelColor(i, Wheel((i+j) & 255));
    }
    strand.show();
    delay(wait);
  }
}

// Slightly different, this makes the rainbow equally distributed throughout
void rainbowCycle(uint8_t wait) {
  uint16_t i, j;

  for(j=0; j<256*5; j++) { // 5 cycles of all colors on wheel
    for(i=0; i< strand.numPixels(); i++) {
      strand.setPixelColor(i, Wheel(((i * 256 / strand.numPixels()) + j) & 255));
    }
    strand.show();
    delay(wait);
  }
}

//Theatre-style crawling lights.
void theaterChase(uint32_t c, uint8_t wait) {
  for (int j=0; j<10; j++) {  //do 10 cycles of chasing
    for (int q=0; q < 3; q++) {
      for (int i=0; i < strand.numPixels(); i=i+3) {
        strand.setPixelColor(i+q, c);    //turn every third pixel on
      }
      strand.show();

      delay(wait);

      for (int i=0; i < strand.numPixels(); i=i+3) {
        strand.setPixelColor(i+q, 0);        //turn every third pixel off
      }
    }
  }
}

//Theatre-style crawling lights with rainbow effect
void theaterChaseRainbow(uint8_t wait) {
  for (int j=0; j < 256; j++) {     // cycle all 256 colors in the wheel
    for (int q=0; q < 3; q++) {
      for (int i=0; i < strand.numPixels(); i=i+3) {
        strand.setPixelColor(i+q, Wheel( (i+j) % 255));    //turn every third pixel on
      }
      strand.show();

      delay(wait);

      for (int i=0; i < strand.numPixels(); i=i+3) {
        strand.setPixelColor(i+q, 0);        //turn every third pixel off
      }
    }
  }
}

// Input a value 0 to 255 to get a color value.
// The colours are a transition r - g - b - back to r.
uint32_t Wheel(byte WheelPos) {
  WheelPos = 255 - WheelPos;
  if(WheelPos < 85) {
    return strand.Color(255 - WheelPos * 3, 0, WheelPos * 3);
  }
  if(WheelPos < 170) {
    WheelPos -= 85;
    return strand.Color(0, WheelPos * 3, 255 - WheelPos * 3);
  }
  WheelPos -= 170;
  return strand.Color(WheelPos * 3, 255 - WheelPos * 3, 0);
}
