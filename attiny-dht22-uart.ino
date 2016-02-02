/*
 ATMEL ATTINY 25/45/85 / ARDUINO
 Pin 1 is /RESET
                  +-\/-+
 Ain0 (D 5) PB5  1|    |8  Vcc
 Ain3 (D 3) PB3  2|    |7  PB2 (D 2) Ain1
 Ain2 (D 4) PB4  3|    |6  PB1 (D 1) pwm1
            GND  4|    |5  PB0 (D 0) pwm0
                  +----+
*/

#define F_CPU 8000000  // This is used by delay.h library

#include <stdlib.h>
#include <avr/io.h>
#include <util/delay.h>

#include <SoftwareSerial.h>
#define rxPin 5
#define txPin 3
SoftwareSerial serial(rxPin, txPin);

#include "TinyDHT.h"
#define DHTPOWERPIN 2
#define DHTPIN 4
#define DHTTYPE DHT22
DHT dht(DHTPIN, DHTTYPE);

char cmd[64];

void setup()
{
  TCCR0A = 2<<COM0A0 | 2<<COM0B0 | 3<<WGM00;
  TCCR0B = 0<<WGM02  | 1<<CS00;
  TCCR1  = 0<<PWM1A  | 0<<COM1A0 | 1<<CS10;
  GTCCR  = 1<<PWM1B  | 2<<COM1B0;

  pinMode(rxPin, INPUT);
  pinMode(txPin, OUTPUT);
  pinMode(DHTPOWERPIN, OUTPUT);

  digitalWrite(DHTPOWERPIN, LOW);

  serial.begin(4800);
  delay(100);
}

void loop()
{

  digitalWrite(DHTPOWERPIN, HIGH);

  _delay_ms(2000);

  int8_t h  = dht.readHumidity();
  int16_t t = dht.readTemperature();

  if ( t == BAD_TEMP || h == BAD_HUM ) { // if error conditions
    serial.println("Failed to read from DHT");
  } else {
    sprintf(cmd, "H:%u%% T:%u*C", h, t);
    serial.println(cmd);
    digitalWrite(DHTPOWERPIN, LOW);
    _delay_ms(10000);
  }

}

