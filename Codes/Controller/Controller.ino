#include  <SPI.h>
#include "nRF24L01.h"
#include "RF24.h"
#include <Smoothed.h>

Smoothed <float> degerexpo;
Smoothed <float> degerxrexpo;
Smoothed <float> degerxlexpo;
Smoothed <float> degeryrexpo;

RF24 radio(9, 10);

const uint64_t pipe = 0xF0F0F0F0E1LL;



float scaleX =  0.1;
float calX =    -527;
float offsetX = 0;

float scaleY = -0.1;
float calY = -507;
float offsetY = 0;

float scaleZ = -0.1;
float calZ = -512;
float offsetZ = 0;

float scaleThrust = 1.5;
float calThrust = -500;
float offsetThrust = 1300;

const int XL_pin = 1; // analog pin connected to XL output
const int YL_pin = 0; // analog pin connected to YL output

const int XR_pin = 3; // analog pin connected to XR output
const int YR_pin = 2; // analog pin connected to YR output

int ID = 0;
float xr, yr;
float xl, yl;
float smoothyl, smoothxr, smoothxl, smoothyr;
int but1, but2, but3;
void readJoyStick();
void printPackage();


struct Package
{
  int   thrust = 0;
  float   x = 0;
  float   y = 0;
  float   z = 0;
  int  id = 0;
  int but1 = 1;
  int but2 = 1;
  int but3 = 1;
};

Package package;

void setup() {
  pinMode(4, INPUT_PULLUP);
  pinMode(5, INPUT_PULLUP);
  pinMode(3, INPUT_PULLUP);
  
  //Serial.begin(57600); //You can enable to debug
  
  radio.begin();
  radio.setAutoAck(false);
  radio.setDataRate(RF24_250KBPS);
  radio.setPALevel(RF24_PA_LOW);
  radio.openWritingPipe(pipe);
  radio.stopListening();
  
  degerexpo.begin(SMOOTHED_EXPONENTIAL, 1);
  degerxrexpo.begin(SMOOTHED_EXPONENTIAL, 1);
  degerxlexpo.begin(SMOOTHED_EXPONENTIAL, 1);
  degeryrexpo.begin(SMOOTHED_EXPONENTIAL, 1);
}

void loop() {
  readJoyStick();

  package.x =       (xr + calX) * scaleX + offsetX;
  package.y =       (yr + calY) * scaleY + offsetY;
  package.z =       (xl + calZ) * scaleZ + offsetZ;
  package.thrust =  (yl + calThrust) * scaleThrust + offsetThrust;
  package.id = ID;
  ID++;
  package.but1 = but1;
  package.but2 = but2;
  package.but3 = but3;
  radio.write(&package, sizeof(package));
  //printPackage(); You can enable to debug
}
void readJoyStick()
{
  but1 = digitalRead(4);
  but2 = digitalRead(5);
  but3 = digitalRead(3);

  smoothxr = analogRead(XR_pin);
  smoothxl = analogRead(XL_pin);
  smoothyr = 1013 - analogRead(YR_pin);
  smoothyl = max(analogRead(YL_pin), 0);

  degerexpo.add(smoothyl);
  degerxrexpo.add(smoothxr);
  degerxlexpo.add(smoothxl);
  degeryrexpo.add(smoothyr);

  xr = degerxrexpo.get();
  xl = degerxlexpo.get();
  yr = degeryrexpo.get();
  yl = degerexpo.get();



}


void printPackage()
{
  Serial.print(package.thrust);
  Serial.print("\t");
  Serial.print(package.z);
  Serial.print("\t");
  Serial.print(package.x);
  Serial.print("\t");
  Serial.println(package.y);

}
