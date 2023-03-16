// #define RESIST
//#define LIGHT 
#define THERMO 

//==лабораторная работа "датчики" (версия #2)==2023==
/*
 * УНИВЕРСАЛЬНЫЙ КОД ДЛЯ ТРËХ ВИДОВ УСТАНОВОК, ВЫБОР ОПРЕДЕЛЯЕТСЯ УСЛОВНОЙ КОМПИЛЯЦИЕЙ:
 * THERMO - исследование датчиков температуры
 * LIGHT - исследование датчиков освещённости
 * RESIST - исследование датчиков угла поворота
 *
 * температура:
 * в качестве наиболее точного (эталлонного) датчика температуры выступает цифровой сенсор DS18B20
 * по показаниям этого датчика следует ориентироваться при получении текущих значений температуры
 * в качестве исследуемых датчиков выступают:
 * - платиновый терморезистор PT100
 * - термистор NTC 3950 100 кОм №1 (расположен вместе с DS18B20, но не на всех головах)
 * - термиcтор NTC 3950 100 кОм №2 (расположен непосредственно у нагревателя, но не на всех головах)
 * 
 * освещённость:
 * WSB-светодиод в качестве источника в чёрном контейнере совместо с фоторезистивными датчиками
 * светодиод к порту А_, потенциометр - к А_
 * снимается зависимость в Red-Green-Blue-White-режимах, от 0 до 100%
 * снимается разница между двумя фоторезисторами в White-режиме и строится водной системе координат
*/
#ifdef RESIST

#include <LiquidCrystal.h>
LiquidCrystal lcd(8,13, 9,      4,   5,  6, 7);

int portR1=A1;//резистор #1
int portR2=A2;//реистор #2

void setup() 
{
Serial.begin(9600);//инициируем монитор последовательного интерфейса со скоростью обмена данными 9600 бит/с
lcd.begin(16,2);
}

void loop() 
{

float U1=(1024-analogRead(A1))/1024.0*5.0;
float U2=(1024-analogRead(A2))/1024.0*5.0;
Serial.println(U1);


lcd.setCursor(0, 0);
lcd.print("U1=");
lcd.setCursor(3, 0);
lcd.print("      ");
lcd.setCursor(3, 0);
lcd.print(U1,4);
lcd.print("V");

lcd.setCursor(0, 1);
lcd.print("U2=");
lcd.setCursor(3, 1);
lcd.print("      ");
lcd.setCursor(3, 1);
lcd.print(U2,4);
lcd.print("V");
delay(100);



}

#endif





#ifdef THERMO

#include <math.h>
#include <LiquidCrystal.h>
#include <OneWire.h>//библиотека для работы с однопроводным интерфейсом#include <LiquidCrystal.h>

LiquidCrystal lcd(8, 13, 9, 4, 5, 6, 7);


//=====================================================//

#define B 3950 // B-коэффициент
#define SERIAL_R 100000 // сопротивление последовательного резистора, 100 кОм
#define THERMISTOR_R 100000 // номинальное сопротивления термистора, 100 кОм

#define SERIAL_R2 100 // сопротивление последовательного резистора, 100 Ом
#define THERMISTOR_R2 100 // номинальное сопротивления термистора, 100 Ом
#define NOMINAL_T2 0 // номинальная температура (при которой TR = 100 Ом)

#define NOMINAL_T 25 // номинальная температура (при которой TR = 100 кОм)

int portR1=A1;//термистор #1
int portR2=A2;//термистор #2
int portR3=A3;//pt100
int Rpin=A4;//регулятор мощности нагревателя A5
int portDS=3;//цифровой датчик


float celsius=0;

OneWire  ds(portDS);  

void setup() 
{
Serial.begin(9600);//инициируем монитор последовательного интерфейса со скоростью обмена данными 9600 бит/с
pinMode(11,OUTPUT);
lcd.begin(16,2);
}




int hpower=0;


float gettemp() 
{
  byte i;
  byte present = 0;
  byte type_s;
  byte data[12];
  byte addr[8];

if ( !ds.search(addr)) 
{
    ds.reset_search();
    delay(50);
    return;
}  
  
  if (OneWire::crc8(addr, 7) != addr[7]) 
  {
      return;
  }
 
  switch (addr[0]) {
    case 0x10:
      type_s = 1;
      break;
    case 0x28:
      type_s = 0;
      break;
    case 0x22:
   
      type_s = 0;
      break;
    default:
     // Serial.println("Device is not a DS18x20 family device.");
      return;
  } 

  ds.reset();
  ds.select(addr);
  ds.write(0x44, 1);        // start conversion, with parasite power on at the end
  
  delay(200);     // maybe 750ms is enough, maybe not
  
  
  present = ds.reset();
  ds.select(addr);    
  ds.write(0xBE);         // Read Scratchpad

  for ( i = 0; i < 9; i++) 
  {        
    data[i] = ds.read();
  }

  int16_t raw = (data[1] << 8) | data[0];
  if (type_s) {
    raw = raw << 3; // 9 bit resolution default
    if (data[7] == 0x10) {
      // "count remain" gives full 12 bit resolution
      raw = (raw & 0xFFF0) + 12 - data[6];
    }
  } else {
    byte cfg = (data[4] & 0x00);
    // at lower res, the low bits are undefined, so let's zero them
    if (cfg == 0x00) raw = raw & ~7;  // 9 bit resolution, 93.75 ms
    else if (cfg == 0x20) raw = raw & ~3; // 10 bit res, 187.5 ms
    else if (cfg == 0x40) raw = raw & ~1; // 11 bit res, 375 ms
    //// default is 12 bit resolution, 750 ms conversion time
  }
  celsius = (float)raw / 16.0;
//Serial.print(celsius);
}




float getNTC100(int A)//в килоомах при 100к резисторе
{
  int t = analogRead( A );
  float tr = 1023.0 / t - 1;
  tr = 100 / tr;
  return tr;
} 

float getPT100(int A)//в омах при 100Ом резисторе
{
  int t = analogRead( A );
  float tr = 1023.0 / t - 1;
  tr = SERIAL_R2 / tr;
  return tr;
} 

int lcd_hpower=0;

float lcd_R1=0.0;
float lcd_T=0.0;
float lcd_R2=0.0;
float lcd_R3=0.0;



float R1=0.0;
float T=0.0;
float R2=0.0;
float R3=0.0;
//клавиатура-------
int key=0;
//int keys[]={0,0,0,0,0};//select - 1, left - 2, up - 3, down - 4, right - 5
//select - 640
//left-412
//up-99
//down-259
//right-0
//-----------------








int a = 3;
char b = '-';
float c = 3.14;
char output_string[16];

int buttonPushCounter = 1;   // counter for the number of button presses
int buttonState = 0;         // current state of the button
int lastButtonState = 0;     // previous state of the button

int real_temp=0;
int temps[] = {30, 35, 40, 45, 50, 55, 60, 65, 70, 75, 80};


void loop() 
{


 
//--считываю все значения--
hpower=analogRead(Rpin);
gettemp();//получаю температуру
T=celsius;
R1=getNTC100(portR1);
R2=getNTC100(portR2);
R3=getPT100(portR3);





int a0=analogRead(A0);
if (a0>=1020)             { key=0; }//nothing
if ((a0>=600)&&(a0<=680)) { key=1; }//select
if ((a0>=400)&&(a0<=440)) { key=2; }//left
if ((a0>=80)&&(a0<=110))  { key=3; }//up
if ((a0>=250)&&(a0<=270)) { key=4; }//down
if ((a0>=-5)&&(a0<=5))    { key=5; }//right
/*
switch (key) { 
case 1:  Serial.print("select "); break; 
case 2:  Serial.print("left ");   break; 
case 3:  Serial.print("up ");     break;  
case 4:  Serial.print("down ");   break; 
case 5:  Serial.print("right ");  break;  
default: Serial.print("none ");
}
*/

  buttonState = key;
  if (buttonState != lastButtonState) {
    if (buttonState == 4) {
      buttonPushCounter++; if (buttonPushCounter>3) {buttonPushCounter=3;}}
      if (buttonState == 3) {
      buttonPushCounter--;if (buttonPushCounter<1) {buttonPushCounter=1;}}


      
   //   Serial.print("number of button pushes: ");
     // Serial.println(buttonPushCounter);
     
    delay(50);
  }
  lastButtonState = buttonState;
  

/*

Serial.print(analogRead(A0));
Serial.print("  ");
Serial.print(analogRead(A1));
Serial.print("  ");
Serial.print(analogRead(A2));
Serial.print("  ");
Serial.print(analogRead(A3));
Serial.print("  ");
Serial.print(analogRead(A4));
Serial.print("  ");
Serial.println(analogRead(A5));
*/



if ((analogRead(A0) < 150)&(analogRead(A0)> 90)) 
{
  
} 
  else 
{
  
}

if (key==1) {hpower=0; analogWrite(11, 0); goto m;}//пока держишь нажатым "селект" - выключает нагреватель и перекидывает на отображение данных без обновления (стоп-кадр)

 lcd_R1=R1;
 lcd_R2=R2;
 lcd_R3=R3;
 lcd_T=T;


for (int i = 0; i < 11; i++) {

if (T == temps[i]) {

   if (real_temp!=temps[i]) {
    Serial.print(T);
Serial.print(" C ");
Serial.print(R1);
Serial.print(" ");
Serial.print(R2);
Serial.print(" ");
Serial.println(R3);
real_temp=temps[i];
    }
   }

  }








hpower =map(hpower,20,1000,0,60);//ограничиваем диапазон входящих значений от 0 до 60 
hpower=constrain(hpower, 0, 60);//контроль выхода за ограничения

//--нагреватель---------------
if (lcd_T>60) 
{
  hpower =map(hpower,0,60,0,50);//ограничиваем диапазон входящих значений от 0 до 60 
  hpower=constrain(hpower, 0, 50);//контроль выхода за ограничения
}

if (lcd_T>70) 
{
  hpower =map(hpower,0,60,0,40);//ограничиваем диапазон входящих значений от 0 до 60 
  hpower=constrain(hpower, 0, 40);//контроль выхода за ограничения
}

if (lcd_T<=80) 
{ 
  analogWrite(11, hpower);//ШИМ на нагреватель (максимум - 255, но сделано меньше для более медленного нагрева)
} 
else
{
  analogWrite(11, 0);
  hpower=0;//контроль выхода за ограничения
}

lcd_hpower=hpower; 

m: 

lcd.setCursor(0, 0);
lcd.print("Heat:");
lcd.setCursor(5, 0);
lcd.print("    ");
lcd.setCursor(5, 0);

lcd.print(map(lcd_hpower,0,60,0,100));//форматирование вывода информации о мощности нагревателя в процентах
lcd.print("%");
lcd.setCursor(10, 0);
//--цифровой датчик---    
lcd.print("T=");

lcd.print(lcd_T);
//--------------------

lcd.setCursor(0, 1);
lcd.print("                ");
lcd.setCursor(0, 1);

//вторая строка динамична,в зависимости от buttonPushCounter

lcd.setCursor(0, 1);

switch (buttonPushCounter) { 
case 1: lcd.print("R1: "); lcd.print(lcd_R1); lcd.print("k "); lcd.setCursor(11, 1); lcd.print("(NTC)");break; 
case 2: lcd.print("R2: "); lcd.print(lcd_R2); lcd.print("k");  lcd.setCursor(11, 1); lcd.print("(NTC)");  break; 
case 3: lcd.print("R3: "); lcd.print(lcd_R3); lcd.setCursor(11, 1); lcd.print("PT100");    break;  
}

delay(50);

}

#endif

#ifdef LIGHT

#include <math.h>
#include <LiquidCrystal.h>
#include <OneWire.h>//библиотека для работы с однопроводным интерфейсом#include <LiquidCrystal.h>
#include <Adafruit_NeoPixel.h>

LiquidCrystal lcd(8, 13, 9, 4, 5, 6, 7);
Adafruit_NeoPixel pixels(1, 15, NEO_GRB + NEO_KHZ800);

void setup() 
{
  pixels.begin(); 
  Serial.begin(9600);//инициируем монитор последовательного интерфейса со скоростью обмена данными 9600 бит/с
  lcd.begin(16,2);
}

int key=0;

int buttonState = 0;         // current state of the button
int lastButtonState = 0;     // previous state of the button

//up-dowm
int maxPushCounter=10;
int minPushCounter=0;
int buttonPushCounter = minPushCounter;   // counter for the number of button presses

//left-right
int maxleftRightPushCounter=5;
int minleftRightPushCounter=1;
int leftRightPushCounter = minleftRightPushCounter;  

int color[3];//вектор rgb-цвета

void getButtons()
{
  int a0=analogRead(A0);
  if (a0>=1020)             { key=0; }//nothing
  if ((a0>=600)&&(a0<=680)) { key=1; }//select
  if ((a0>=400)&&(a0<=440)) { key=2; }//left
  if ((a0>=80)&&(a0<=110))  { key=3; }//up
  if ((a0>=250)&&(a0<=270)) { key=4; }//down
  if ((a0>=-5)&&(a0<=5))    { key=5; }//right

  buttonState = key;
  
  if (buttonState != lastButtonState) {
    //up-dowm
    if (buttonState == 3) { buttonPushCounter++; if (buttonPushCounter>maxPushCounter) {buttonPushCounter=maxPushCounter;} }
    if (buttonState == 4) { buttonPushCounter--; if (buttonPushCounter<minPushCounter) {buttonPushCounter=minPushCounter;} }        
    if (buttonState == 5) { leftRightPushCounter++; if (leftRightPushCounter>maxleftRightPushCounter) {leftRightPushCounter=maxleftRightPushCounter;} }
    if (buttonState == 2) { leftRightPushCounter--; if (leftRightPushCounter<minleftRightPushCounter) {leftRightPushCounter=minleftRightPushCounter;} }        
    delay(50);
  }
  
  lastButtonState = buttonState;
}

void setColor()
{ //беру ргб-массив и каждый элемент умножаю на шим-коэффициент
  int percent = map(buttonPushCounter, minPushCounter,maxPushCounter ,0,255);
  pixels.setPixelColor(0, pixels.Color(percent*color[0], percent*color[1], percent*color[2]));
  pixels.show(); 
}
  
void loop() 
{
  getButtons();//опрос клавиатруры

  lcd.setCursor(0, 0);
  lcd.print("                ");
  lcd.setCursor(0, 0);

  switch (leftRightPushCounter) { 
    case 1: lcd.print("OFF");   color[0]=0; color[1]=0; color[2]=0; break; 
    case 2: lcd.print("RED");   color[0]=1; color[1]=0; color[2]=0; break; 
    case 3: lcd.print("GREEN"); color[0]=0; color[1]=1; color[2]=0; break; 
    case 4: lcd.print("BLUE");  color[0]=0; color[1]=0; color[2]=1; break;  
    case 5: lcd.print("WHITE"); color[0]=1; color[1]=1; color[2]=1; break;  
  }

  lcd.setCursor(0, 1);
  lcd.print("                ");
  lcd.setCursor(0, 1);
  lcd.print(map(buttonPushCounter, minPushCounter,maxPushCounter ,0,100));
  lcd.print("%");
  setColor();

  lcd.setCursor(9, 0);
  lcd.print("result:");
  lcd.setCursor(9, 1);

  if (key==1) { goto m; }

  int sens = analogRead(A5);

  m: lcd.print(sens);

  delay(50);

}

#endif
