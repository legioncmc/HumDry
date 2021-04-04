#include "Wire.h" //Шина I2C
#include <Adafruit_Sensor.h> // Датчик температуры
#include <Adafruit_BME280.h>  // Датчик температуры
//#include <AM2320.h> //Датчик температуры старый
#include <DallasTemperature.h> //Датчик температуры
#include <LiquidCrystal_I2C.h> // LCD Экран
#include "GyverEncoder.h" //Энкодер
#include "TimerOne.h" //Таймер
#include "GyverPID.h" //ПИД регулятор
#include <EEPROM.h> //Работа с памятью

#define DS18B20 10  // Датчик температуры
#define ngvPin 3    // Нагреватель
#define ventPin 8   // Вентилятор
#define ventArPin 6 // Вентилятор на плату

#define CLK 4       // Энкодер  пин даннынх
#define DT 5        // Энкодер пин данных
#define SW 2        // Энкодер пин кнопки

#define INIT_ADDR 1023  // номер резервной ячейки
#define INIT_KEY 50     // ключ первого запуска. 0-254, на выбор


//AM2320 th;                              // Датчик температуры
OneWire oneWire(DS18B20);                 // Датчик температуры - шина
DallasTemperature sensors(&oneWire);      // Теперь это сенсор
Adafruit_BME280 bme;                      // BME

GyverPID regulatorTemp(0.1, 0.05, 0.01, 10);  // ПИД регулятор температуры

LiquidCrystal_I2C lcd(0x27,16,2);         // LCD

Encoder enc1(CLK, DT, SW);                // Энкодер

unsigned long windowStartTime;            //служебная для таймера экрана

int CursorPos = 0;        // Позиция курсора в меню
int OldCursorPos = 0;     // Старая позиция курсора, чтобы стереть
bool setupMode = false;   // Режим Настройки
bool AllOn = true;        // Включить вентиляторы

struct TTempStruct        // Тип для температуры для EEPROMа
{
  float Temp1;
  float Temp2;
  float Hum;
};

TTempStruct TempStruct;   // Переменная для температур

void setup() {
  Serial.begin(9600); //Монитор порта
  Serial.print("Input");Serial.print(' ');Serial.print("Output");Serial.print(' ');Serial.println("SetPoint"); // Для графиков вводим заголовки
  pinMode(ventPin, OUTPUT);   // Пин вентилятора
  pinMode(ventArPin, OUTPUT); // Пин вентилятора Платы

  //digitalWrite(ngvPin, HIGH);
  digitalWrite(ventPin, HIGH);  // Включаем Вентилятор
  digitalWrite(ventArPin, HIGH);  // Включаем Вентилятор Платы
  bme.begin(0x76, &Wire); // Запускаем датчик температуры
  AllOn = true; // Считаем, что все включено

  if (EEPROM.read(INIT_ADDR) != INIT_KEY) { // первый запуск
    EEPROM.write(INIT_ADDR, INIT_KEY);    // записали ключ
    // записали стандартное значение яркости
    // в данном случае это значение переменной, объявленное выше
    TempStruct.Temp1 = 50;
    TempStruct.Temp2 = 100;
    TempStruct.Hum = 30;
    EEPROM.put(0, TempStruct);
  }
  EEPROM.get(0,TempStruct);
  
  //Gyver pid
  regulatorTemp.setDirection(NORMAL); // направление регулирования (NORMAL/REVERSE). ПО УМОЛЧАНИЮ СТОИТ NORMAL
  regulatorTemp.setLimits(0, 255);    // пределы (ставим для 8 битного ШИМ). ПО УМОЛЧАНИЮ СТОЯТ 0 И 255
  regulatorTemp.setpoint = TempStruct.Temp1;        // сообщаем регулятору температуру, которую он должен поддерживать
  // в процессе работы можно менять коэффициенты. Данные коэфициенты взял через настройки ПИДа
  regulatorTemp.Kp = 38.03;
  regulatorTemp.Ki = 1.58;
  regulatorTemp.Kd = 228.64;

  lcd.init();                     // Инициация экрана                     
  lcd.backlight();                // Включаем подсветку дисплея
  lcd.print("Humid camera 1.0");  // Стартовая надпись
  lcd.setCursor(8, 1);            
  lcd.print("LCD 1602");
  lcd.clear();

  enc1.setType(TYPE2);            // Тип экрана - второй

  Timer1.initialize(100);         // установка таймера на каждые 1000 микросекунд (= 1 мс) для энкодера. В прямую хроен работает
  Timer1.attachInterrupt(isr);    // Функция прерывания по Таймеру кнопок
}

TTempStruct Temper;   // Живые показания температуры

void loop() {

  //читаем температуру с датчика с влажностью - пока убил
  /*switch(th.Read()) {
    case 2:
      Serial.println("CRC failed");
      break;
    case 1:
      Serial.println("Sensor offline");
      break;
    case 0:
      Temper.Hum = th.h;
      Temper.Temp1 = th.t;
      break;
  }*/

  // Чтение датчика
  Temper.Temp2 = bme.readTemperature(); // Теперь температуру берем с него для ПИДа
  Temper.Hum = bme.readHumidity();
  
  //читаем вторую температуру
  sensors.requestTemperatures(); // А температура внутри просто информационно
  float ttt = sensors.getTempCByIndex(0); // Чобы не было ошибки -127
  if (ttt>0) 
  {
    Temper.Temp1=ttt;

    // Проверка шкафа! Если превшает температуру - все включить!
    if (ttt>TempStruct.Temp2)
    {
      analogWrite(ngvPin, 0);       
      digitalWrite(ventPin, LOW);   
      digitalWrite(ventArPin, LOW);   
    }
  }
  
  // Gyver PID
  // Проверяем включено или нет
  if (AllOn){
    regulatorTemp.input = Temper.Temp2;                   // Вносим в ПИД текущую температуру
    analogWrite(ngvPin, regulatorTemp.getResultTimer());  // Коррекитруем температуру - на транзистор подает частоту (0..255) в зависимости от результата ПИДа
    digitalWrite(ventPin, HIGH);                          // Включаем Вент, если вдруг выключен
  } else {
    analogWrite(ngvPin, 0);                               // Не трогаем ПИД 
    digitalWrite(ventPin, LOW);                           // Выключаем вент
  }
  
  //Показываем дисплей раз в 100мс - Чаще моргания
  if (millis()-windowStartTime>100){

    // Пишим в монитор дебаг
    Serial.print(Temper.Temp2);Serial.print(' ');
    Serial.print(Temper.Temp1);Serial.print(' ');
    Serial.print(Temper.Hum);Serial.print(' ');
    Serial.print(regulatorTemp.output);Serial.print(' ');
    Serial.println(TempStruct.Temp1);

    DisplayLCD();               // Отображаем мониторчик

    windowStartTime = millis(); // Сбрасываем таймер
    
  }
}

void DisplayLCD()
{
  String s1=" T1   T2   H    "; // Верхняя строка
  lcd.setCursor(0,0);
  lcd.print(s1);

  if (setupMode){       // Setup Mode
    // ЗВездочки показыват, что мы сетапе
    lcd.setCursor(3,0); 
    lcd.print("*");
    lcd.setCursor(8,0);
    lcd.print("*");
    lcd.setCursor(12,0);
    lcd.print("*");
    lcd.setCursor(15,0);
    lcd.print("S");

    s1=String(TempStruct.Temp1,1)+" "+String(TempStruct.Temp2,1)+" "+String(TempStruct.Hum,1)+" "; // вторая строка из настроек температур
    lcd.setCursor(0, 1);
    lcd.print(s1);

  }else{              // Обычный режим
    // Стираем к черту звездочки, если они есть.
    lcd.setCursor(3,0);
    lcd.print(" ");
    lcd.setCursor(8,0);
    lcd.print(" ");
    lcd.setCursor(12,0);
    lcd.print(" ");
    lcd.setCursor(15,0);
    lcd.print("V");

    s1=String(Temper.Temp2,1)+" "+String(Temper.Temp1,1)+" "+String(Temper.Hum,1)+" "; // вторая строка из реальной температуры
    lcd.setCursor(0, 1);
    lcd.print(s1);
    
  }

  // Показываем мы в сетапе или в обычном режиме
  lcd.setCursor(15, 1);
  if (AllOn)
    lcd.print("*");
  else 
    lcd.print(" ");

  // стираем старый курсор
  switch (OldCursorPos) {
    case 0:
      lcd.setCursor(0,0);
      break;
    case 1:
      lcd.setCursor(5,0);
      break;
    case 2:
      lcd.setCursor(10,0);
      break;
    case 3:
      lcd.setCursor(14,0);
      break;
  }
  lcd.print(" ");

  // Рисуем новый курсор
  switch (CursorPos) {
    case 0:
      lcd.setCursor(0,0);
      break;
    case 1:
      lcd.setCursor(5,0);
      break;
    case 2:
      lcd.setCursor(10,0);
      break;
    case 3:
      lcd.setCursor(14,0);
      break;
  }
  lcd.print(">");  
  
}

// Обработка кнопок!
void isr() {
  enc1.tick();  // отработка в прерывании

  float Pos = 0;

  // Влево-вправо обычный шаг
  if (enc1.isRight()) Pos = 1;
  if (enc1.isLeft()) Pos = -1;

  // Нажато - тонкая настройка
  if (enc1.isRightH()) Pos = 0.1;
  if (enc1.isLeftH()) Pos = -0.1;

  // быстро крутить по 5
  if (enc1.isFastR()) Pos = 5;
  if (enc1.isFastL()) Pos = -5;

  // Нажали - перешли в сетап или вышли
  if (enc1.isClick()) setupMode = !setupMode;
  

  // Сняли показания, а теперь меняем состояния
  if (setupMode){               // В сетапе
    switch (CursorPos){
      case 0:
        TempStruct.Temp1 += Pos;
        break;
      case 1:
        TempStruct.Temp2 += Pos;
        break;
      case 2:
        TempStruct.Hum += Pos;
        break;
      case 3:
        if (Pos != 0) AllOn = !AllOn; // пофигу быстро или медленно, главное, чтобы не ноль.
        break;
    }

    // Меняем запиись в EEPROMе
    EEPROM.put(0,TempStruct);
    regulatorTemp.setpoint = TempStruct.Temp1; 
    
  } else {                    // В обычном режиме гоняем курсор (4 позиции - зацикливаемм
    // Два сложных ифа - чтобы при равенстве нулю не затереть курсор. Скорость вращения энкодера нам не интересна
    if (Pos>0)
    {
      OldCursorPos = CursorPos;
      CursorPos += 1;
      if (CursorPos>3) CursorPos = 0;
    }
    if (Pos<0)
    {
      OldCursorPos = CursorPos;
      CursorPos -= 1;
      if (CursorPos<0) CursorPos = 3;
    }
  }
}
