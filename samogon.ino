
#include <EEPROM.h>
#include <SPI.h>
#include <Wire.h>
#include <OneWire.h>
#include <DallasTemperature.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include <Bounce2.h>
#include <Rotary.h>
#include <PID_v1.h>


//PID
double PIDSetpoint, PIDInput, PIDOutput;
double Kp = 4, Ki = 1, Kd = 1;


volatile boolean EmergencyExitCode = 0; //общая критическая ошибка - проверяется везде!

//дисплей
Adafruit_SSD1306 display(128, 64, &Wire, 4);

//уровень мощности
static uint8_t Power = 0; // Задаваемый уровень мощности в %
int DesiredPower = 0; //желаемая мощность по умолчанию
const int PowerEepromAddress = 1; //адрес куда сохраняется мощность в EEPROM
static int PDMError = 0; // Ошибка дискретизации pdm-модулятора
static int pp = 1; // Условынй индикатор знака полупериода (1 - положительный, -1 - отрицательный)
static int ps = 0; // ps пропорциональна постоянной составляющей
static uint8_t HeaterPin = 10;   // Пин ТЭНА (для управления симистором)
static uint8_t pFlag = 0; // Наличие сигнала (импульса) на выходе модулятора
static unsigned long pStart; // Момент начала импульса на выходе модулятора


//поворотная кнопка
const int EncoderButton = 6;  // кнопка энкодера
const int EncoderBack = 3; //против часовой стрелки поворотная кнопка
const int EncoderForward = 5; //по часовой стрелке поворотная кнопка
const int EncoderButtonDebouncePin = 4; //проверочный пин для нажатия кнопки

//температура
double MemoryTempSensorValue = 0;
int TempSensorErrorCount = 0; //буфер в который складываются ошибки датчика 
const int OneWireTempSensorAddress = 11; //датчик температуры воды
const int SensorErrorMax = 10; //максимальное количество игнорируемых ошибок датчиков температуры


PID myPID(&PIDInput, &PIDOutput, &PIDSetpoint, Kp, Ki, Kd, DIRECT);

//инициируем объект дебаунсера
Bounce DebouncedEncoder = Bounce();

void PrintOLED(int Size, String Temperature, int CursorX=20, int CursorY=20) {
  display.setTextSize(Size);
  display.setTextColor(WHITE);
  display.setCursor(CursorX, CursorY);
  display.println(String(Temperature));
  display.display();
}

void ClearOLED() {
  display.clearDisplay();
}



void setup() {

  Serial.begin(9600);
  attachInterrupt(0, Brezenhem, FALLING); // Прерывания по спаду импульсов входного сигнала
  pinMode(HeaterPin, OUTPUT); //Инициализируем пин ТЭНА на выход
  digitalWrite(HeaterPin, LOW); //ТЭН выключен по умолчанию
  //Берем данные из ПЗУ если они там есть
  PIDInput=MemoryTempSensorValue;
  PIDSetpoint = 64;
  Power = EEPROM.read(PowerEepromAddress);

  //turn the PID on
  myPID.SetMode(AUTOMATIC);

  // SSD1306_SWITCHCAPVCC = generate display voltage from 3.3V internally
  if (!display.begin(SSD1306_SWITCHCAPVCC, 0x3C)) { // Address 0x3C for 128x64
    if (!display.begin(SSD1306_SWITCHCAPVCC, 0x3D)) { // Address 0x3D for 128x64
      Serial.println("Address 0x3C and 0x3d for OLED failed");
    }
  }

  PrintOLED(2, String(Power) + "%");
  //input interfaces (sensors):
  pinMode(EncoderButton, INPUT_PULLUP);
  //цепляемся раздребезгом к кнопке
  DebouncedEncoder.attach(EncoderButton);
  DebouncedEncoder.interval(5);
}

void loop() {
  if (digitalRead(EncoderButton) == LOW) {
    InteractiveInput();
  } else {
	EvaluatePID();
  }
}

void EvaluatePID() {
    PIDInput=GetTemperature(OneWireTempSensorAddress);
    PrintOLED(2 , String(PIDInput) + "t");

    //пропорционально уменьшаем параметры пид регулятора при приближении к желаемой температуре
    double gap = abs(PIDSetpoint - PIDInput); //расстояние дл желаемой температуры
    if (gap < int(PIDSetpoint/2))
    { 
      myPID.SetTunings(Kp/2, Ki/2, Kd/2);
    }
    else if (gap < int(PIDSetpoint/4))
    { 
      myPID.SetTunings(Kp/4, Ki/4, Kd/4);
    }
    else if (gap < int(PIDSetpoint/8))
    {
      myPID.SetTunings(Kp/8, Ki/8, Kd/8);
    }
    else 
    {
      //we're far from setpoint, use aggressive tuning parameters
      myPID.SetTunings(Kp, Ki, Kd);
    }

    //считаем ПИД функцию
    myPID.Compute();

    //показываем полученный от ПИД функции параметр мощности
    Power=int(PIDOutput/2.55);
    PrintOLED(2, String(Power) + "%");
}

void Brezenhem() { // Функция - обработчик внешнего прерывания INT0
  int pulse = 0, lev = Power + PDMError; // Текущий уровень с учетом ошибки дискретизации, сделанной на предыдущем полупериоде.
  if (lev >= 50) pulse = 1; // Нужно бы подать импульс, но проверим на постоянную составляющую
  // Если pulse == 1, но подача импульса приведет к увеличению постоянной составляющей, то отложим
  // подачу импульса на следующий полупериод. Он будет другого знака, что приведет к уменьшению
  // постоянной составляющей
  if (ps * pulse * pp > 0) pulse = 0;
  if (pulse) {
    digitalWrite(HeaterPin, HIGH); //подаем +5 вольт на выход к тэну;
    PDMError = lev - 100;  // Считаем ошибку для следующего полупериода
  } else {
    digitalWrite(HeaterPin, LOW); //подаем 0 вольт (заземляем) на выход к тэну;
    PDMError = lev;
  }
  pStart = millis(); pFlag = 1; // Фиксируем момент начала управляющего импульса (или его отсутствия)
  ps += pp * pulse; // Считаем текущую постоянную составляющую
  pp = -pp; // Следующий полупериод будет другого знака
}


void InteractiveInput() {
  //инициализируем поворотную кнопку (энкодер)
  Rotary r = Rotary(EncoderBack, EncoderForward);
  Serial.println(DesiredPower);
  ClearOLED();
  delay(200);//wait encoder button to calm down)
  PrintOLED(4, String(Power) + "%");
  //вводим желаемое значение температуры воды
  DebouncedEncoder.update();
  while (digitalRead(EncoderButton) != LOW) {
    unsigned char RotaryDirection = r.process();
    if (RotaryDirection) {
      Serial.println(RotaryDirection);
      if ((RotaryDirection == DIR_CCW) and (Power > 0)) {
        Serial.println(Power);
        Power--;
      } else if (Power < 100) {
        Serial.println(Power);
        Power++;
      }
      ClearOLED();
      PrintOLED(4, String(Power) + "%");
    }
  }
  EEPROM.update(PowerEepromAddress, Power);

  ClearOLED();
  PrintOLED(4, "OK");
  delay(200);
  ClearOLED();
  PrintOLED(4, String(Power) + "%");
}



double GetTemperature(int SensorPort) {
  double Temperature;
  OneWire oneWire(SensorPort);
  DallasTemperature sensors(&oneWire);
  DeviceAddress TempSensBus;
  sensors.begin();
  sensors.requestTemperatures();
  if (!sensors.getAddress(TempSensBus, 0)) {
    if (SensorPort == OneWireTempSensorAddress) {
      TempSensorErrorCount++;
      Temperature = MemoryTempSensorValue;
    }

    if (TempSensorErrorCount > SensorErrorMax) {
      if (SensorPort == OneWireTempSensorAddress) {
        EmergencyExitCode = 1;
        goto finish;
      }
    }
    if (Temperature > 125) {
      Temperature = 125;
    }
    return Temperature;
  } else {
    Temperature = sensors.getTempCByIndex(0);
    if (SensorPort == OneWireTempSensorAddress) {
      MemoryTempSensorValue = Temperature;
      TempSensorErrorCount = 0;
    }
    return Temperature;
  }
finish:;
}
