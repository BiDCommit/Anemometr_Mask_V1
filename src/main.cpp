#include <SPI.h>
#include <Arduino.h>
#include <Adafruit_BMP085.h>
#include <BLEDevice.h>
#include <BLEUtils.h>
#include <BLEServer.h>

const int photoInterrupterPin = 27; // Пин для первого модуля фотореле
volatile unsigned long pulseCount = 0; // Счётчик импульсов первого модуля
unsigned long lastMillis = 0; // Время последнего обновления первого модуля
const float diameter = 0.038; // Диаметр лопастей анимометра в метрах
const int pulsesPerRevolution = 30; // Количество импульсов за одно полное вращение

const int hx710DataPin = 4; // Пин для данных HX710
const int hx710ClockPin = 18; // Пин для часов HX710



// Промежуточные переменные для хранения давления
long lastPressure = 0;
long currentPressure = 0;

// Структура для хранения данных
struct SensorData {
  float airVolume;
  float windSpeed;
  uint8_t breathState; // 0 - выдох, 1 - вдох
  long pressureOnBreath; // Давление при дыхании
};

// BLE Service and Characteristics
BLECharacteristic *pSensorDataCharacteristic;

#define SERVICE_UUID "12345678-1234-5678-1234-56789abcdef0"
#define SENSOR_DATA_CHAR_UUID "abcdef01-1234-5678-1234-56789abcdef0"

void IRAM_ATTR handleInterrupt() {
  pulseCount++;
}

long readHX710() {
  long value = 0;
  while (digitalRead(hx710DataPin)); // Ждём пока OUT не станет LOW

  for (int i = 0; i < 24; i++) {
    digitalWrite(hx710ClockPin, HIGH);
    value = value << 1;
    digitalWrite(hx710ClockPin, LOW);
    if (digitalRead(hx710DataPin)) {
      value++;
    }
  }

  // Дополнительный такт для завершения преобразования
  digitalWrite(hx710ClockPin, HIGH);
  value = value ^ 0x800000;
  digitalWrite(hx710ClockPin, LOW);

  return value;
}

void setup() {
  Serial.begin(9600);


  pinMode(photoInterrupterPin, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(photoInterrupterPin), handleInterrupt, FALLING);

  pinMode(hx710DataPin, INPUT);
  pinMode(hx710ClockPin, OUTPUT);

  Serial.println("OXYReader Test");

  // Initialize BLE
  BLEDevice::init("OXYReader 1.0"); // Устанавливаем имя BLE-устройства
  BLEServer *pServer = BLEDevice::createServer();
  BLEService *pService = pServer->createService(SERVICE_UUID);

  pSensorDataCharacteristic = pService->createCharacteristic(
                                SENSOR_DATA_CHAR_UUID,
                                BLECharacteristic::PROPERTY_READ |
                                BLECharacteristic::PROPERTY_NOTIFY
                              );

  pService->start();

  BLEAdvertising *pAdvertising = BLEDevice::getAdvertising();
  pAdvertising->addServiceUUID(SERVICE_UUID);
  pAdvertising->setScanResponse(false);
  pAdvertising->setMinPreferred(0x06);  // функции, помогающие с проблемами подключения iPhone
  pAdvertising->setMinPreferred(0x12);
  BLEDevice::startAdvertising();
  Serial.println("Waiting for a client connection to notify...");
}

void loop() {
  unsigned long currentMillis = millis();

  if (currentMillis - lastMillis >= 500) { // Обновляем каждые 500 миллисекунд
    detachInterrupt(digitalPinToInterrupt(photoInterrupterPin));

    // Количество оборотов за одну секунду
    float revolutions = pulseCount / (float)pulsesPerRevolution;

    // Обнуляем счётчик импульсов для следующего измерения
    pulseCount = 0;

    // Возвращаем прерывание
    attachInterrupt(digitalPinToInterrupt(photoInterrupterPin), handleInterrupt, FALLING);

    // Расчёт скорости ветра
    float circumference = diameter * PI;
    float windSpeed = revolutions * circumference;
    float radius = diameter / 2.0;
    float area = PI * radius * radius;
    float airVolume = windSpeed * area;

    // Чтение текущего давления с HX710
    currentPressure = readHX710();

    // Определение вдоха и выдоха
    uint8_t breathState = 0;
    if (currentPressure > lastPressure) {
      breathState = 1; // вдох
    } else if (currentPressure < lastPressure) {
      breathState = 0; // выдох
    }

    // Обновление последнего давления
    lastPressure = currentPressure;

    // Сохранение данных в структуру
    SensorData data;
    data.airVolume = airVolume;
    data.windSpeed = windSpeed;
    data.breathState = breathState;
    data.pressureOnBreath = currentPressure; // Давление при дыхании

    // Сериализация структуры в массив байтов
    uint8_t dataArray[sizeof(SensorData)];
    memcpy(dataArray, &data, sizeof(SensorData));

    // Отправка данных по BLE
    pSensorDataCharacteristic->setValue(dataArray, sizeof(dataArray));
    pSensorDataCharacteristic->notify();

    lastMillis = currentMillis;
  }

  // Выводим температуру

}
