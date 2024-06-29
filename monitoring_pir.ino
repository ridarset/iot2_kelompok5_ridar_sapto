#define THINGER_SERIAL_DEBUG

//memasukan library yang akan dipakai
#include <ThingerESP32.h>
#include "arduino_secrets.h"
#include <Wire.h>
#include <AHT20.h>
#include <MQUnifiedsensor.h>
AHT20 aht20;

//inisialisasi pin dan sensor
float temperature = 0;
float humidity = 0;
int dataPIR = 0;
float data_gas = 0.0;
int PIRsens = 13;
int led1 = 14;
int led2 = 27;
int led3 = 26;
int led4 = 25;

/************************Hardware Related Macros************************************/
#define         Board                   ("ESP-32")
#define         Pin                     (32)  //Analog input 3 of your arduino
/***********************Software Related Macros************************************/
#define         Type                    ("MQ-2") //MQ2
#define         Voltage_Resolution      (3.3)
#define         ADC_Bit_Resolution      (12) // For arduino UNO/MEGA/NANO
#define         RatioMQ2CleanAir        (9.83) //RS / R0 = 9.83 ppm 

/*****************************Globals***********************************************/
MQUnifiedsensor MQ2(Board, Voltage_Resolution, ADC_Bit_Resolution, Pin, Type);



ThingerESP32 thing(USERNAME, DEVICE_ID, DEVICE_CREDENTIAL);

#define INTERVAL_MESSAGE1 3000 //jeda waktu pengiriman ke server 3 detik
#define INTERVAL_MESSAGE2 5000 //jeda waktu pengiriman ke server 5 detik
unsigned long time_1 = 0;
unsigned long time_2 = 0;

void setup() {
  // open serial for debugging
  Serial.begin(115200); //komunikaasi serial pada baudrate 115200
  Wire.begin(); //Join I2C bus

  MQ2.setRegressionMethod(1); //_PPM =  a*ratio^b
  MQ2.setA(574.25); MQ2.setB(-2.222);
  MQ2.init();

  Serial.print("Calibrating sensor MQ-2 please wait.");
  float calcR0 = 0;
  for (int i = 1; i <= 10; i ++)
  {
    MQ2.update(); // Update data, the arduino will read the voltage from the analog pin
    calcR0 += MQ2.calibrate(RatioMQ2CleanAir);
    Serial.print(".");
  }
  MQ2.setR0(calcR0 / 10);
  Serial.println("  done!.");
  MQ2.serialDebug(true);

  pinMode(PIRsens, INPUT); //pin PIRsens sebagai INPUT
  pinMode(led1, OUTPUT); //pin led1 sebagai OUTPUT
  pinMode(led2, OUTPUT); //pin led2 sebagai OUTPUT
  pinMode(led3, OUTPUT); //pin led3 sebagai OUTPUT
  pinMode(led4, OUTPUT); //pin led4 sebagai OUTPUT
  
  for (int i = 0; i <= 10; i++) {
    digitalWrite(led1, !digitalRead(led1));
    digitalWrite(led2, !digitalRead(led2));
    digitalWrite(led3, !digitalRead(led3));
    digitalWrite(led4, !digitalRead(led4));
    delay(200);
  }

  digitalWrite(led1, HIGH);
  digitalWrite(led2, HIGH);
  digitalWrite(led3, HIGH);
  digitalWrite(led4, HIGH);

  if (aht20.begin() == false)
  {
    Serial.println("AHT20 not detected. Please check wiring. Freezing.");
    while (1);
  }

  thing.add_wifi(SSID, SSID_PASSWORD);

  // digital pin control example (i.e. turning on/off a light, a relay, configuring a parameter, etc)
  thing["LED 1"] << digitalPin(led1);
  thing["LED 2"] << digitalPin(led2);
  thing["LED 3"] << digitalPin(led3);
  thing["LED 4"] << digitalPin(led4);

  // resource output example (i.e. reading a sensor value)
  thing["temperature"] >> outputValue(temperature);
  thing["humidity"] >> outputValue(humidity);
  thing["LPG_gas"] >> outputValue(data_gas);
  thing["dataPIR"] >> outputValue(dataPIR);

  // more details at http://docs.thinger.io/arduino/
}

void loop() {
  thing.handle();

  if (millis() >= time_1 + INTERVAL_MESSAGE1) { //mengirimkan data sensor pir ke server
    time_1 += INTERVAL_MESSAGE1;
    dataPIR = digitalRead(PIRsens);
    Serial.print("data Pir = ");
    Serial.println(dataPIR);

  }


  if (millis() >= time_2 + INTERVAL_MESSAGE2) { //mengirimkan data sensor suhu dan juga MQ2 ke server
    time_2 += INTERVAL_MESSAGE2;
    MQ2.update();
    data_gas = MQ2.readSensor();
    Serial.print("Data gas LPG= ");
    Serial.println(data_gas);
    if (aht20.available() == true)
    {
      //Get the new temperature and humidity value
      temperature = aht20.getTemperature();
      humidity = aht20.getHumidity();

      //Print the results
      Serial.print("Temperature: ");
      Serial.print(temperature, 2);
      Serial.print(" C\t");
      Serial.print("Humidity: ");
      Serial.print(humidity, 2);
      Serial.print("% RH");
      Serial.println();
    }
  }

}
