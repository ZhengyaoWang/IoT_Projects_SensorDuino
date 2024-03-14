//This is a Function Demonstration Program for SensorDuinoV2.0 
//Board and Program Designed and Tested by Zhengyao William Wang
//2024/03/13 Version 1.01

#include <Wire.h>
#include <BH1750.h>

#define MPU6050_addr 0x68
#define BH1750_addr 0x23
#define SHT30_addr 0x44
#define HP203N_addr 0x77
#define ZE08_addr 0x32

#define MQ2_Switch_Pin 5
#define MQ2_Read_Pin 36
#define Battery_Monitor_Pin 39

#define ZE08_Switch_Pin 4

#define LED_Pin 14
#define Buzzer_Pin 32

#define SensorDuino_RXD 17
#define SensorDuino_TXD 16

BH1750 On_Board_BH1750;

void setup() {

  Serial.begin(9600);
  Serial2.begin(9600, SERIAL_8N1, SensorDuino_TXD, SensorDuino_RXD);

  Wire.begin();
  On_Board_BH1750.begin(BH1750::CONTINUOUS_HIGH_RES_MODE, BH1750_addr);  
   

  pinMode(MQ2_Switch_Pin, OUTPUT);
  pinMode(ZE08_Switch_Pin, OUTPUT);

  pinMode(MQ2_Read_Pin,INPUT);
  pinMode(Battery_Monitor_Pin,INPUT);

  pinMode(LED_Pin, OUTPUT);
  pinMode(Buzzer_Pin, OUTPUT);

}

void loop() {

  digitalWrite(ZE08_Switch_Pin, HIGH); //turn on MQ2 and ZE08 all the time for testing, will not use in normal mode cauz it consumes too much power
  digitalWrite(MQ2_Switch_Pin, HIGH);
  
  Serial.println("/////////////////////Data Received!/////////////////////");

  delay(500);
////////////////////////////////////////////////////////////////////////////////////////
  // Part 1, obtain temperature and humidity data from SHT30
  // Due to received values are in String datatype, datatype conversion required
  float SHT30_ReadTempHumidity_RECEIVED[3];
  String SHT30_ReadTempHumidity_RAW = SHT30_ReadTempHumidity(SHT30_addr);
  byte index = 0;

  char data[SHT30_ReadTempHumidity_RAW.length()];
  SHT30_ReadTempHumidity_RAW.toCharArray(data, SHT30_ReadTempHumidity_RAW.length());

  char* SHT30_ELEMENT = strtok(data,",");
  while (SHT30_ELEMENT != NULL)
  {
    SHT30_ReadTempHumidity_RECEIVED[index] = atof(SHT30_ELEMENT);
    index++;
    SHT30_ELEMENT = strtok(NULL,",");
  }

  double SHT30_Temp_C = SHT30_ReadTempHumidity_RECEIVED[0];
  double SHT30_Temp_F = SHT30_ReadTempHumidity_RECEIVED[1];
  double SHT30_Humidity_Level = SHT30_ReadTempHumidity_RECEIVED[2];

  delay(100);
////////////////////////////////////////////////////////////////////////////////////////
  //Part 2, obtain luminosity reading from BH1750
  double BH1750_Luminosity_lux = On_Board_BH1750.readLightLevel();
  
  delay(100);
////////////////////////////////////////////////////////////////////////////////////////
  //Part 3, obtain pressure reading in kPa from HP203N
  double HP203N_Pressure_kPa = HP203N_ReadPressure(HP203N_addr,4096);
  
  delay(100);
////////////////////////////////////////////////////////////////////////////////////////
  //Part 4, obtain smoke and combustile gas reading from MQ-2B in percentage
  double MQ2_Gas_Level = MQ2_ReadGas(MQ2_Switch_Pin,MQ2_Read_Pin)/4096*100;
  
  delay(100);
////////////////////////////////////////////////////////////////////////////////////////
  //Part 5, obtain formaldehyde (CH2O) reading from ZE08 in mg/m3
  double ZE08_CH2O_mgm3 = ZE08_ReadFormaldehyde(ZE08_Switch_Pin);
  
  delay(100);
////////////////////////////////////////////////////////////////////////////////////////
  //Part 6, test the LED and Buzzer
  LED_ON_OFF(LED_Pin, 1);
  delay(100);
  LED_ON_OFF(LED_Pin, 0);
  delay(100);
  LED_Dimming(LED_Pin, 4000, 0, 8, 1); //PWM freq = 4000, duty cycle = 1/255*100%
  Buzzer_Dimming(Buzzer_Pin, 300, 1, 8, 20); //PWM freq = 300, duty cycle = 50/255*100%
  delay(100);
  LED_Dimming(LED_Pin, 4000, 0, 8, 0);
  Buzzer_Dimming(Buzzer_Pin, 500, 1, 8, 0); 

  //Print readings to Serial Port
  Serial.print("\nAmbient Light Strength = ");Serial.print(BH1750_Luminosity_lux);Serial.println(" lux");delay(10);
  Serial.print("\nTemperature Celsius    = ");Serial.print(SHT30_Temp_C);Serial.println(" C");delay(10);
  Serial.print("\nTemperature Fahrenheit = ");Serial.print(SHT30_Temp_F);Serial.println(" F");delay(10);
  Serial.print("\nRelative Humidity      = ");Serial.print(SHT30_Humidity_Level);Serial.println(" %");delay(10);
  Serial.print("\nAtmospheric Pressure   = ");Serial.print(HP203N_Pressure_kPa);Serial.println(" kPa");delay(10);
  Serial.print("\nSmoke   = ");Serial.print(MQ2_Gas_Level);Serial.println(" %");delay(10);
  Serial.print("\nCH2O   = ");Serial.print(ZE08_CH2O_mgm3);Serial.println("mg/m3");delay(10);
  Serial.println();

  delay(100);
}

float HP203N_ReadPressure(int HP203N_ADDR,int HP203N_SAMPLE_RATE)
{
  Wire.beginTransmission(HP203N_ADDR);

  /*Sample rate: 0x00:4096, 0x01:2048, 0x08:1024, 0x03:512*/
  int HP203N_SAMPLE_RATE_HEX;
  switch(HP203N_SAMPLE_RATE)
  {
    case 4096:
      HP203N_SAMPLE_RATE_HEX = 0x00;
    break;

    case 2048:
      HP203N_SAMPLE_RATE_HEX = 0x01;
    break;

    case 1024:
      HP203N_SAMPLE_RATE_HEX = 0x08;
    break;

    case 512:
      HP203N_SAMPLE_RATE_HEX = 0x03;
    break;
  }

  Wire.write(0x40|HP203N_SAMPLE_RATE_HEX);
  Wire.endTransmission(HP203N_ADDR);

  delay(100);

  Wire.beginTransmission(HP203N_ADDR);
  Wire.write(0x30);//read both temp & pres, 0x30 for pres only, 0x32 for temp only
  Wire.endTransmission(HP203N_ADDR);

  Wire.requestFrom(HP203N_ADDR,6);//for only pres or only temp, use 3 bytes

  float HP203N_PRES;
  if(Wire.available())
  {
    long BYTE_READ;
    BYTE_READ = Wire.read();
    BYTE_READ <<= 8;
    BYTE_READ |= Wire.read();
    BYTE_READ <<= 8;
    BYTE_READ |= Wire.read();
    
    HP203N_PRES = float(BYTE_READ)/1000.00;
    
  }
  return HP203N_PRES;
}

String SHT30_ReadTempHumidity(int SHT30_ADDR)
{
  unsigned int data[6]; 

  Wire.beginTransmission(SHT30_ADDR);
  //Measurement command 0x2C26
  Wire.write(0x2C);
  Wire.write(0x06);
  Wire.endTransmission();

  delay(50);
  

  Wire.requestFrom(SHT30_ADDR, 6);

  if (Wire.available() == 6)
  {
    data[0] = Wire.read(); //temp high byte
    data[1] = Wire.read(); //temp low byte
    data[2] = Wire.read(); //temp crc
    data[3] = Wire.read(); //humidity high byte
    data[4] = Wire.read(); //humidity low byte
    data[5] = Wire.read(); //humidity crc
  }

  //convert from bytes to degree Celsius, Fahrenheit, and humidity
  float cTemp = ((((data[0] * 256.0) + data[1]) * 175) / 65535.0) - 45;
  float fTemp = (cTemp * 1.8) + 32;
  float humidity = ((((data[3] * 256.0) + data[4]) * 100) / 65535.0);

  String SHT30_ReadTempHumidity = (String(cTemp)+","+String(fTemp)+","+String(humidity));

  return SHT30_ReadTempHumidity;
}

float MQ2_ReadGas(int MQ2_SWICH_PIN, int MQ2_READ_PIN)
{
    //digitalWrite(MQ2_SWITCH_PIN,HIGH);
    //output range of sensor: 2.5V-3.3V (4.0V originally, 3.6V due to on-board Voltage Divider)
    //for ESP Boards, the ADC range is 0-3.3V:0-4095
    int ADC_READ = analogRead(MQ2_READ_PIN);

    return ADC_READ;

}

float ZE08_ReadFormaldehyde(int ZE08_SWITCH_PIN)
{
  //digitalWrite(ZE08_SWITCH_PIN,HIGH);
  
  delay(1500);
  
  int incomingByte = 0; 
  int Byte_read[9] = {0};
  double CH2O_concentration = 0;

  delay(1500);

  delay(5);
      if(Serial2.available() > 0)
      {
          for (int i=0; i<9; i++){
            incomingByte = Serial2.read();
            // Serial.print(i); //For Tunning
            // Serial.print("read:"); //For Tunning
            // Serial.println(incomingByte); //For Tunning
            delay(20);
            Byte_read[i] = incomingByte; 
            // Serial.println(Byte_read[i]); //For Tunning                  
      } 
          CH2O_concentration = (Byte_read[4]*256 + Byte_read[5])*1.25/1000;
      }
      return CH2O_concentration;
}

void LED_ON_OFF(int LED_pin,bool LED_State)
{

    if(LED_State)
    {
      digitalWrite(LED_pin, HIGH);
    }
    else 
    {
      digitalWrite(LED_pin,LOW);
    }  

}

void LED_Dimming(int LED_pin, double LED_freq, int LED_channel, int LED_resolution, int LED_dutyCycle)
{
  //set up PWM channel
  ledcSetup(LED_channel, LED_freq, LED_resolution);
  // attach the channel to the GPIO to be controlled
  ledcAttachPin(LED_pin, LED_channel);

  ledcWrite(LED_channel, LED_dutyCycle);
}

void Buzzer_Dimming(int Buzzer_pin, double Buzzer_freq, int Buzzer_channel, int Buzzer_resolution, int Buzzer_dutyCycle)
{
  //set up PWM channel
  ledcSetup(Buzzer_channel, Buzzer_freq, Buzzer_resolution);
  // attach the channel to the GPIO to be controlled
  ledcAttachPin(Buzzer_pin, Buzzer_channel);

  ledcWrite(Buzzer_channel, Buzzer_dutyCycle);
}
