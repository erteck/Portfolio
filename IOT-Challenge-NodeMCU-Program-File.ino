/*
Author: Erick Alberto Bustos Cruz

Using: SparkFun MAX3010x Pulse and Proximity Library

Script that:
1) Takes oxygen, temperature and heart rate measurments using a NODEMCU
2) Sends them back to the comuputer via WiFi
3) And publishes them in a Local Database

*/

// Internet Connection
#include <ESP8266WiFi.h>
#include <WiFiClient.h>

const char *ssid = "WiFi Name";
const char *password = "WiFi Password";

char host[48];
String strhost = "Your computer's IP";

String strurl = "/path/save_data_db.php"; // Location of php file to send information to the Data Base

//RGB Pin Declaration NodeMCU
int redLED  = 15;   //D8
int greenLED = 12;  //D5
int blueLED  = 14;   //D6

WiFiClient clientWiFi;

//Declarations for  MAX30102 sensor
#include <Wire.h>
#include "MAX30105.h"
#include "heartRate.h"
#include "spo2_algorithm.h"

MAX30105 particleSensor;
#define MAX_BRIGHTNESS 255

//Variables to obtain data
const byte RATE_SIZE = 4; 
byte hrates[RATE_SIZE]; 
long lastBeat = 0; 
byte rateSpot = 0;
float beatsPerMinute;
int beatAvg;
float valuesox[2]; 

//Variables to be saved in database
float temp; // Temperature
float heart_rate; // Heart Rate
float spo; // %SpO2


bool spocond = true;
bool heartRatecond = true;

//Creation of variables for oximetry measurement

#if defined(__AVR_ATmega328P__) || defined(__AVR_ATmega168__)
//Arduino Uno doesn't have enough SRAM to store 100 samples of IR led data and red led data in 32-bit format
//To solve this problem, 16-bit MSB of the sampled data will be truncated. Samples become 16-bit data.
uint16_t irBuffer[100]; //infrared LED sensor data
uint16_t redBuffer[100];  //red LED sensor data
#else

uint32_t irBuffer[100]; //infrared LED sensor data
uint32_t redBuffer[100];  //red LED sensor data
#endif

int32_t bufferLength; //data length
int32_t spo2; //SPO2 value
int8_t validSPO2; //indicator to show if the SPO2 calculation is valid
int32_t heartRate; //heart rate value
int8_t validHeartRate; //indicator to show if the heart rate calculation is valid

byte pulseLED = 13; //Must be on PWM pin
byte readLED = 16; //Blinks with each data read

//LED COLOR FUNCTIONS
void off()
{
  digitalWrite(redLED, LOW);
  digitalWrite(greenLED, LOW);
  digitalWrite(blueLED, LOW);
  delay(100);
}
void red()
{
  //RED
  digitalWrite(redLED, HIGH);
  digitalWrite(greenLED, LOW);
  digitalWrite(blueLED, LOW);
  delay(100);
}
void green()
{
  //GREEN
  digitalWrite(redLED, LOW);
  digitalWrite(greenLED, HIGH);
  digitalWrite(blueLED, LOW);
  delay(100);
}
void blue()
{
  //BLUE
  digitalWrite(redLED, LOW);
  digitalWrite(greenLED, LOW);
  digitalWrite(blueLED, HIGH);
  delay(100);
}

void yellow()
{
  //YELLOW
  digitalWrite(redLED, HIGH);
  digitalWrite(greenLED, HIGH);
  digitalWrite(blueLED, LOW);
  delay(100);

}

void cyan()
{
  //CYAN
  digitalWrite(redLED, LOW);
  digitalWrite(greenLED, HIGH);
  digitalWrite(blueLED, HIGH);
  delay(100);
}

void purple()
{
  //PURPLE
  digitalWrite(redLED, HIGH);
  digitalWrite(greenLED, LOW);
  digitalWrite(blueLED, HIGH);
  delay(100);

}

void white()
{
  //WHITE
  digitalWrite(redLED, HIGH);
  digitalWrite(greenLED, HIGH);
  digitalWrite(blueLED, HIGH);
  delay(100);
}

//Sends data to from NODEMCU to the computer
String senddata(String data)
{
  String line_error = "Connection Failed";
  String line_connection = "Successful Connection";
  int count = 0;

  WiFiClient client;
  strhost.toCharArray(host, 49);
  if (!client.connect(host, 80)) {
    red();
    return line_error;
  }


  client.print(String("POST ") + strurl + " HTTP/1.1" + "\r\n" +
               "Host: " + strhost + "\r\n" +
               "Accept: */*" + "*\r\n" +
               "Content-Length: " + data.length() + "\r\n" +
               "Content-Type: application/x-www-form-urlencoded" + "\r\n" +
               "\r\n" + data);
  delay(10);
  Serial.print("Sending data to SQL...");

  //If after 5 seconds, the client does not answer, terminate the request
  count = 0;
  while (client.available() == 0)
  {
    delay(1000);
    if (count == 4)
    {
      Serial.println("Client out of time!");
      client.stop();
      red();
      return line_error;
    }
    count++;
  }
  Serial.println("Data sent");
  green();

  // Reads all the lines it receives from the server and prints them through the serial terminal
  while (client.available()) {
    line_connection = client.readStringUntil('\r');
  }
  Serial.println(line_connection);
  return line_connection;
}


//Connects the NODEMCU to WiFi
void internetconnection() {
  Serial.print("Connecting to WiFi: ");
  Serial.println(ssid);
  WiFi.mode(WIFI_STA);
  WiFi.begin(ssid, password);
  while (WiFi.status() != WL_CONNECTED)
  {
    delay(500);
    Serial.print(".");

  }
  Serial.println("");
  Serial.println("WiFi connected");
  Serial.print("IP Adress: ");
  Serial.println(WiFi.localIP());
  Serial.println("");
  Serial.println("");

}


//Measures temperature with MAX30102 sensor
float temperature() {
  Serial.println("temperature\n");
  float temperature;
  for ( int j = 0; j < 51; j++) {
    temperature = particleSensor.readTemperature();
  }
  return temperature + 8;
}

//Measures oxigen levels with MAX30102 sensor
void oxygen() {
  bufferLength = 100; //buffer length of 100 stores 4 seconds of samples running at 25sps

  //read the first 100 samples, and determine the signal range
  for (byte i = 0 ; i < bufferLength ; i++)
  {
    while (particleSensor.available() == false) //do we have new data?
      particleSensor.check(); //Check the sensor for new data

    redBuffer[i] = particleSensor.getRed();

    irBuffer[i] = particleSensor.getIR();
    particleSensor.nextSample(); //We're finished with this sample so move to next sample

    Serial.print(F("red="));
    Serial.print(redBuffer[i], DEC);
    Serial.print(F(", ir="));
    Serial.println(irBuffer[i], DEC);
  }

  //calculate heart rate and SpO2 after first 100 samples (first 4 seconds of samples)
  maxim_heart_rate_and_oxygen_saturation(irBuffer, bufferLength, redBuffer, &spo2, &validSPO2, &heartRate, &validHeartRate);

  int contloop = 0;


  //Continuously taking samples from MAX30102.  Heart rate and SpO2 are calculated every 1 second
  while (1)
  {
    contloop = contloop + 1;
    //dumping the first 25 sets of samples in the memory and shift the last 75 sets of samples to the top
    for (byte i = 25; i < 100; i++)
    {
      redBuffer[i - 25] = redBuffer[i];
      irBuffer[i - 25] = irBuffer[i];
    }

    //take 25 sets of samples before calculating the heart rate.
    for (byte i = 75; i < 100; i++)
    {
      while (particleSensor.available() == false) //do we have new data?
        particleSensor.check(); //Check the sensor for new data

      digitalWrite(readLED, !digitalRead(readLED)); //Blink onboard LED with every data read

      redBuffer[i] = particleSensor.getRed();

      irBuffer[i] = particleSensor.getIR();
      particleSensor.nextSample(); //We're finished with this sample so move to next sample

      //send samples and calculation result to terminal program through UART
 
      Serial.print(F("HR="));
      Serial.print(heartRate, DEC);

      Serial.print(F(", HRvalid="));
      Serial.print(validHeartRate, DEC);

      Serial.print(F(", SPO2="));
      Serial.print(spo2, DEC);

      Serial.print(F(", SPO2Valid="));
      Serial.println(validSPO2, DEC);
    }

    maxim_heart_rate_and_oxygen_saturation(irBuffer, bufferLength, redBuffer, &spo2, &validSPO2, &heartRate, &validHeartRate);

    if (heartRate > 50 && heartRate < 110 && heartRatecond == true) {
      heartRatecond = false;
      valuesox[1] = heartRate;
    }
    if (spo2 < 101 && spo2 > 69 && spocond == true) {
      spocond = false;
      valuesox[0] = spo2;
    }
    if ((spocond == true || heartRatecond == true) && contloop > 10) {
      red();
      break;

    }
    if(spocond == false && heartRatecond == false){
      break;
    }

  }



}


void setup()
{
  //Definition of RGB LED
  pinMode(redLED, OUTPUT);
  pinMode(greenLED, OUTPUT);
  pinMode(blueLED, OUTPUT);

  purple();

  delay(3000);


  // Comenzar serial
  Serial.begin(115200);

  //Connect to WiFi
  blue(); //Indicator: Connected
  delay(1000);
  internetconnection();
  blue();

  //Initialize oxymetry pins
  pinMode(pulseLED, OUTPUT);
  pinMode(readLED, OUTPUT);

  // Initialize MAX30102
  if (!particleSensor.begin(Wire, I2C_SPEED_FAST)) //Use default I2C port, 400kHz speed
  {
    Serial.println("MAX30105 was not found. Please check wiring/power. ");
    while (1);
  }

  byte ledBrightness = 60; //Options: 0=Off to 255=50mA
  byte sampleAverage = 4; //Options: 1, 2, 4, 8, 16, 32
  byte ledMode = 2; //Options: 1 = Red only, 2 = Red + IR, 3 = Red + IR + Green
  byte sampleRate = 100; //Options: 50, 100, 200, 400, 800, 1000, 1600, 3200
  int pulseWidth = 411; //Options: 69, 118, 215, 411
  int adcRange = 4096; //Options: 2048, 4096, 8192, 16384

  particleSensor.setup(ledBrightness, sampleAverage, ledMode, sampleRate, pulseWidth, adcRange);

  Serial.println("Starting Measurments\n");
  green();
  delay(2000);
  off();
  green();
  delay(2000);
  off(); //Ready to use sensors
  green();
  delay(2000);
  off();
  green();
  delay(2000);


  //Measure oxygen and heart rate
  yellow(); //Indicator: In use
  oxygen();
  spo = valuesox[0];
  heart_rate = valuesox[1];

  particleSensor.enableDIETEMPRDY();
  
  // Measure temperature
  temp = temperature();
  delay(1000);

  Serial.println("Temperature:");
  Serial.println(temp);
  Serial.println("Heart Rate:");
  Serial.println(heart_rate);
  Serial.println("SPO2:");
  Serial.println(spo);

  
  if (spocond == false && heartRatecond == false) {
    cyan(); // Indicator: I finished taking the measurements and I am sending them
    delay(2000);
    senddata("oxygen=" + String(spo) + "&heart_rate=" + String(heart_rate) + "&temperature=" + String(temp, 2));
  }
  //Indicator: green if  sent  correctly, red if not
}

void loop() {

}
