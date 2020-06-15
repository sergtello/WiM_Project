//Source code - Arduino version
//WiN Project 1.0v - United Nations Development Programme
//Written by Sergio Tello for Ecobridge SAC
/*Hardware Description:
  uC : ESP32 Firebeetle
  Humidity sensor: DHT22
  Temperature sensor: DBS18B20
  Altitude and Atmospheric Pressure sensor: BMP180
  Dust sensor: GP2Y1010AU0F
  LoRa Radio module: RAK811 Breakout Board
  Reed switch
  SD Card module
  18650 Li-ion 3.7V Battery 
*/

//Required Arduino libraries
//DHT22
#include <Adafruit_Sensor.h>
#include <DHT.h>
#include <DHT_U.h>

//DBS18B20
#include <OneWire.h>
#include <DallasTemperature.h>

//BMP180
#include <Wire.h>
//#include <Adafruit_Sensor.h>              /* Already included */
#include <Adafruit_BMP085_U.h>

//SD Card module
#include "FS.h"
#include "SD.h"
#include <SPI.h>

//Get timestamp
#include <WiFi.h>
#include "time.h"

//Reed switch - ULP
#include "esp_sleep.h"
#include "driver/rtc_io.h"
#include "esp32/ulp.h"
#include "ulp_main.h"


//Pin definition and initialization
//DHT22
#define DHTPIN      15 
#define DHTTYPE     DHT22
DHT_Unified dht(DHTPIN, DHTTYPE);

//DBS18B20
#define ONE_WIRE_BUS 4
OneWire oneWire(ONE_WIRE_BUS);
DallasTemperature dbs18(&oneWire);

//BMP180 
//Default I2C pins for the ESP32 board were selected
/*
 * BMP_SDA_PIN = 21
 * BMP_SCL_PIN = 22
 */
#define PRESSURE_SEALEVELQNH  1014.0F           /* Average sea level pressure at your location in hPa */
const int BMP180_ID = 10085;                    /* Sensor ID, useful in case of using many BMP180 sensors */
Adafruit_BMP085_Unified bmp = Adafruit_BMP085_Unified(BMP180_ID);

//GP2Y1010AU0F
#define DUST_MEASURE_PIN 35
#define DUST_IRLED_PIN   27
const int samplingTime  = 280;                 /* Time constants expressed in microseconds */
const int deltaTime     = 40;
const int sleepTime     = 9680;

//LoRa Radio Module RAK811
//Default second UART channel pins for the ESP32 board were selected
/*
 * RX_UART2_PIN = 16
 * TX_UART2_PIN = 17
 */
 #define LORAWAN_PORT 1                       /* LoRaWAN port used for sending the messages */

//SD Card module
//Default VSPI pins for the ESP32 board were selected
/*
 * SD_MISO_PIN = 19
 * SD_MOSI_PIN = 23
 * SD_CLK_PIN  = 18
 */
#define SD_CS_PIN 5
//Select the file format for the data output type
#define OUTPUT_DATA_FORMAT 1                /* 1 = ".csv" file extension, 0 = ".txt" file extension */
const char* file_name =    "/data";         /* Name must start with the '/' character */
RTC_DATA_ATTR char file_name_type[30];

//Timestamp
const char* ssid       = "YOUR_SSID";           /* WiFi credentials */    
const char* password   = "YOUR_PASSWORD";
const char* ntpServer = "pool.ntp.org";     /* NTP server selection */
const char* TimeZone =  "EST+5";            /* Timezone selection */

//Deep Sleep
#define uS_TO_S_FACTOR 1000000ULL      /* Conversion factor for micro seconds to seconds */
#define TIME_TO_SLEEP  120              /* Time ESP32 will go to sleep (in seconds) */
RTC_DATA_ATTR int bootCount = 0;      /* Boot counter stored in RTC memory */

//Reed switch - ULP
#define ULP_WAKEUP_PERIOD  250        /* ULP wake up period in miliseconds */
extern const uint8_t ulp_main_bin_start[] asm("_binary_ulp_main_bin_start");
extern const uint8_t ulp_main_bin_end[]   asm("_binary_ulp_main_bin_end");
gpio_num_t ulp_2 = GPIO_NUM_2;        /* Enabling GPIO2 to be used by the ULP script */


//External wake up
#define EXT_WAKEUP_PIN GPIO_NUM_25      /* ESP32 will wake up when the GPIO2 is HIGH */
#define WAKEUP_STATE HIGH

/* #define EXT_INTERRUPT_PIN    2 */        /* Enabling ESP32 external interrupt on GPIO2 */
/* #define EXT_INTERRUPT_SOURCE RISING */
/* uint8_t reed_switch_flag=0;  */           /* Increase its value with each tipping-bucket movement */
/* RTC_DATA_ATTR int tippingCount = 0; */


//Functions and methods required
//DHT22 /////////////////////////////////////////
void init_dht22(){
  dht.begin();
  sensor_t sensor;
// Print humidity sensor details.
  dht.humidity().getSensor(&sensor);  
  Serial.println(F("Humidity Sensor"));
  Serial.print  (F("Sensor Type: ")); Serial.println(sensor.name);
  Serial.print  (F("Driver Ver:  ")); Serial.println(sensor.version);
  Serial.print  (F("Unique ID:   ")); Serial.println(sensor.sensor_id);
  Serial.print  (F("Max Value:   ")); Serial.print(sensor.max_value); Serial.println(F("%"));
  Serial.print  (F("Min Value:   ")); Serial.print(sensor.min_value); Serial.println(F("%"));
  Serial.print  (F("Resolution:  ")); Serial.print(sensor.resolution); Serial.println(F("%"));
  //Serial.println(sensor.min_delay);
  Serial.println(F("------------------------------------\n"));
}

void get_humidity_measure(float* humidity_value){
   //dht.begin();
   sensors_event_t event;
// Get humidity event and print its value.
   dht.humidity().getEvent(&event);
   if (isnan(event.relative_humidity)) {
    Serial.println(F("Error reading humidity!"));
   }
   else {
    *humidity_value = constrain(event.relative_humidity,0.0,100.0);
    Serial.print(F("Humidity: "));
    Serial.print(*humidity_value);
    Serial.println(F("%"));
   }
}
////////////////////////////////////////////////

//DBS18B20 /////////////////////////////////////
void get_temperature_measure(float* temperature_value){
   dbs18.begin();
// Call sensors.requestTemperatures() to issue a global temperature 
// Request to all devices on the bus
   Serial.print(F("Requesting temperatures..."));
   dbs18.requestTemperatures(); // Send the command to get temperatures
   Serial.println(F("DONE"));
// After we got the temperatures, we can print them here.
// We use the function ByIndex in order to get the temperature from the first sensor only, in our case there is just one.
   *temperature_value = constrain(dbs18.getTempCByIndex(0),0.0,100.0);
   Serial.print(F("Temperature for the sensor (index 0) is: "));
   Serial.print(*temperature_value);  
   Serial.println(F("°C"));
}
/////////////////////////////////////////////////

//BMP180/////////////////////////////////////////
void init_bmp180(){
  if(!bmp.begin()){
 // There was a problem detecting the BMP085 ... check your connections */
    Serial.print(F("Ooops, no BMP085 detected ... Check your wiring or I2C ADDR!"));
    ESP.restart();
  }
  sensor_t sensor;
  bmp.getSensor(&sensor);
  Serial.println(F("Atmospheric pressure Sensor"));
  Serial.print  (F("Sensor:       ")); Serial.println(sensor.name);
  Serial.print  (F("Driver Ver:   ")); Serial.println(sensor.version);
  Serial.print  (F("Unique ID:    ")); Serial.println(sensor.sensor_id);
  Serial.print  (F("Max Value:    ")); Serial.print(sensor.max_value); Serial.println(F(" hPa"));
  Serial.print  (F("Min Value:    ")); Serial.print(sensor.min_value); Serial.println(F(" hPa"));
  Serial.print  (F("Resolution:   ")); Serial.print(sensor.resolution); Serial.println(F(" hPa"));  
  Serial.println(F("------------------------------------\n"));
}

void get_pressure_measure(float* pressure_value, float* altitude_value){
  bmp.begin();
  sensors_event_t event;
 // Get a new pressure event
  bmp.getEvent(&event);
  if (event.pressure){
 // Display atmospheric pressure in hPa 
    *pressure_value = constrain(event.pressure,0.0,1500.0);
    Serial.print(F("Pressure:    "));
    Serial.print(*pressure_value);
    Serial.println(F(" hPa")); 
    /* Calculating altitude with reasonable accuracy requires pressure    *
     * sea level pressure for your position at the moment the data is     *
     * converted                                                          */
//Then convert the atmospheric pressure, and SLP to altitude         
    float seaLevelPressure = PRESSURE_SEALEVELQNH;
    *altitude_value = constrain(bmp.pressureToAltitude(seaLevelPressure,*pressure_value),0.0,5000.0);
    Serial.print(F("Altitude:    ")); 
    Serial.print(*altitude_value); 
    Serial.println(F(" MASL"));
  }
  else  Serial.println(F("Sensor error"));
  
}
/////////////////////////////////////////////////

//GP2Y1010AU0F //////////////////////////////////
void get_dust_measure(uint8_t measurePin, uint8_t ledPower, float* dust_density_value){
 int voMeasured = 0;
 float calcVoltage = 0.0;
 float dustDensity = 0.0;
 // Power on the IR LED
  pinMode(ledPower,OUTPUT);
  digitalWrite(ledPower,LOW); 
  delayMicroseconds(samplingTime);
 // Read the dust density value
  voMeasured = analogRead(measurePin); 
  delayMicroseconds(deltaTime);
  digitalWrite(ledPower,HIGH); // turn the LED off
  delayMicroseconds(sleepTime);
 
 // 0 - 3.3V mapped to 0 - 4095 integer values
  calcVoltage = voMeasured * (3.3 / 4096.0);
 // Linear equation taken from http://www.howmuchsnow.com/arduino/airquality/
  dustDensity = 0.17 * calcVoltage - 0.03;
  *dust_density_value = constrain(dustDensity,0.0,2.0); 
  
  Serial.print(F("Raw Signal Value (0-4095): "));
  Serial.print(voMeasured);
 
  Serial.print(F(" - Voltage: "));
  Serial.println(calcVoltage);
 
  Serial.print(F("Dust Density: "));
  Serial.print(*dust_density_value,5);
  Serial.println(F(" mg/m^3"));
}
///////////////////////////////////////////////

//LoRa Radio Module RAK811 ////////////////////
//Assuming that the region, device class, join mode and credentials have been configured in advance
void join_lora_server(){
  String at_join = "at+join";
  String response;
  Serial2.setTimeout(7000);
  //Joining the LoRaWAN Server
  Serial2.print(at_join + "\r\n");
  //Waiting for the confirmation
  response = Serial2.readString();
  if(strcmp(response.c_str(),"OK Join Success")){
    Serial.println(F("Connected to the LoRaWAN Server"));
  }
  else{
    Serial.println(F("Unable to connect to the LoRaWAN Server"));
  }
}

void send_lora_message(char* lora_message,uint8_t lora_port){
  String at_send = "at+send=lora:";
  //Selecting the appropiate port
  String port = String(lora_port) + ":";
  String message = String(lora_message);
  String response;
  Serial2.setTimeout(7000);
  //Sending the message
  Serial2.print(at_send + port + message + "\r\n");
  //Waiting for the confirmation
  response = Serial2.readString();
  if(strcmp(response.c_str(),"OK")){
    Serial.println(F("Message successfully sent to the LoRaWAN Server"));
  }
  else{
    Serial.println(F("Failed to send the message to the LoRaWAN Server"));
  }
}

void lora_radio_sleep(uint8_t sleep_state){
  String at_sleep = "at+set_config=device:sleep:";
  String sleep = String(sleep_state);
  String response;
  Serial2.setTimeout(7000);
  //Sending the radio to sleep
  Serial2.print(at_sleep + sleep + "\r\n");
  //Waiting for the confirmation
  response = Serial2.readString();
  if(sleep_state){
    if(strcmp(response.c_str(),"OK Sleep")){
      Serial.println(F("The LoRa radio module is in sleep mode"));
    }
    else{
      Serial.println(F("Failed to enter sleep mode"));
    }
  }
  else{
    if(strcmp(response.c_str(),"OK Wake Up")){
      Serial.println(F("The LoRa radio module is in regular working mode"));
    }
    else{
      Serial.println(F("Failed to enter working mode"));
    }
  }
}
///////////////////////////////////////////////

//Reed switch - ULP //////////////////////////
static void init_ulp_program(){
  esp_err_t err = ulp_load_binary(0, ulp_main_bin_start,
                                  (ulp_main_bin_end - ulp_main_bin_start) / sizeof(uint32_t));
  ESP_ERROR_CHECK(err);
  rtc_gpio_init(ulp_2);
  rtc_gpio_set_direction(ulp_2, RTC_GPIO_MODE_INPUT_ONLY);

  /* Set ULP wake up period */
  ulp_set_wakeup_period(0, ULP_WAKEUP_PERIOD * 1000);
  /* Disable pullup on GPIO15, in case it is connected to ground to suppress
     boot messages. */
  //  rtc_gpio_pullup_dis(GPIO_NUM_15);
  //  rtc_gpio_hold_en(GPIO_NUM_15);
}

static void start_ulp_program(){
  /* Start the program */
  esp_err_t err = ulp_run((&ulp_entry - RTC_SLOW_MEM) / sizeof(uint32_t));
  ESP_ERROR_CHECK(err);
}
/////////////////////////////////////////////
/*
//External interrupt ///////////////
void IRAM_ATTR isr(){
  reed_switch_flag=1;
  tippingCount++;
  Serial.println(F("***"));
}
//////////////////////////////////////////
*/

//SD Card module ///////////////////////
void initialize_SD(uint8_t SD_CS){
      SD.begin(SD_CS);
      if(!SD.begin(SD_CS)){
        Serial.println(F("Card Mount Failed"));
        ESP.restart();
      }
      uint8_t cardType = SD.cardType();
      if(cardType == CARD_NONE){
        Serial.println(F("No SD card attached"));
         ESP.restart();
      }
      Serial.println(F("Initializing SD card..."));
}

void initialize_File(const char* filename, const char* initial_message){
  File file = SD.open(filename);
      if(!file) {
        Serial.println(F("File doens't exist"));
        Serial.println(F("Creating file..."));
        writeFile(SD, filename, initial_message);
      }
      else {
        Serial.println(F("File already exists"));  
      }
      file.close();
}

void writeFile(fs::FS &fs, const char * path, const char * message){
    Serial.printf("Writing file: %s\n", path);

    File file = fs.open(path, FILE_WRITE);
    if(!file){
        Serial.println(F("Failed to open file for writing"));
        return;
    }
    if(file.print(message)){
        Serial.println(F("File written"));
    } else {
        Serial.println(F("Write failed"));
    }
    file.close();
}

void appendFile(fs::FS &fs, const char * path, const char * message){
    Serial.printf("Appending to file: %s\n", path);

    File file = fs.open(path, FILE_APPEND);
    if(!file){
        Serial.println(F("Failed to open file for appending"));
        return;
    }
    if(file.print(message)){
        Serial.println(F("Message appended"));
    } else {
        Serial.println(F("Append failed"));
    }
    file.close();
}
///////////////////////////////////////

//Deep Sleep ///////////////////////////
void print_wakeup_reason(){
  esp_sleep_wakeup_cause_t wakeup_reason;

  wakeup_reason = esp_sleep_get_wakeup_cause();

  switch(wakeup_reason)
  {
    case ESP_SLEEP_WAKEUP_EXT0 : Serial.println(F("Wakeup caused by external signal using RTC_IO")); break;
    case ESP_SLEEP_WAKEUP_EXT1 : Serial.println(F("Wakeup caused by external signal using RTC_CNTL")); break;
    case ESP_SLEEP_WAKEUP_TIMER : Serial.println(F("Wakeup caused by timer")); break;
    case ESP_SLEEP_WAKEUP_TOUCHPAD : Serial.println(F("Wakeup caused by touchpad")); break;
    case ESP_SLEEP_WAKEUP_ULP : Serial.println(F("Wakeup caused by ULP program")); break;
    default : Serial.printf("Wakeup was not caused by deep sleep: %d\n",wakeup_reason); break;
  }
}
//////////////////////////////////////////////////

//Timestamp /////////////////////////////////////
void connect_NTP_server(){
    Serial.printf("Connecting to %s ", ssid);
    WiFi.begin(ssid, password);
    while (WiFi.status() != WL_CONNECTED) {
        delay(500);
        Serial.print(F("."));
    }
    Serial.println(F(" CONNECTED"));
    
//Init and get the time
    configTime(0, 0, ntpServer);
    get_time_date();

    char first_test[30];
    printLocalTime(first_test);
//Disconnect WiFi as it is no longer needed
    WiFi.disconnect(true);
    WiFi.mode(WIFI_OFF);
}

void get_time_date(){
  // TZ string information: https://www.gnu.org/software/libc/manual/html_node/TZ-Variable.html
  setenv("TZ", TimeZone, 1);    /* Save the TZ variable */
  tzset();
}

void printLocalTime(char * time_date){
  char date[30];
  struct tm timeinfo;
  if(!getLocalTime(&timeinfo)){
    Serial.println(F("Failed to obtain time"));
    return;
  }
  //tm struct to string format information: http://www.cplusplus.com/reference/ctime/strftime/
  //Serial.println(&timeinfo, "%d/%m/%Y - %H:%M:%S");
  if(OUTPUT_DATA_FORMAT){
    strftime(date,30,"%d/%m/%Y,%H:%M:%S",&timeinfo);
    strcpy(time_date,date);
  }
  else{
    strftime(date,30,"%d/%m/%Y - %H:%M:%S",&timeinfo);
    strcpy(time_date,date);
  }
  Serial.println(time_date);

}
///////////////////////////////////////////////////////


void setup() {
  Serial.begin(115200);
  Serial2.begin(115200); /* Oppening the serial port destinated to the radio */ 
  delay(100);           /* Take some time to open up the Serial Monitor */
  //pinMode(DUST_IRLED_PIN,OUTPUT);
  //attachInterrupt(EXT_INTERRUPT_PIN, isr, EXT_INTERRUPT_SOURCE);
  //Increment boot number and print it every reboot
  ++bootCount;
  Serial.println("Boot number: " + String(bootCount));
  Serial.println(F("Setting up the LoRa radio module to regular work mode"));
  lora_radio_sleep(0);
  delay(500);
  //Executed during the first boot
  if(bootCount==1){
    connect_NTP_server();
    Serial.println(F("Joining the LoRaWAN Server"));
    join_lora_server();
    delay(500);
    //Initial confirmation message
    send_lora_message("01",LORAWAN_PORT);
    delay(500);
    init_ulp_program();
    initialize_SD(SD_CS_PIN);
    if(OUTPUT_DATA_FORMAT){
      char* init_message = "Measurement,Date(dd/mm/yy),Time(hh:mm:ss),Relative humidity(%),Ambient Temperature(Celsius),Atmospheric pressure(hPa),Altitude(MASL),Dust density(mg/m^3),Tipping-bucket counter\n";
      strcat(file_name_type,file_name);
      strcat(file_name_type,".csv");
      initialize_File(file_name_type,init_message);
    }
    else {
      char* init_message ="\t\tMeasurement of ambient parameters\n\n";
      strcat(file_name_type,file_name);
      strcat(file_name_type,".txt");
      initialize_File(file_name_type,init_message);
    } 
  }

  print_wakeup_reason();

  if(bootCount>=2){
    initialize_SD(SD_CS_PIN);
    get_time_date();

    char date_time[30],counter_format[25],lora_message_format[50];
    float humidity_val=0.0, temperature_val=0.0, pressure_val=0.0, altitude_val=0.0, dust_density_val=0.0;
    
    printLocalTime(date_time);
    delay(500);
    init_dht22();
    delay(500);
    get_temperature_measure(&temperature_val);
    delay(500);
    get_pressure_measure(&pressure_val,&altitude_val);
    delay(500);
    get_dust_measure(DUST_MEASURE_PIN,DUST_IRLED_PIN,&dust_density_val);
    delay(500);
    get_humidity_measure(&humidity_val);
    delay(500);
    Serial.println("Number of tipping-bucket counts:  "+ String(ulp_tip_counter & UINT16_MAX));
    String message;

    if(OUTPUT_DATA_FORMAT){
      message=String(bootCount-1)+ ","+ String(date_time)+ ","+ String(humidity_val,2)+
              ","+ String(temperature_val,2)+ ","+ String(pressure_val,2)+ ","+
              String(altitude_val,2)+ ","+ String(dust_density_val,5)+ ","+
              String(ulp_tip_counter & UINT16_MAX)+"\n";
    }

    else{
      sprintf(counter_format,"%6d.- Timestamp:",bootCount-1);
      message= String(counter_format)+ "            " + String(date_time)+" \r\n"+
              "\t Relative humidity:     "+ String(humidity_val,2)+ " % \r\n"+
              "\t Ambient temperature:   "+ String(temperature_val,2)+ " °C \r\n"+
              "\t Atmospheric pressure:  "+ String(pressure_val,2)+ " hPa \r\n"+
              "\t Altitude:              "+ String(altitude_val,2)+ " MASL \r\n"+
              "\t Dust density:          "+ String(dust_density_val,5)+ " mg/m^3 \r\n"+
              "\t Tipping-bucket counter:"+ String(ulp_tip_counter & UINT16_MAX)+ "tips \r\n\n";  
    }
    Serial.print(F("Save data:  \n"));
    Serial.println(message);
    appendFile(SD,file_name_type,message.c_str());

    sprintf(lora_message_format,"%04x%04x%04x%04x%06x%06x", (int)floor(humidity_val*100),
                                                            (int)floor(temperature_val*100),
                                                            (int)floor(dust_density_val*100000),
                                                            (int)(ulp_tip_counter & UINT16_MAX),
                                                            (int)floor(pressure_val*100),
                                                            (int)floor(altitude_val*100));
    Serial.println(F("Sending message to the LoRaWAN Server"));
    send_lora_message(lora_message_format,LORAWAN_PORT);
    delay(500);
  }
  start_ulp_program();
  //ESP_ERROR_CHECK( esp_sleep_enable_ulp_wakeup() );
  esp_sleep_enable_ext0_wakeup(EXT_WAKEUP_PIN,WAKEUP_STATE);
  esp_sleep_enable_timer_wakeup(TIME_TO_SLEEP * uS_TO_S_FACTOR);
  
  Serial.println(F("Setting up the LoRa radio module to sleep mode"));
  lora_radio_sleep(1);
  delay(500);
  
  Serial.println("Setup ESP32 to sleep for every " + String(TIME_TO_SLEEP) + " Seconds");
  Serial.println(F("Going to sleep now"));
  Serial.flush(); 
  esp_deep_sleep_start();
}

void loop() {
  ESP.restart();
  
}
