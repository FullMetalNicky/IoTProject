/*
 * Copyright (c) 2019-2022 Piotr Stolarz
 * OneWireNg: Arduino 1-wire service library
 *
 * Distributed under the 2-clause BSD License (the License)
 * see accompanying file LICENSE for details.
 *
 * This software is distributed WITHOUT ANY WARRANTY; without even the
 * implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
 * See the License for more information.
 */

/**
 * Dallas family thermometers access example (Arduino).
 *
 * Required configuration:
 * - @c CONFIG_SEARCH_ENABLED for non single sensor setup,
 * - @c CONFIG_PWR_CTRL_ENABLED if @c PWR_CTRL_PIN is configured.
 */
#include "OneWireNg_CurrentPlatform.h"
#include "drivers/DSTherm.h"
#include "utils/Placeholder.h"

/*
 * 1-wire bus pin number.
 */
#ifndef OW_PIN
# define OW_PIN         6
#endif
float TEMP_HIGH = 30;
float TEMP_LOW = 10;

#define PHSensorPin A0          // the pH meter Analog output is connected with the Arduino’s Analog
float PH_LOW = 6.5;
float PH_HIGH = 8.5;

#include <DFRobot_Geiger.h>
#if defined ESP32
#define geiger_pin D3
#else
#define geiger_pin 3
#endif
DFRobot_Geiger  geiger(geiger_pin);
float GEIGER_HIGH = 200;

//#include "PERIPUMP.h"
#include "Servo.h"

Servo s;
#define pump_pin 5
#define pump_time 50000
#define wait_time 1000

#include <TinyGPS.h>
TinyGPS gps;
float flat, flon;
float last_flat = 0.0;
float last_flon = 0.0;
float dlon = 0.0; // 0.0001
float dlat = 0.0; // 0.0001

#include "AS726X.h"
AS726X SpectralSensor;

long int last_time;
long int dtime = 0; // 1000 * 60 * 60 * 24

#define RED_LED_PIN 1
#define GREEN_LED_PIN 2

#include <FlashIAPBlockDevice.h>
#include "FlashIAPLimits.h"

using namespace mbed;


/*
 * If defined: sensors powered parasitically.
 */
//#define PARASITE_POWER

/*
 * If defined: only one sensor device is allowed to be connected to the bus.
 * The library may be configured with 1-wire search activity disabled to
 * reduce its footprint.
 */
#define SINGLE_SENSOR

/*
 * The parameter specific type of power provisioning for parasitically powered
 * sensors:
 * - Not defined: power provided by bus pin.
 * - Defined: power provided by a switching transistor and controlled by the
 *   pin number specified by the parameter.
 */
//#define PWR_CTRL_PIN    9

/*
 * If defined: set permanent, common resolution for all sensors on the bus.
 * Resolution may vary from 9 to 12 bits.
 */
//#define COMMON_RES      (DSTherm::RES_12_BIT)

#if !defined(SINGLE_SENSOR) && !CONFIG_SEARCH_ENABLED
# error "CONFIG_SEARCH_ENABLED is required for non single sensor setup"
#endif

#if defined(PWR_CTRL_PIN) && !CONFIG_PWR_CTRL_ENABLED
# error "CONFIG_PWR_CTRL_ENABLED is required if PWR_CTRL_PIN is configured"
#endif

#if (CONFIG_MAX_SEARCH_FILTERS > 0)
static_assert(CONFIG_MAX_SEARCH_FILTERS >= DSTherm::SUPPORTED_SLAVES_NUM,
    "CONFIG_MAX_SEARCH_FILTERS too small");
#endif

#ifdef PARASITE_POWER
# define PARASITE_POWER_ARG true
#else
# define PARASITE_POWER_ARG false
#endif

static Placeholder<OneWireNg_CurrentPlatform> ow;

/* returns false if not supported */
static bool printId(const OneWireNg::Id& id)
{
    const char *name = DSTherm::getFamilyName(id);

    Serial.print(id[0], HEX);
    for (size_t i = 1; i < sizeof(OneWireNg::Id); i++) {
        Serial.print(':');
        Serial.print(id[i], HEX);
    }
    if (name) {
        Serial.print(" -> ");
        Serial.print(name);
    }
    Serial.println();

    return (name != NULL);
}

static void printScratchpad(const DSTherm::Scratchpad& scrpd)
{
    const uint8_t *scrpd_raw = scrpd.getRaw();

    Serial.print("  Scratchpad:");
    for (size_t i = 0; i < DSTherm::Scratchpad::LENGTH; i++) {
        Serial.print(!i ? ' ' : ':');
        Serial.print(scrpd_raw[i], HEX);
    }

    Serial.print("; Th:");
    Serial.print(scrpd.getTh());

    Serial.print("; Tl:");
    Serial.print(scrpd.getTl());

    Serial.print("; Resolution:");
    Serial.print(9 + (int)(scrpd.getResolution() - DSTherm::RES_9_BIT));

    long temp = scrpd.getTemp();
    Serial.print("; Temp:");
    if (temp < 0) {
        temp = -temp;
        Serial.print('-');
    }
    Serial.print(temp / 1000);
    Serial.print('.');
    Serial.print(temp % 1000);
    Serial.print(" C");

    Serial.println();
}

static long extractTemperture(const DSTherm::Scratchpad& scrpd)
{
  long temp = scrpd.getTemp();

  return temp;
}

void setupTempSensor()
{
#ifdef PWR_CTRL_PIN
    new (&ow) OneWireNg_CurrentPlatform(OW_PIN, PWR_CTRL_PIN, false);
#else
    new (&ow) OneWireNg_CurrentPlatform(OW_PIN, false);
#endif
    DSTherm drv(ow);

    Serial.begin(115200);

#if (CONFIG_MAX_SEARCH_FILTERS > 0)
    drv.filterSupportedSlaves();
#endif

#ifdef COMMON_RES
    /*
     * Set common resolution for all sensors.
     * Th, Tl (high/low alarm triggers) are set to 0.
     */
    drv.writeScratchpadAll(0, 0, COMMON_RES);

    /*
     * The configuration above is stored in volatile sensors scratchpad
     * memory and will be lost after power unplug. Therefore store the
     * configuration permanently in sensors EEPROM.
     */
    drv.copyScratchpadAll(PARASITE_POWER_ARG);
#endif
}

long getTemperature()
{
    DSTherm drv(ow);

    /* convert temperature on all sensors connected... */
    drv.convertTempAll(DSTherm::MAX_CONV_TIME, PARASITE_POWER_ARG);

#ifdef SINGLE_SENSOR
    /* single sensor environment */

    long temperature = -1;

    /*
     * Scratchpad placeholder is static to allow reuse of the associated
     * sensor id while reissuing readScratchpadSingle() calls.
     * Note, due to its storage class the placeholder is zero initialized.
     */
    static Placeholder<DSTherm::Scratchpad> scrpd;

    OneWireNg::ErrorCode ec = drv.readScratchpadSingle(scrpd);
    if (ec == OneWireNg::EC_SUCCESS) {
        //printId(scrpd->getId());
        //printScratchpad(scrpd);
        temperature = extractTemperture(scrpd);

    } else if (ec == OneWireNg::EC_CRC_ERROR)
        Serial.println("  CRC error.");
#endif

    return temperature;
}

void setupPHSensor()
{
  pinMode(13,OUTPUT);  
  Serial.begin(9600);  
  Serial.println("Ready");    //Test the serial monitor
}

float getPHValues()
{
  unsigned long int avgValue;  //Store the average value of the sensor feedback
  float b;
  int buf[10];
  int temp;

  analogReadResolution(10);
  for(int i=0;i<10;i++)       //Get 10 sample value from the sensor for smooth the value
  { 
    buf[i]=analogRead(PHSensorPin);
    delay(10);
  }
  for(int i=0;i<9;i++)        //sort the analog from small to large
  {
    for(int j=i+1;j<10;j++)
    {
      if(buf[i]>buf[j])
      {
        temp=buf[i];
        buf[i]=buf[j];
        buf[j]=temp;
      }
    }
  }
  avgValue=0;
  for(int i=2;i<8;i++)                      //take the average value of 6 center sample
    avgValue+=buf[i];
  float avgAnal = (float)avgValue / 6.0; 
  // the voltage increases when Ph decreses
  float avgVol = (float)(avgAnal)*5.0/1023.0;
  //phValue = -5.70 * phValue + 21.34;
  float phValue= 16.0 - 18.75 * avgVol / 5.0;                      //convert the millivolt into pH value
  //float phValue= avgVol / 5.0; 

  return phValue;
  // Serial.print("    pH:");  
  // Serial.print(phValue,2);
  // Serial.println(" ");
  // digitalWrite(13, HIGH);       
  // delay(800);
  // digitalWrite(13, LOW); 
 
}

void setupRadioactiveSensor()
{
  geiger.start();
}

float getRadioactiveValues()
{
  float CPM = geiger.getCPM();
  float nSvh = geiger.getnSvh();
  float uSvh = geiger.getuSvh();

  return nSvh;
}

void setupPump()
{
  s.attach(pump_pin);  
}

void fillContainer(int ptime)
{
  Serial.println("Start fill!");
  s.write(0);
  delay(ptime);
  s.write(90);
  delay(1000);
  Serial.println("Finish fill!");
}

void drainContainer(int ptime)
{
  Serial.println("Start drain!");
  s.write(180);
  delay(ptime);
  s.write(90);
  delay(1000);
   Serial.println("Finish drain!");
}

void setupGPS()
{
   Serial1.begin(9600);
}

float getGPSvalues()
{
   // gps
  bool newData = false;
  unsigned long chars;
  unsigned short sentences, failed;

  // For one second we parse GPS data and report some key values
  for (unsigned long start = millis(); millis() - start < 1000;)
  {
    while (Serial1.available())
    {
      char c = Serial1.read();
      //Serial.write(c); // uncomment this line if you want to see the GPS data flowing
      if (gps.encode(c)) // Did a new valid sentence come in?
        Serial.print("DATA FOUND -----------------------------------------------------------------------------------------");
        newData = true;
    }
  }

  if (newData)
  {
    //float flat, flon;
    unsigned long age;
    gps.f_get_position(&flat, &flon, &age);
    // Serial.print("LAT=");
    // Serial.print(flat == TinyGPS::GPS_INVALID_F_ANGLE ? 0.0 : flat, 6);
    // Serial.print(" LON=");
    // Serial.print(flon == TinyGPS::GPS_INVALID_F_ANGLE ? 0.0 : flon, 6);
    // Serial.print(" SAT=");
    // Serial.print(gps.satellites() == TinyGPS::GPS_INVALID_SATELLITES ? 0 : gps.satellites());
    // Serial.print(" PREC=");
    // Serial.print(gps.hdop() == TinyGPS::GPS_INVALID_HDOP ? 0 : gps.hdop());
  }
  
  gps.stats(&chars, &sentences, &failed);
  Serial.print(" CHARS=");
  Serial.print(chars);
  Serial.print(" SENTENCES=");
  Serial.print(sentences);
  Serial.print(" CSUM ERR=");
  Serial.println(failed);
  if (chars == 0)
    Serial.println("** No characters received from GPS: check wiring **");

  return 0.0;
}

void setupSpectral()
{
  Wire.begin();
  SpectralSensor.begin();
}

float getSpectralValues()
{
  SpectralSensor.takeMeasurements();
  float R = SpectralSensor.getCalibratedR();
  float S = SpectralSensor.getCalibratedS();
  float T = SpectralSensor.getCalibratedT();
  float U = SpectralSensor.getCalibratedU();
  float V = SpectralSensor.getCalibratedV();
  float W = SpectralSensor.getCalibratedW();
  float F = SpectralSensor.getTemperatureF();

  return R;
}

int analize(float temperature, float phValue, float nSvh, float R)
{

  int res = 0;
  if ((temperature > TEMP_HIGH) || (temperature < TEMP_LOW)) res = 1;
  if ((phValue > PH_HIGH) || (phValue < PH_LOW)) res = 1;
  if (nSvh > GEIGER_HIGH) res = 1;
  // if something with R

  return res;
}

void setupRedLED()
{
  pinMode(RED_LED_PIN, OUTPUT);
}

void redLEDOn()
{
  digitalWrite(RED_LED_PIN, HIGH);
}

void redLEDOff()
{
  digitalWrite(RED_LED_PIN, LOW);
}

void setupGreenLED()
{
  pinMode(GREEN_LED_PIN, OUTPUT);
}

void greenLEDOn()
{
  digitalWrite(GREEN_LED_PIN, HIGH);
}

void greenLEDOff()
{
  digitalWrite(GREEN_LED_PIN, LOW);
}

void saveData(String data)
{

  // Get limits of the the internal flash of the microcontroller
  auto [flashSize, startAddress, iapSize] = getFlashIAPLimits();

  // Serial.print("Flash Size: ");
  // Serial.print(flashSize / 1024.0 / 1024.0);
  // Serial.println(" MB");
  // Serial.print("FlashIAP Start Address: 0x");
  // Serial.println(startAddress, HEX);
  // Serial.print("FlashIAP Size: ");
  // Serial.print(iapSize / 1024.0 / 1024.0);
  // Serial.println(" MB");

  // Create a block device on the available space of the flash
  FlashIAPBlockDevice blockDevice(startAddress, iapSize);

  // Initialize the Flash IAP block device and print the memory layout
  blockDevice.init();
  
  const auto eraseBlockSize = blockDevice.get_erase_size();
  const auto programBlockSize = blockDevice.get_program_size();

  Serial.println("Block device size: " + String((unsigned int) blockDevice.size() / 1024.0 / 1024.0) + " MB");
  Serial.println("Readable block size: " + String((unsigned int) blockDevice.get_read_size())  + " bytes");
  Serial.println("Programmable block size: " + String((unsigned int) programBlockSize) + " bytes");
  Serial.println("Erasable block size: " + String((unsigned int) eraseBlockSize / 1024) + " KB");
    
  // Calculate the amount of bytes needed to store the message
  // This has to be a multiple of the program block size
  const auto messageSize = data.length() + 1; // C String takes 1 byte for NULL termination
  const unsigned int requiredEraseBlocks = ceil(messageSize / (float)  eraseBlockSize);
  const unsigned int requiredProgramBlocks = ceil(messageSize / (float)  programBlockSize);
  const auto dataSize = requiredProgramBlocks * programBlockSize;  
  char buffer[dataSize] {};

  // Read back what was stored at previous execution
  Serial.println("Reading previously stored data...");
  blockDevice.read(buffer, 0, dataSize);
  Serial.println(buffer);

  // Erase a block starting at the offset 0 relative
  // to the block device start address
  blockDevice.erase(0, requiredEraseBlocks * eraseBlockSize);

  // Write an updated message to the first block
  Serial.println("Writing data...");
  Serial.println(data);  
  blockDevice.program(data.c_str(), 0, dataSize);

  // Deinitialize the device
  blockDevice.deinit();
  Serial.println("Done.");

}


void setup()
{
  setupTempSensor();
  setupPHSensor();
  setupRadioactiveSensor();
  setupPump();
  setupGPS();
  setupSpectral();
  last_time = millis();
  setupRedLED();
  digitalWrite(RED_LED_PIN, LOW);
  // setupGreenLED();
}

void loop()
{

  getGPSvalues();
  long int curr_time = millis();

  // check if we moved enough since the last check, or if the 24 hours passed since the last time
  if ((fabs(last_flat - flat) >= dlat) || (fabs(last_flon - flon) >= dlon) || ((curr_time - last_time) > dtime)) // 0.01 diff is 1 km
  {
    last_flat = flat;
    last_flon = flon;
    last_time = curr_time;

    Serial.print("LAT=");
    Serial.print(flat == TinyGPS::GPS_INVALID_F_ANGLE ? 0.0 : flat, 6);
    Serial.print(" LON=");
    Serial.print(flon == TinyGPS::GPS_INVALID_F_ANGLE ? 0.0 : flon, 6);
    Serial.println(" "); 

    fillContainer(pump_time);

    // allow sensors to get to the correct values
    delay(wait_time);

    long temperature = getTemperature();
    Serial.print("Temp:");
    if (temperature < 0) {
        temperature = -temperature;
        Serial.print('-');
    }
    float temp = temperature / 1000 + float(temperature % 1000) / 1000.0;
    Serial.print(temp);
    // Serial.print(temperature / 1000);
    // Serial.print('.');
    // Serial.print(temperature % 1000);
    Serial.print(" C");

    Serial.println();

    float phValue = getPHValues();
    Serial.print("pH:");  
    Serial.print(phValue,2);
    Serial.println(" ");
    digitalWrite(13, HIGH);       
    delay(800);
    digitalWrite(13, LOW); 

    float nSvh = getRadioactiveValues();
    Serial.print("nSvh:");  
    Serial.print(nSvh,2);
    Serial.println(" ");

    float R = getSpectralValues();
    Serial.print("Spectral R: ");
    Serial.print(R, 2);
    Serial.println(" ");

    // chek if values are within the desired range
    int res = analize(temp, phValue, nSvh, R);
    if (res)
    {
      Serial.print("Water quality analysis: FAIL");
      //greenLEDOff();
      redLEDOn();
    }
    else
    {
      Serial.print("Water quality analysis: PASS");
      redLEDOff();
      // greenLEDOn();
    }
    Serial.println(" ");

    //long flat1 = int(flat);
    //long flat2 = flat % 1;


    String data = String(curr_time) + "," + String(flat) + "," + String(flon) + "," + String(temp) + "," + String(phValue) + "," + String(nSvh) + "," + String(R) + "," + String(res);
    saveData(data);

    drainContainer(pump_time);
    Serial.println("----------");

  }
}

