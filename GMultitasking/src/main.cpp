#include <Arduino.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <WiFiMulti.h>
#include "SX1272.h"
#include "ThingSpeak.h"
#include <CircularBuffer.h>
#include <ESP32Time.h>
#include <time.h>

#define INTERVAL 11

#include "soc/timer_group_struct.h"
#include "soc/timer_group_reg.h"

#define TS_ENABLE_SSL

#ifdef ARDUINO
// IMPORTANT when using an Arduino only. For a Raspberry-based gateway the distribution uses a radio.makefile file
///////////////////////////////////////////////////////////////////////////////////////////////////////////
// please uncomment only 1 choice
//
// uncomment if your radio is an HopeRF RFM92W, HopeRF RFM95W, Modtronix inAir9B, NiceRF1276
// or you known from the circuit diagram that output use the PABOOST line instead of the RFO line
#define PABOOST
///////////////////////////////////////////////////////////////////////////////////////////////////////////
#endif

// IMPORTANT
///////////////////////////////////////////////////////////////////////////////////////////////////////////
// please uncomment only 1 choice
//#define BAND868
#define BAND900
//#define BAND433
///////////////////////////////////////////////////////////////////////////////////////////////////////////
// For a Raspberry-based gateway the distribution uses a radio.makefile file that can define MAX_DBM
//
#ifndef MAX_DBM
#define MAX_DBM 14
#endif

#ifndef LORA_PREAMBLE_LENGTH
#define LORA_PREAMBLE_LENGTH 8
#endif

#ifndef ARDUINO
#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#ifdef GETOPT_ISSUE
int getopt(int argc, char *const argv[], const char *optstring);
extern char *optarg;
extern int optind, opterr, optopt;
#endif
#include <getopt.h>
#include <termios.h>
#include <signal.h>
#include <sys/time.h>
#include <time.h>
#include <math.h>
#include <string.h>
#endif

#ifdef ARDUINO
// and SPI library on Arduino platforms
#include <SPI.h>

#define PRINTLN Serial.println("")
#define PRINT_CSTSTR(fmt, param) Serial.print(F(param))
#define PRINT_STR(fmt, param) Serial.print(param)
#define PRINT_VALUE(fmt, param) Serial.print(param)
#define PRINT_HEX(fmt, param) Serial.print(param, HEX)
#define FLUSHOUTPUT Serial.flush();
#else
#define PRINTLN printf("\n")
#define PRINT_CSTSTR(fmt, param) printf(fmt, param)
#define PRINT_STR(fmt, param) PRINT_CSTSTR(fmt, param)
#define PRINT_VALUE(fmt, param) PRINT_CSTSTR(fmt, param)
#define PRINT_HEX(fmt, param) PRINT_VALUE(fmt, param)
#define FLUSHOUTPUT fflush(stdout);
#endif

#ifdef DEBUG
#define DEBUGLN PRINTLN
#define DEBUG_CSTSTR(fmt, param) PRINT_CSTSTR(fmt, param)
#define DEBUG_STR(fmt, param) PRINT_CSTSTR(fmt, param)
#define DEBUG_VALUE(fmt, param) PRINT_VALUE(fmt, param)
#else
#define DEBUGLN
#define DEBUG_CSTSTR(fmt, param)
#define DEBUG_STR(fmt, param)
#define DEBUG_VALUE(fmt, param)
#endif

///////////////////////////////////////////////////////////////////////////////////////////////////////////
// FOR DOWNLINK FEATURES
//
#if not defined ARDUINO && defined DOWNLINK
#define MAX_DOWNLINK_ENTRY 100

#include "rapidjson/reader.h"
#include "rapidjson/document.h"
#include "rapidjson/writer.h"
#include <iostream>

using namespace rapidjson;
using namespace std;

char *json_entry[MAX_DOWNLINK_ENTRY];

size_t json_entry_size = 100;
FILE *fp;
int dl_line_index;
ssize_t dl_line_size;
int dl_total_line;
bool hasDownlinkEntry = true; //false
bool enableDownlinkCheck = true; //false
bool optNDL = false;

unsigned long lastDownlinkCheckTime = 0;
// we set to 5s after the gw receives a lora packet
// to give some time for the post-processing stage to generate a downlink.txt file if any
unsigned long interDownlinkCheckTime = 5000L;
unsigned long lastDownlinkSendTime = 0;
// 20s between 2 downlink transmissions when there are queued requests
unsigned long interDownlinkSendTime = 20000L;

//#define INCLUDE_MIC_IN_DOWNLINK

int xtoi(const char *hexstring);
#endif
///////////////////////////////////////////////////////////////////////////////////////////////////////////
//#define SHOW_FREEMEMORY
//#define GW_RELAY
//#define RECEIVE_ALL
//#define CAD_TEST
//#define LORA_LAS
//#define WINPUT
//#define ENABLE_REMOTE

#ifdef BAND868
#define MAX_NB_CHANNEL 15
#define STARTING_CHANNEL 4
#define ENDING_CHANNEL 18
#ifdef SENEGAL_REGULATION
uint8_t loraChannelIndex = 0;
#else
uint8_t loraChannelIndex = 6;
#endif
uint32_t loraChannelArray[MAX_NB_CHANNEL] = {CH_04_868, CH_05_868, CH_06_868, CH_07_868, CH_08_868, CH_09_868,
                                             CH_10_868, CH_11_868, CH_12_868, CH_13_868, CH_14_868, CH_15_868, CH_16_868, CH_17_868, CH_18_868};

#elif defined BAND900
#define MAX_NB_CHANNEL 13
#define STARTING_CHANNEL 0
#define ENDING_CHANNEL 12
uint8_t loraChannelIndex = 12;
uint32_t loraChannelArray[MAX_NB_CHANNEL] = {CH_00_900, CH_01_900, CH_02_900, CH_03_900, CH_04_900, CH_05_900, CH_06_900, CH_07_900, CH_08_900,
                                             CH_09_900, CH_10_900, CH_11_900, CH_12_900};
#elif defined BAND433
#define MAX_NB_CHANNEL 4
#define STARTING_CHANNEL 0
#define ENDING_CHANNEL 3
uint8_t loraChannelIndex = 0;
uint32_t loraChannelArray[MAX_NB_CHANNEL] = {CH_00_433, CH_01_433, CH_02_433, CH_03_433};
#endif

// use the dynamic ACK feature of our modified SX1272 lib
#define GW_AUTO_ACK

///////////////////////////////////////////////////////////////////
// DEFAULT LORA MODE
#define LORAMODE 1
// the special mode to test BW=125MHz, CR=4/5, SF=12
// on the 868.1MHz channel for BAND868, 923.2MHz for BAND900 and 433.175 for BAND433
//#define LORAMODE 11
///////////////////////////////////////////////////////////////////

///////////////////////////////////////////////////////////////////
// GATEWAY HAS ADDRESS 1
#define LORA_ADDR 1
///////////////////////////////////////////////////////////////////

// to unlock remote configuration feature
#define UNLOCK_PIN 1234
// will use 0xFF0xFE to prefix data received from LoRa, so that post-processing stage can differenciate
// data received from radio
#define WITH_DATA_PREFIX

#ifdef WITH_DATA_PREFIX
#define DATA_PREFIX_0 0xFF
#define DATA_PREFIX_1 0xFE
#endif

#ifdef LORA_LAS
#include "LoRaActivitySharing.h"
// acting as the LR-BS
LASBase loraLAS = LASBase();
#endif

///////////////////////////////////////////////////////////////////
// CONFIGURATION VARIABLES
//
#ifndef ARDUINO
char keyPressBuff[30];
uint8_t keyIndex = 0;
int ch;
#endif

// be careful, max command length is 60 characters
#define MAX_CMD_LENGTH 100

char cmd[MAX_CMD_LENGTH] = "****************";
int msg_sn = 0;

// number of retries to unlock remote configuration feature
uint8_t unlocked_try = 3;
boolean unlocked = false;
boolean receivedFromSerial = false;
boolean receivedFromLoRa = false;
boolean withAck = false;

bool radioON = false;
bool RSSIonSend = true;

uint8_t loraMode = LORAMODE;

uint32_t loraChannel = loraChannelArray[loraChannelIndex];
#if defined PABOOST || defined RADIO_RFM92_95 || defined RADIO_INAIR9B || defined RADIO_20DBM
// HopeRF 92W/95W and inAir9B need the PA_BOOST
// so 'x' set the PA_BOOST but then limit the power to +14dBm
char loraPower = 'x';
#else
// other radio board such as Libelium LoRa or inAir9 do not need the PA_BOOST
// so 'M' set the output power to 15 to get +14dBm
char loraPower = 'M';
#endif

uint8_t loraAddr = LORA_ADDR;

int status_counter = 0;
unsigned long startDoCad, endDoCad;
bool extendedIFS = true;
uint8_t SIFS_cad_number;
// set to 0 to disable carrier sense based on CAD
uint8_t send_cad_number = 3;
uint8_t SIFS_value[11] = {0, 183, 94, 44, 47, 23, 24, 12, 12, 7, 4};
uint8_t CAD_value[11] = {0, 62, 31, 16, 16, 8, 9, 5, 3, 1, 1};

bool optAESgw = false;
uint16_t optBW = 0;
uint8_t optSF = 0;
uint8_t optCR = 0;
uint8_t optCH = 0;
bool optRAW = false;
double optFQ = -1.0;
uint8_t optSW = 0x12;
bool optHEX = false;

boolean lorawan = false;
///////////////////////////////////////////////////////////////////

#if defined ARDUINO && defined SHOW_FREEMEMORY && not defined __MK20DX256__ && not defined __MKL26Z64__ && not defined __SAMD21G18A__ && not defined _VARIANT_ARDUINO_DUE_X_
int freeMemory()
{
  extern int __heap_start, *__brkval;
  int v;
  return (int)&v - (__brkval == 0 ? (int)&__heap_start : (int)__brkval);
}
#endif

long getCmdValue(int &i, char *strBuff = NULL)
{

  char seqStr[7] = "******";

  int j = 0;
  // character '#' will indicate end of cmd value
  while ((char)cmd[i] != '#' && (i < strlen(cmd)) && j < strlen(seqStr))
  {
    seqStr[j] = (char)cmd[i];
    i++;
    j++;
  }

  // put the null character at the end
  seqStr[j] = '\0';

  if (strBuff)
  {
    strcpy(strBuff, seqStr);
  }
  else
    return (atol(seqStr));
}

void startConfig()
{

  int e;

  // has customized LoRa settings
  if (optBW != 0 || optCR != 0 || optSF != 0)
  {

    e = sx1272.setCR(optCR - 4);
    PRINT_CSTSTR("%s", "^$LoRa CR ");
    PRINT_VALUE("%d", optCR);
    PRINT_CSTSTR("%s", ": state ");
    PRINT_VALUE("%d", e);
    PRINTLN;

    e = sx1272.setSF(optSF);
    PRINT_CSTSTR("%s", "^$LoRa SF ");
    PRINT_VALUE("%d", optSF);
    PRINT_CSTSTR("%s", ": state ");
    PRINT_VALUE("%d", e);
    PRINTLN;

    e = sx1272.setBW((optBW == 125) ? BW_125 : ((optBW == 250) ? BW_250 : BW_500));
    PRINT_CSTSTR("%s", "^$LoRa BW ");
    PRINT_VALUE("%d", optBW);
    PRINT_CSTSTR("%s", ": state ");
    PRINT_VALUE("%d", e);
    PRINTLN;

    // indicate that we have a custom setting
    // loraMode=0;

    if (optSF < 10)
      SIFS_cad_number = 6;
    else
      SIFS_cad_number = 3;
  }
  else
  {

    // Set transmission mode and print the result
    PRINT_CSTSTR("%s", "^$LoRa mode ");
    PRINT_VALUE("%d", loraMode);
    PRINTLN;

    e = sx1272.setMode(loraMode);
    PRINT_CSTSTR("%s", "^$Setting mode: state ");
    PRINT_VALUE("%d", e);
    PRINTLN;

#ifdef LORA_LAS
    loraLAS.setSIFS(loraMode);
#endif

    if (loraMode > 7)
      SIFS_cad_number = 6;
    else
      SIFS_cad_number = 3;
  }

  // Select frequency channel

  // LoRaWAN
  if (loraMode == 11)
  {
    PRINT_CSTSTR("%s", "^$Configuring for LoRaWAN\n");
    // if we start with mode 11, then switch to 868.1MHz for LoRaWAN test
    // Note: if you change to mode 11 later using command /@M11# for instance, you have to use /@C18# to change to the correct channel

    if (optFQ < 0.0)
    {
#ifdef BAND868
      e = sx1272.setChannel(CH_18_868);
      optFQ = 868.1;
      PRINT_CSTSTR("%s", "^$Set frequency to 868.1MHz: state ");
#elif defined BAND900
      // hardcoded with the first LoRaWAN frequency
      optFQ = 923.2;
      loraChannel = optFQ * 1000000.0 * RH_LORA_FCONVERT;
      e = sx1272.setChannel(loraChannel);
      PRINT_CSTSTR("%s", "^$Set frequency to 923.2MHz: state ");
#elif defined BAND433
      // hardcoded with the first LoRaWAN frequency
      optFQ = 433.175;
      loraChannel = optFQ * 1000000.0 * RH_LORA_FCONVERT;
      e = sx1272.setChannel(loraChannel);
      PRINT_CSTSTR("%s", "^$Set frequency to 433.175MHz: state ");
#endif
    }
    else
    {
      PRINT_CSTSTR("%s", "^$Frequency ");
      PRINT_VALUE("%f", optFQ);
      PRINT_CSTSTR("%s", ": state ");
    }

    // set raw mode for LoRaWAN
    // overriding existing configuration
    optRAW = true;

    // set sync word for LoRaWAN
    optSW = 0x34;
  }
  else
  {
    e = sx1272.setChannel(loraChannel);

    if (optFQ > 0.0)
    {
      PRINT_CSTSTR("%s", "^$Frequency ");
      PRINT_VALUE("%f", optFQ);
      PRINT_CSTSTR("%s", ": state ");
    }
    else
    {
#ifdef BAND868
      if (loraChannelIndex > 5)
      {
        PRINT_CSTSTR("%s", "^$Channel CH_1");
        PRINT_VALUE("%d", loraChannelIndex - 6);
      }
      else
      {
        PRINT_CSTSTR("%s", "^$Channel CH_0");
        PRINT_VALUE("%d", loraChannelIndex + STARTING_CHANNEL);
      }
      PRINT_CSTSTR("%s", "_868: state ");
#elif defined BAND900
      PRINT_CSTSTR("%s", "^$Channel CH_");
      PRINT_VALUE("%d", loraChannelIndex);
      PRINT_CSTSTR("%s", "_900: state ");
#elif defined BAND433
      // e = sx1272.setChannel(0x6C4000);
      PRINT_CSTSTR("%s", "^$Channel CH_");
      PRINT_VALUE("%d", loraChannelIndex);
      PRINT_CSTSTR("%s", "_433: state ");
#endif
      optFQ = loraChannel / (1000000.0 * RH_LORA_FCONVERT);
    }
  }
  PRINT_VALUE("%d", e);
  PRINTLN;

  // Select amplifier line; PABOOST or RFO
#ifdef PABOOST
  sx1272._needPABOOST = true;
  // previous way for setting output power
  // loraPower='x';
  PRINT_CSTSTR("%s", "^$Use PA_BOOST amplifier line");
  PRINTLN;
#else
  // previous way for setting output power
  // loraPower='M';
#endif

  // Select output power in dBm
  e = sx1272.setPowerDBM((uint8_t)MAX_DBM);

  PRINT_CSTSTR("%s", "^$Set LoRa power dBm to ");
  PRINT_VALUE("%d", (uint8_t)MAX_DBM);
  PRINTLN;

  PRINT_CSTSTR("%s", "^$Power: state ");
  PRINT_VALUE("%d", e);
  PRINTLN;

  // get preamble length
  e = sx1272.getPreambleLength();
  PRINT_CSTSTR("%s", "^$Get Preamble Length: state ");
  PRINT_VALUE("%d", e);
  PRINTLN;
  PRINT_CSTSTR("%s", "^$Preamble Length: ");
  PRINT_VALUE("%d", sx1272._preamblelength);
  PRINTLN;

  if (sx1272._preamblelength != LORA_PREAMBLE_LENGTH)
  {
    PRINT_CSTSTR("%s", "^$Bad Preamble Length: set back to default\n");
    sx1272.setPreambleLength(LORA_PREAMBLE_LENGTH);
    e = sx1272.getPreambleLength();
    PRINT_CSTSTR("%s", "^$Get Preamble Length: state ");
    PRINT_VALUE("%d", e);
    PRINTLN;
    PRINT_CSTSTR("%s", "^$Preamble Length: ");
    PRINT_VALUE("%d", sx1272._preamblelength);
    PRINTLN;
  }

  // Set the node address and print the result
  // e = sx1272.setNodeAddress(loraAddr);
  sx1272._nodeAddress = loraAddr;
  e = 0;
  PRINT_CSTSTR("%s", "^$LoRa addr ");
  PRINT_VALUE("%d", loraAddr);
  PRINT_CSTSTR("%s", ": state ");
  PRINT_VALUE("%d", e);
  PRINTLN;

  if (optAESgw)
    PRINT_CSTSTR("%s", "^$Handle AES encrypted data\n");

  if (optRAW)
  {
    PRINT_CSTSTR("%s", "^$Raw format, not assuming any header in reception\n");
    // when operating n raw format, the SX1272 library do not decode the packet header but will pass all the payload to stdout
    // note that in this case, the gateway may process packet that are not addressed explicitly to it as the dst field is not checked at all
    // this would be similar to a promiscuous sniffer, but most of real LoRa gateway works this way
    sx1272._rawFormat = true;
  }

  // Print a success message
  PRINT_CSTSTR("%s", "^$SX1272/76 configured ");
  PRINT_CSTSTR("%s", "as LR-BS. Waiting RF input for transparent RF-serial bridge\n");
#if defined ARDUINO && defined GW_RELAY
  PRINT_CSTSTR("%s", "^$Act as a simple relay gateway\n");
#endif
}

#define WIFI_SSID "Na_Jaemin"
#define WIFI_PASSWORD "nanadevy"

// #define WIFI_SSID "IOT"
// #define WIFI_PASSWORD "iot12345"

WiFiMulti wifiMulti;
WiFiClient client;
// unsigned long myChannelNumber = 1846927;
// const char *myWriteAPIKey = "J00GWMB1EZILVTAF";

unsigned long myChannelNumber = 1949242;
const char *myWriteAPIKey = "SCWXRF52TE9MXRZ3";

ESP32Time rtc(3600);

// ---------------------------------------------------------------------------


namespace data2 {
  typedef struct {
    float TC;
    float HU;
    float DIS;
  } record2;

  void print(record2 r) {
    Serial.print(r.TC);
    Serial.print("  ");
    Serial.print(r.HU);
    Serial.print("  ");
    Serial.print(r.DIS);
    Serial.print("  ");
  }
}

CircularBuffer<data2::record2, 10> struct2s;
// ---------------------------------------CIRCULAR BUFFER------------------------------------------------

#define BUFFER_SIZE 5 // Ukuran buffer circular
char* buffer[BUFFER_SIZE]; // Buffer circular
int front = 0; // Indeks depan buffer
int rear = 0; // Indeks belakang buffer

// ----------------------------------DEF CIRCULAR BUFFER END-------------------------------------------------

int count1;
int count2;
int count3;

void task1(void *pvParameters);
void task2(void *pvParameters);
void task3(void *pvParameters);
// void task3(void *pvParameters);
// void task4(void *pvParameters);
  

void setup() {
  int e;
  #ifdef ARDUINO
  delay(3000);
  randomSeed(analogRead(14));

  // Open serial communications and wait for port to open:
  #ifdef __SAMD21G18A__
  SerialUSB.begin(38400);
  #else
  Serial.begin(38400);
  #endif

  //CONNECTED TO WIFI
  wifiMulti.addAP(WIFI_SSID, WIFI_PASSWORD);

  while (wifiMulti.run() != WL_CONNECTED)
  {
    delay(1000);
  }
  Serial.println("Connected");
  Serial.println(WiFi.localIP());

  ThingSpeak.begin(client); // Initialize ThingSpeak
  rtc.setTime(30, 24, 23, 3, 4, 2023); //RTC // 17th Jan 2021 15:24:30 

  //TASK

  xTaskCreate(
    task1,
    "task1",
    10000,
    NULL,
    1,
    NULL
  );

  xTaskCreate(
    task2,
    "task2",
    10000,
    NULL,
    1,
    NULL
  );

  xTaskCreate(
    task3,
    "task3",
    10000,
    NULL,
    1,
    NULL
  );

  vTaskStartScheduler();


  #if defined ARDUINO && defined SHOW_FREEMEMORY && not defined __MK20DX256__ && not defined __MKL26Z64__ && not defined __SAMD21G18A__ && not defined _VARIANT_ARDUINO_DUE_X_
    // Print a start message
    Serial.print(freeMemory());
    Serial.println(F(" bytes of free memory."));
  #endif

  #else
    srand(time(NULL));
  #endif

  // Power ON the module
  e = sx1272.ON();

  PRINT_CSTSTR("%s", "^$**********Power ON: state ");
  PRINT_VALUE("%d", e);
  PRINTLN;

  if (!e)
  {
    radioON = true;
    startConfig();
  }

  e = sx1272.getSyncWord();

  if (!e)
  {
    PRINT_CSTSTR("%s", "^$Default sync word: 0x");
    PRINT_HEX("%X", sx1272._syncWord);
    PRINTLN;
  }

  if (optSW != 0x12)
  {
    e = sx1272.setSyncWord(optSW);

    PRINT_CSTSTR("%s", "^$Set sync word to 0x");
    PRINT_HEX("%X", optSW);
    PRINTLN;
    PRINT_CSTSTR("%s", "^$LoRa sync word: state ");
    PRINT_VALUE("%d", e);
    PRINTLN;
  }

  FLUSHOUTPUT;
  delay(1000);

  #ifdef LORA_LAS
  loraLAS.ON(LAS_ON_WRESET);
  #endif

  #ifdef CAD_TEST
  PRINT_CSTSTR("%s", "Do CAD test\n");
  #endif

  #if not defined ARDUINO && defined DOWNLINK

  lastDownlinkCheckTime = millis();

  #endif

}

// we could use the CarrierSense function added in the SX1272 library, but it is more convenient to duplicate it here
// so that we could easily modify it for testing
//
// in v1.5 the "only once" behavior is implemented for the gateway when it transmit downlink packets
// to avoid blocking the gateway on a busy channel. Therefore from v1.5 the implementation differs from the
// carrier sense function added in the SX1272 library
//
int CarrierSense(bool onlyOnce = false) {

  int e;
  bool carrierSenseRetry = false;

  if (send_cad_number)
  {
    do
    {
      do
      {

        // check for free channel (SIFS/DIFS)
        startDoCad = millis();
        e = sx1272.doCAD(send_cad_number);
        endDoCad = millis();

        PRINT_CSTSTR("%s", "--> CAD duration ");
        PRINT_VALUE("%ld", endDoCad - startDoCad);
        PRINTLN;

        if (!e)
        {
          PRINT_CSTSTR("%s", "OK1\n");

          if (extendedIFS)
          {
            // wait for random number of CAD
            #ifdef ARDUINO
              uint8_t w = random(1, 8);
            #else
              uint8_t w = rand() % 8 + 1;
            #endif

            PRINT_CSTSTR("%s", "--> waiting for ");
            PRINT_VALUE("%d", w);
            PRINT_CSTSTR("%s", " CAD = ");
            PRINT_VALUE("%d", CAD_value[loraMode] * w);
            PRINTLN;

            delay(CAD_value[loraMode] * w);

            // check for free channel (SIFS/DIFS) once again
            startDoCad = millis();
            e = sx1272.doCAD(send_cad_number);
            endDoCad = millis();

            PRINT_CSTSTR("%s", "--> CAD duration ");
            PRINT_VALUE("%ld", endDoCad - startDoCad);
            PRINTLN;

            if (!e)
              PRINT_CSTSTR("%s", "OK2");
            else
              PRINT_CSTSTR("%s", "###2");

            PRINTLN;
          }
        }
        else
        {
          PRINT_CSTSTR("%s", "###1\n");

          // if we have "only once" behavior then exit here to not have retries
          if (onlyOnce)
            return 1;

            // wait for random number of DIFS
            #ifdef ARDUINO
              uint8_t w = random(1, 8);
            #else
              uint8_t w = rand() % 8 + 1;
            #endif

            PRINT_CSTSTR("%s", "--> waiting for ");
            PRINT_VALUE("%d", w);
            PRINT_CSTSTR("%s", " DIFS (DIFS=3SIFS) = ");
            PRINT_VALUE("%d", SIFS_value[loraMode] * 3 * w);
            PRINTLN;

            delay(SIFS_value[loraMode] * 3 * w);

            PRINT_CSTSTR("%s", "--> retry\n");
        }

      } while (e);

      // CAD is OK, but need to check RSSI
      if (RSSIonSend)
      {

        e = sx1272.getRSSI();

        uint8_t rssi_retry_count = 10;

        if (!e)
        {

          PRINT_CSTSTR("%s", "--> RSSI ");
          PRINT_VALUE("%d", sx1272._RSSI);
          PRINTLN;

          while (sx1272._RSSI > -90 && rssi_retry_count)
          {

            delay(1);
            sx1272.getRSSI();
            PRINT_CSTSTR("%s", "--> RSSI ");
            PRINT_VALUE("%d", sx1272._RSSI);
            PRINTLN;
            rssi_retry_count--;
          }
        }
        else
          PRINT_CSTSTR("%s", "--> RSSI error\n");

        if (!rssi_retry_count)
          carrierSenseRetry = true;
        else
          carrierSenseRetry = false;
      }

    } while (carrierSenseRetry);
  }

  return 0;
}

  int i = 0, e;
  long cmdValue;

void loop() {

  int i = 0, e;
  long cmdValue;
  ///////////////////////////////////////////////////////////////////
  // ONLY FOR TESTING CAD
  #ifdef CAD_TEST

    startDoCad = millis();
    e = sx1272.doCAD(SIFS_cad_number);
    endDoCad = millis();

    PRINT_CSTSTR("%s", "--> SIFS duration ");
    PRINT_VALUE("%ld", endDoCad - startDoCad);
    PRINTLN;

    if (!e)
      PRINT_CSTSTR("%s", "OK");
    else
      PRINT_CSTSTR("%s", "###");

    PRINTLN;

    delay(200);

    startDoCad = millis();
    e = sx1272.doCAD(SIFS_cad_number * 3);
    endDoCad = millis();

    PRINT_CSTSTR("%s", "--> DIFS duration ");
    PRINT_VALUE("%ld", endDoCad - startDoCad);
    PRINTLN;

    if (!e)
      PRINT_CSTSTR("%s", "OK");
    else
      PRINT_CSTSTR("%s", "###");

    PRINTLN;

    delay(200);
  #endif
  // ONLY FOR TESTING CAD
  /// END/////////////////////////////////////////////////////////////

  ///////////////////////////////////////////////////////////////////
  // START OF PERIODIC TASKS

  receivedFromSerial = false;
  receivedFromLoRa = false;

  #ifdef LORA_LAS
    // call periodically to be able to detect the start of a new cycle
    loraLAS.checkCycle();
  #endif

  #if defined ARDUINO && not defined GW_RELAY
    // check if we received data from the input serial port
    if (Serial.available())
    {

      i = 0;

      while (Serial.available() && i < 80)
      {
        cmd[i] = Serial.read();
        i++;
        delay(50);
      }

      cmd[i] = '\0';

      PRINT_CSTSTR("%s", "Rcv serial: ");
      PRINT_STR("%s", cmd);
      PRINTLN;

      receivedFromSerial = true;
    }
  #endif

  // handle keyboard input from a UNIX terminal
  // quick & dirty way to provide command to a running gateway
  // for test & debug purposes mainly
  #if not defined ARDUINO && defined WINPUT

    while (unistd::read(0, &ch, 1))
    {

      if (ch == '\n')
      {

        strcpy(cmd, keyPressBuff);
        PRINT_CSTSTR("%s", "Cmd from keyboard: ");
        PRINT_STR("%s", cmd);
        PRINTLN;

        keyIndex = 0;
        receivedFromSerial = true;
      }
      else
      {
        // backspace
        if (ch == 127 || ch == 8)
        {
          keyIndex--;
        }
        else
        {

          keyPressBuff[keyIndex] = (char)ch;
          keyIndex++;
        }
      }

      keyPressBuff[keyIndex] = '\0';

      PRINT_CSTSTR("%s", "keyboard input : ");
      PRINT_STR("%s", keyPressBuff);
      PRINTLN;
    }
  #endif

  
}

void task1(void *pvParameters)
  {
    for(;;){
      // vTaskDelay(2000/portTICK_PERIOD_MS);
      TIMERG0.wdt_wprotect=TIMG_WDT_WKEY_VALUE;
      TIMERG0.wdt_feed=1;
      TIMERG0.wdt_wprotect=0;

      disableCore0WDT();
      
      if (radioON && !receivedFromSerial){

      e = 1;
      #ifndef CAD_TEST

      if (status_counter == 60 || status_counter == 0)
      {
        PRINT_CSTSTR("%s", "^$Low-level gw status ON");
        PRINTLN;
        FLUSHOUTPUT;
        status_counter = 0;
      }

        // check if we received data from the receiving LoRa module
        #ifdef RECEIVE_ALL
            e = sx1272.receiveAll(MAX_TIMEOUT);
        #else
        #ifdef GW_AUTO_ACK

          e = sx1272.receivePacketTimeout(MAX_TIMEOUT);

          status_counter++;
          if (e != 0 && e != 3)
          {
            PRINT_CSTSTR("%s", "^$Receive error ");
            PRINT_VALUE("%d", e);
            PRINTLN;

            if (e == 2)
            {
              // Power OFF the module
              sx1272.OFF();
              radioON = false;
              PRINT_CSTSTR("%s", "^$Resetting radio module");
              PRINTLN;
              e = sx1272.ON();
              PRINT_CSTSTR("%s", "^$Setting power ON: state ");
              PRINT_VALUE("%d", e);
              PRINTLN;

              if (!e)
              {
                radioON = true;
                startConfig();
              }
              // to start over
              status_counter = 0;
              e = 1;
            }
            FLUSHOUTPUT;
          }

          if (!e && sx1272._requestACK_indicator)
          {
            PRINT_CSTSTR("%s", "^$ACK requested by ");
            PRINT_VALUE("%d", sx1272.packet_received.src);
            PRINTLN;
            FLUSHOUTPUT;
          }
        #else
          // OBSOLETE normally we always use GW_AUTO_ACK
          // Receive message
          if (withAck)
            e = sx1272.receivePacketTimeoutACK(MAX_TIMEOUT);
          else
            e = sx1272.receivePacketTimeout(MAX_TIMEOUT);
        #endif
        #endif
    #endif
    ///////////////////////////////////////////////////////////////////

    if (!e)
    {

      int a = 0, b = 0;
      uint8_t tmp_length;

      receivedFromLoRa = true;

      #if not defined ARDUINO && defined DOWNLINK
        // set timer, gw will check for downlink request (downlink.txt) after
        // interDownlinkCheckTime
        lastDownlinkCheckTime = millis();
        // only if we enabled downlink check after packet reception
        if (!optNDL)
          enableDownlinkCheck = true;
      #endif

        ///////////////////////////////////////////////////////////////////
        // for Linux-based gateway only
        // provide reception timestamp

      #if not defined ARDUINO && not defined GW_RELAY
        char time_buffer[30];
        int millisec;
        struct tm *tm_info;
        struct timeval tv;

        gettimeofday(&tv, NULL);

        millisec = lrint(tv.tv_usec / 1000.0); // Round to nearest millisec

        if (millisec >= 1000)
        { // Allow for rounding up to nearest second
          millisec -= 1000;
          tv.tv_sec++;
        }

        tm_info = localtime(&tv.tv_sec);
      #endif

      // tmp_length=sx1272._payloadlength;
      tmp_length = sx1272.getPayloadLength();

      #if not defined GW_RELAY

        sx1272.getSNR();
        sx1272.getRSSIpacket();

        sprintf(cmd, "--- rxlora. dst=%d type=0x%02X src=%d seq=%d",
                sx1272.packet_received.dst,
                sx1272.packet_received.type,
                sx1272.packet_received.src,
                sx1272.packet_received.packnum);

        PRINT_STR("%s", cmd);

        sprintf(cmd, " len=%d SNR=%d RSSIpkt=%d BW=%d CR=4/%d SF=%d\n",
                tmp_length,
                sx1272._SNR,
                sx1272._RSSIpacket,
                (sx1272._bandwidth == BW_125) ? 125 : ((sx1272._bandwidth == BW_250) ? 250 : 500),
                sx1272._codingRate + 4,
                sx1272._spreadingFactor);

        PRINT_STR("%s", cmd);

        // provide a short output for external program to have information about the received packet
        // ^psrc_id,seq,len,SNR,RSSI
        sprintf(cmd, "^p%d,%d,%d,%d,",
                sx1272.packet_received.dst,
                sx1272.packet_received.type,
                sx1272.packet_received.src,
                sx1272.packet_received.packnum);

        PRINT_STR("%s", cmd);

        sprintf(cmd, "%d,%d,%d\n",
                tmp_length,
                sx1272._SNR,
                sx1272._RSSIpacket);

        PRINT_STR("%s", cmd);

        // ^rbw,cr,sf,fq
        sprintf(cmd, "^r%d,%d,%d,%ld\n",
                (sx1272._bandwidth == BW_125) ? 125 : ((sx1272._bandwidth == BW_250) ? 250 : 500),
                sx1272._codingRate + 4,
                sx1272._spreadingFactor,
                (uint32_t)(optFQ * 1000.0));

        PRINT_STR("%s", cmd);
      #endif
      ///////////////////////////////////////////////////////////////////

      #if not defined ARDUINO && not defined GW_RELAY
        strftime(time_buffer, 30, "%Y-%m-%dT%H:%M:%S", tm_info);
        sprintf(cmd, "^t%s.%03d\n", time_buffer, millisec);
        PRINT_STR("%s", cmd);
      #endif

      #ifdef LORA_LAS
        if (loraLAS.isLASMsg(sx1272.packet_received.data))
        {

          // tmp_length=sx1272.packet_received.length-OFFSET_PAYLOADLENGTH;
          // tmp_length=sx1272._payloadlength;
          tmp_length = sx1272.getPayloadLength();

          int v = loraLAS.handleLASMsg(sx1272.packet_received.src,
                                      sx1272.packet_received.data,
                                      tmp_length);

          if (v == DSP_DATA)
          {
            a = LAS_DSP + DATA_HEADER_LEN + 1;
          #ifdef WITH_DATA_PREFIX
                    PRINT_STR("%c", (char)DATA_PREFIX_0);
                    PRINT_STR("%c", (char)DATA_PREFIX_1);
          #endif
          }
          else
            // don't print anything
            a = tmp_length;
        }
        else
          PRINT_CSTSTR("%s", "No LAS header. Write raw data\n");
      #else
        #if defined WITH_DATA_PREFIX && not defined GW_RELAY
          PRINT_STR("%c", (char)DATA_PREFIX_0);
          PRINT_STR("%c", (char)DATA_PREFIX_1);
        #endif
      #endif

      #if defined ARDUINO && defined GW_RELAY

        // here we resend the received data to the next gateway
        //
        // set correct header information
        sx1272._nodeAddress = sx1272.packet_received.src;
        sx1272._packetNumber = sx1272.packet_received.packnum;
        sx1272.setPacketType(sx1272.packet_received.type);

        CarrierSense();

        e = sx1272.sendPacketTimeout(1, sx1272.packet_received.data, tmp_length, 10000);

        PRINT_CSTSTR("%s", "Packet re-sent, state ");
        PRINT_VALUE("%d", e);
        PRINTLN;

        // set back the gateway address
        sx1272._nodeAddress = loraAddr;

      #else
      // print to stdout the content of the packet
      //
      FLUSHOUTPUT;

      for (; a < tmp_length; a++)
      {

        if (optHEX)
        {
          if ((uint8_t)sx1272.packet_received.data[a] < 16)
            PRINT_CSTSTR("%s", "0");
          PRINT_HEX("%X", (uint8_t)sx1272.packet_received.data[a]);
          PRINT_CSTSTR("%s", " ");
        }
        else
          PRINT_STR("%c", (char)sx1272.packet_received.data[a]);

        if (b < MAX_CMD_LENGTH)
        {
          cmd[b] = (char)sx1272.packet_received.data[a];
          b++;
        }
      }

      // strlen(cmd) will be correct as only the payload is copied
      cmd[b] = '\0';
      PRINTLN;
      FLUSHOUTPUT;

      #if not defined ARDUINO && defined WINPUT
        // if we received something, display again the current input
        // that has still not be terminated
        if (keyIndex)
        {
          PRINT_CSTSTR("%s", "keyboard input : ");
          PRINT_STR("%s", keyPressBuff);
          PRINTLN;
        }
      #endif
      #endif
    }
  }

  if (receivedFromSerial || receivedFromLoRa)
  {

    i = 0;

    if (cmd[i] == '/' && cmd[i + 1] == '@')
    {

      PRINT_CSTSTR("%s", "^$Parsing command\n");
      i = 2;

      PRINT_CSTSTR("%s", "^$");
      PRINT_STR("%s", cmd);
      PRINTLN;

      if ((receivedFromLoRa && cmd[i] != 'U' && !unlocked) || !unlocked_try)
      {
        PRINT_CSTSTR("%s", "^$Remote config locked\n");
        // just assign an unknown command
        cmd[i] = '*';
      }

      switch (cmd[i])
      {

      // comment if you want your device to be remotely configured for test purposes mainly
      #ifdef ENABLE_REMOTE
        case 'U':

          if (unlocked_try)
          {
            i++;
            cmdValue = getCmdValue(i);

            if (cmdValue == UNLOCK_PIN)
            {

              unlocked = !unlocked;

              if (unlocked)
                PRINT_CSTSTR("%s", "^$Unlocked\n");
              else
                PRINT_CSTSTR("%s", "^$Locked\n");
            }
            else
              unlocked_try--;

            if (unlocked_try == 0)
              PRINT_CSTSTR("%s", "^$Bad pin\n");
          }
          break;
      // comment if you want your device to be remotely configured for test purposes mainly
      #endif
      case 'S':

        if (cmd[i + 1] == 'F')
        {
          i = i + 2;
          cmdValue = getCmdValue(i);

          if (cmdValue > 5 && cmdValue < 13)
          {
            PRINT_CSTSTR("%s", "^$set SF: ");
            PRINT_VALUE("%d", cmdValue);
            PRINTLN;
            // Set spreading factor
            e = sx1272.setSF(cmdValue);
            PRINT_CSTSTR("%s", "^$set SF: state ");
            PRINT_VALUE("%d", e);
            PRINTLN;
          }
        }
        break;

      case 'M':
        i++;
        cmdValue = getCmdValue(i);
        // cannot set mode greater than 11 (11 being the LoRaWAN test mode)
        if (cmdValue > 11)
          cmdValue = 4;
        // cannot set mode lower than 0
        if (cmdValue < 0)
          cmdValue = 4;
        // set dest addr
        loraMode = cmdValue;

        PRINT_CSTSTR("%s", "^$Set LoRa mode to ");
        PRINT_VALUE("%d", loraMode);
        PRINTLN;
        // Set transmission mode and print the result
        e = sx1272.setMode(loraMode);
        PRINT_CSTSTR("%s", "^$LoRa mode: state ");
        PRINT_VALUE("%d", e);
        PRINTLN;

        #ifdef LORA_LAS
          loraLAS.setSIFS(loraMode);
        #endif
        // get preamble length
        e = sx1272.getPreambleLength();
        PRINT_CSTSTR("%s", "Get Preamble Length: state ");
        PRINT_VALUE("%d", e);
        PRINTLN;
        PRINT_CSTSTR("%s", "Preamble Length: ");
        PRINT_VALUE("%d", sx1272._preamblelength);
        PRINTLN;
        break;

      case 'W':
        i++;
        cmdValue = getCmdValue(i);

        // we expect an HEX format value
        cmdValue = (cmdValue / 10) * 16 + (cmdValue % 10);

        // cannot set sync word greater than 255
        if (cmdValue > 255)
          cmdValue = 0x12;
        // cannot set sync word lower than 0
        if (cmdValue <= 0)
          cmdValue = 0x12;

        PRINT_CSTSTR("%s", "^$Set sync word to 0x");
        PRINT_HEX("%X", cmdValue);
        PRINTLN;

        e = sx1272.setSyncWord(cmdValue);
        PRINT_CSTSTR("%s", "^$LoRa sync word: state ");
        PRINT_VALUE("%d", e);
        PRINTLN;
        break;

      case 'C':

        if (cmd[i + 1] == 'A' && cmd[i + 2] == 'D')
        {

          if (cmd[i + 3] == 'O' && cmd[i + 4] == 'N')
          {
            i = i + 5;
            cmdValue = getCmdValue(i);
            // cannot set send_cad_number greater than 255
            if (cmdValue > 255)
              cmdValue = 255;

            send_cad_number = cmdValue;

            PRINT_CSTSTR("%s", "Set send_cad_number to ");
            PRINT_VALUE("%d", send_cad_number);
            PRINTLN;
            break;
          }

          if (cmd[i + 3] == 'O' && cmd[i + 4] == 'F' && cmd[i + 5] == 'F')
          {
            send_cad_number = 0;
            break;
          }

          startDoCad = millis();
          e = sx1272.doCAD(SIFS_cad_number);
          endDoCad = millis();

          PRINT_CSTSTR("%s", "--> SIFS duration ");
          PRINT_VALUE("%ld", endDoCad - startDoCad);
          PRINTLN;

          if (!e)
            PRINT_CSTSTR("%s", "OK");
          else
            PRINT_CSTSTR("%s", "###");

          PRINTLN;
        }
        else
        {
          i++;
          cmdValue = getCmdValue(i);

          if (cmdValue < STARTING_CHANNEL || cmdValue > ENDING_CHANNEL)
            loraChannelIndex = STARTING_CHANNEL;
          else
            loraChannelIndex = cmdValue;

          loraChannelIndex = loraChannelIndex - STARTING_CHANNEL;
          loraChannel = loraChannelArray[loraChannelIndex];

          optFQ = (double)loraChannel / RH_LORA_FCONVERT / 1000000.0;

          PRINT_CSTSTR("%s", "^$Set LoRa channel to ");
          PRINT_VALUE("%d", cmdValue);
          PRINTLN;

          // Select frequency channel
          e = sx1272.setChannel(loraChannel);
          PRINT_CSTSTR("%s", "^$Setting Channel: state ");
          PRINT_VALUE("%d", e);
          PRINTLN;
        }
        break;

      case 'P':

        if (cmd[i + 1] == 'L' || cmd[i + 1] == 'H' || cmd[i + 1] == 'M' || cmd[i + 1] == 'x' || cmd[i + 1] == 'X')
        {
          loraPower = cmd[i + 1];

          PRINT_CSTSTR("%s", "^$Set LoRa Power to ");
          PRINT_VALUE("%c", loraPower);
          PRINTLN;

          e = sx1272.setPower(loraPower);
          PRINT_CSTSTR("%s", "^$Setting Power: state ");
          PRINT_VALUE("%d", e);
          PRINTLN;
        }
        else
          PRINT_CSTSTR("%s", "Invalid Power. L, H, M, x or X accepted.\n");
        break;

      case 'O':

        if (cmd[i + 1] == 'N')
        {

          PRINT_CSTSTR("%s", "^$Setting LoRa module to ON");

          // Power ON the module
          e = sx1272.ON();
          PRINT_CSTSTR("%s", "^$Setting power ON: state ");
          PRINT_VALUE("%d", e);
          PRINTLN;

          if (!e)
          {
            radioON = true;
            startConfig();
          }

          delay(500);
        }
        else if (cmd[i + 1] == 'F' && cmd[i + 2] == 'F')
        {
          PRINT_CSTSTR("%s", "^$Setting LoRa module to OFF\n");

          // Power OFF the module
          sx1272.OFF();
          radioON = false;
        }
        else
          PRINT_CSTSTR("%s", "Invalid command. ON or OFF accepted.\n");
        break;

      // act as an LR-BS if IS_RCV_GATEWAY or an end-device if IS_SEND_GATEWAY
      case 'L':
        #ifdef LORA_LAS
          if (cmd[i + 1] == 'A' && cmd[i + 2] == 'S')
          {

            if (cmd[i + 3] == 'S')
              loraLAS.showLAS();

            if (cmd[i + 3] == 'R')
            {
              loraLAS.reset();
              loraLAS.showLAS();
            }

            if (cmd[i + 3] == 'O' && cmd[i + 4] == 'N')
              loraLAS.ON(LAS_ON_NORESET);

            if (cmd[i + 3] == 'O' && cmd[i + 4] == 'F' && cmd[i + 5] == 'F')
              loraLAS.OFF();

            // only the base station can sent an INIT restart message
            // sends an init restart
            if (cmd[i + 3] == 'I')
            {
              loraLAS.sendInit(LAS_INIT_RESTART);
            }
          }
        #endif

        if (cmd[i + 1] == 'W')
        {
          lorawan = !lorawan;

          if (lorawan)
          {
            PRINT_CSTSTR("%s", "LORAWAN FORMAT ON (RAW MODE)\n");
            // indicate to SX1272 lib that raw mode at reception is required
            sx1272._rawFormat = true;
            PRINT_CSTSTR("%s", "SYNC WORD 0x34\n");
            e = sx1272.setSyncWord(0x34);
            PRINT_CSTSTR("%s", "state ");
            PRINT_VALUE("%d", e);
            PRINTLN;

            PRINT_CSTSTR("%s", "CONSIDER SETTING SF12BW125 AND SYNC WORD 0x34 BY USING MODE 11: /@M11#\n");
            PRINT_CSTSTR("%s", "CONSIDER SETTING CHANNEL to 18 for EU868 (868.1MHz): /@C18#\n");
            PRINT_CSTSTR("%s", "OR USING /@F868100#\n");
          }
          else
          {
            PRINT_CSTSTR("%s", "LORAWAN FORMAT OFF (RAW MODE DISABLED)\n");
            sx1272._rawFormat = false;
            PRINT_CSTSTR("%s", "SYNC WORD 0x12\n");
            e = sx1272.setSyncWord(0x12);
            PRINT_CSTSTR("%s", "state ");
            PRINT_VALUE("%d", e);
            PRINTLN;
          }
        }
        break;

      // set the frequency
      // "F433175#"
      case 'F':
        i++;
        cmdValue = getCmdValue(i);

        loraChannel = cmdValue * 1000.0 * RH_LORA_FCONVERT;
        optFQ = cmdValue / 1000.0;

        // Select frequency channel
        e = sx1272.setChannel(loraChannel);

        PRINT_CSTSTR("%s", "Set frequency to ");
        PRINT_VALUE("%d", cmdValue);
        PRINTLN;

        PRINT_CSTSTR("%s", "state ");
        PRINT_VALUE("%d", e);
        PRINTLN;
        break;

      // toggle raw mode
      // "RAW"
      case 'R':

        if (cmd[i + 1] == 'A' && cmd[i + 2] == 'W')
        {

          sx1272._rawFormat = !sx1272._rawFormat;

          if (sx1272._rawFormat)
            PRINT_CSTSTR("%s", "RAW MODE ON\n");
          else
            PRINT_CSTSTR("%s", "RAW MODE OFF\n");
        }
        break;

      // toggle hex output mode
      // "HEX"
      case 'H':

        if (cmd[i + 1] == 'E' && cmd[i + 2] == 'X')
        {

          optHEX = !optHEX;

          if (optHEX)
            PRINT_CSTSTR("%s", "HEX MODE ON\n");
          else
            PRINT_CSTSTR("%s", "HEX MODE OFF\n");
        }
        break;

      default:

        PRINT_CSTSTR("%s", "Unrecognized cmd\n");
        break;
      }
      FLUSHOUTPUT;
    }
  } 
  // ********************************* end of "if (receivedFromSerial || receivedFromLoRa)" *********************************************

  //--------------------------------------push buffer-------------------------------//

  char* item = (char*)sx1272.packet_received.data;

  buffer[rear] = (char*)malloc(10*sizeof(char *)); // Alokasikan memori untuk item
  sprintf(buffer[rear], item); // Isi item dengan data
  rear = (rear + 1) % BUFFER_SIZE;

   
  // vTaskDelay(2000/portTICK_PERIOD_MS);

  }
  
}

// void terimaDataSuhu(float suhu){
//   printf("Data suhu diterima: %d\n", suhu);
// }

void task2(void *pvParameters){
  for(;;){

    vTaskDelay(4000/portTICK_PERIOD_MS);
    disableCore1WDT();

    char* item = buffer[front];
    front = (front + 1) % BUFFER_SIZE;
  
    char* dta = item;
    char * token = strtok(dta, "\\!/");
    char **rawdata_pp = (char **) malloc(10*sizeof(char *));

    int idx=0;
    rawdata_pp[idx++] = token;
    while( token != NULL) {
      printf("%s\n", token); //printing each token
      token = strtok(NULL, "\\!/");
      rawdata_pp[idx++] = token;
    }

      char const *str1 ="TC1";
      char* str2 = rawdata_pp[1];
      char const *str3 = rawdata_pp[0];
      char const *str4 = "PH";
      char const *str5 = "TDS";

      if (strcmp(str1, str2) == 0){

        float TC = atof(rawdata_pp[2]);
        float HU = atof(rawdata_pp[4]);
        float DIS = atof(rawdata_pp[6]);

        struct2s.push(data2::record2{ TC, HU, DIS });
        if (struct2s.isFull()) {
        Serial.println("\nStack is full:");
          while (!struct2s.isEmpty()) {
            data2::print(struct2s.shift());
            Serial.println();
          }
        Serial.println("START AGAIN");
        }

        ThingSpeak.setField(1, rawdata_pp[2]);
        ThingSpeak.setField(2, rawdata_pp[4]);
        ThingSpeak.setField(3, rawdata_pp[6]);

        ThingSpeak.writeFields(myChannelNumber, myWriteAPIKey);


        ///----------------------------------------------------------------------///
      }
      else if (strcmp(str4, str3) == 0){
        ThingSpeak.setField(4, rawdata_pp[1]);

        ThingSpeak.writeFields(myChannelNumber, myWriteAPIKey);
      }
      else if (strcmp(str5, str3) == 0){
        ThingSpeak.setField(5, rawdata_pp[1]);

        ThingSpeak.writeFields(myChannelNumber, myWriteAPIKey);
      }
      //  vTaskDelay(2000/portTICK_PERIOD_MS);

  }
}

void task3(void *pvParameters){
  for(;;){
    vTaskDelay(11000/portTICK_PERIOD_MS);

    if (struct2s.first().TC != NULL){
      double x = struct2s.first().TC;
      double y;
      int i;

      // Compute output
      y = 0.0;
      y += 0.9998001247048035 * x;

      printf("Prediction Output: %lf\n", y);

      // ThingSpeak.setField(6, y);

      // ThingSpeak.writeFields(myChannelNumber, myWriteAPIKey);
    }
  }
}

// void task3(void *pvParameters){
//   for(;;){
    
//     vTaskDelay(11000/portTICK_PERIOD_MS);
//     // float suhu = 0;
//     if (struct2s.first().TC != NULL){
//         double x = struct2s.first().TC;
//         double y;
//         int i;

//         // Compute output
//         y = 0.0;
//         y += 0.9998001247048035 * x;

//       printf("Prediction Output: %lf\n", y);

//       // ThingSpeak.setField(6, y);

//       // ThingSpeak.writeFields(myChannelNumber, myWriteAPIKey);
//     }
//     else{
//       printf("coba \n");
//     }
//   //  time_t last_received_time = time(NULL);
//   //   bool data_received = false;
    
//   //   while (1) {
//   //       time_t current_time = time(NULL);
//   //       int elapsed_time = difftime(current_time, last_received_time);

//   //       if (elapsed_time >= INTERVAL) {
//   //           if (struct2s.first().TC != NULL) {
//   //               // Data suhu diterima dalam rentang waktu, kirim data aktual
//   //               printf("Data suhu diterima.\n");
//   //               // Misalnya, dapatkan data suhu dari sensor
//   //               // dan simpan dalam variabel 'suhu'
//   //               float suhu = struct2s.first().TC; // Contoh nilai suhu
//   //               terimaDataSuhu(suhu);
                
//   //               last_received_time = current_time;
//   //               data_received = true;
//   //           } else {
//   //               // Data suhu tidak diterima dalam rentang waktu, lakukan prediksi
//   //               printf("Data suhu tidak diterima. Melakukan prediksi.\n");
//   //               double x = struct2s.first().TC;
//   //               double y;
//   //               int i;

//   //               // Compute output
//   //               y = 0.0;
//   //               y += 0.9998001247048035 * x;

//   //               int suhu = y;

//   //               terimaDataSuhu(suhu);
                
//   //               last_received_time = current_time;
//   //               data_received = false;
//   //           }
//   //       }


//   //       sleep(1); // Tunggu 1 detik
//   //   }
//     //  vTaskDelay(2000/portTICK_PERIOD_MS);
//   }
// }