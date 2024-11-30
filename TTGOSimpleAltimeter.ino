/*
    TTGO Mini Bear Altimeter ver0.1
    Copyright Boris du Reau 2012-2024
    This is using a BMP085 or BMP180 presure sensor
    to compile it use ESP32 S3 Dev module
    This uses an 24LC512 eeprom to record the flight

    Major changes on version 0.1
    Initial version of the code, this is re-using code for the TTGO Bear Altimeter firmware
    can record altitude, acceleration, temperature and pressure


*/

#include <TFT_eSPI.h>
#include <Button2.h>
#include <TFT_eWidget.h>               // Widget library
#include <driver/rtc_io.h>
//#include "config.h"

#include "Bear_BMP085.h"
#include "kalman.h"
#include <Pangodream_18650_CL.h>

#include "images/bear_altimeters128x128.h"
#include "images/battery_01.h"
#include "images/battery_02.h"
#include "images/battery_03.h"
#include "images/battery_04.h"
#include "images/battery_05.h"

#include <Preferences.h>
Preferences preferences;

unsigned long initialTime = 0;

unsigned long timeToApogee = 0;

#define BTN_UP 47
#define BTN_DWN 0 // Pinnumber for button for down/next and back / exit actions (don't change this if you want to use the onboard buttons)

Button2 btnUp(BTN_UP); // Initialize the up button
Button2 btnDwn(BTN_DWN); // Initialize the down button

TFT_eSPI tft = TFT_eSPI();
GraphWidget gr = GraphWidget(&tft);    // Graph widget
// Flight curves are drawn on tft using graph instance
TraceWidget trAltitude = TraceWidget(&gr);    // Altitude
TraceWidget trTemperature = TraceWidget(&gr);
TraceWidget trPressure = TraceWidget(&gr);


//////////////////////////////////////////////////////////////////////
// Global variables
//////////////////////////////////////////////////////////////////////
#define ICON_WIDTH 70
#define ICON_HEIGHT 36
#define STATUS_HEIGHT_BAR ICON_HEIGHT
#define ARRAY_SIZE(x) (sizeof(x) / sizeof(x[0]))
#define ICON_POS_X (tft.width() - ICON_WIDTH)

#define MIN_USB_VOL 4.7
//#define ADC_PIN 34
#define ADC_PIN 4
#define CONV_FACTOR 1.8
#define READS 20
#define MAJOR_VERSION 0
#define MINOR_VERSION 1
#define BOARD_FIRMWARE "TTGOSimpleAltimeter"
#define IMU_SDA      16
#define IMU_SCL      17
#define PIN_LCD_BL   10
#ifndef RX1
#define RX1 33
#endif

#ifndef TX1
#define TX1 32
#endif

Pangodream_18650_CL BL(ADC_PIN, CONV_FACTOR, READS);

// Built in button GPIO - adjust for your board
#define BUTTON_GPIO GPIO_NUM_35

bool inGraph = false;

BMP085 bmp;

//telemetry
boolean telemetryEnable = false;
long lastTelemetry = 0;
boolean liftOff = false;

// current file number that you are recording
int currentFileNbr = 0;

//ground level altitude
long initialAltitude;

//stop recording a maximum of 120 seconds after main has fired
//long recordingTimeOut = 120 * 1000;
boolean canRecord = true;
boolean exitRecording = false;

long liftoffAltitude = 20;
long lastAltitude;
//current altitude
long currAltitude;
long diplayedFlightNbr = 0;
long currentCurveType = 0;
long apogeeAltitude = 0;

//nbr of measures to do so that we are sure that apogee has been reached
unsigned long measures = 5;

/*
   drawingText(String text)

*/
void drawingText(String text) {
  tft.fillRect(0, 0, ICON_POS_X, ICON_HEIGHT, TFT_BLACK);
  tft.setTextDatum(5);
  tft.drawString(text, ICON_POS_X - 2, STATUS_HEIGHT_BAR / 2, 4);
}

/*
   tft_output(int16_t x, int16_t y, uint16_t w, uint16_t h, uint16_t* bitmap)
*/
bool tft_output(int16_t x, int16_t y, uint16_t w, uint16_t h, uint16_t* bitmap)
{
  if ( y >= tft.height() ) return 0;
  tft.pushImage(x, y, w, h, bitmap);
  return 1;
}

/*
   button_init()

*/
void button_init()
{
  btnUp.setLongClickHandler([](Button2 & b) {
    // Button up for more than 3s
    // turn off the altimeter
    unsigned int time = b.wasPressedFor();
    if (time >= 3000) {
      Serial.println("Turning off");
      inGraph = false;
      enter_sleep();
    }
  });


  btnDwn.setLongClickHandler([](Button2 & b) {
    // Button Down slow
    // if you press the down button more than 1s and less than 10s
    // This will read the flights
    Serial.println("Button Down slow");
    unsigned int time = b.wasPressedFor();
    if (time >= 1000 & time < 10000) {
      if (!inGraph) {
        long lastFlightNbr = 0;
        preferences.begin("flights", false);
        bool doesExist = preferences.isKey("nbr");
        if (doesExist) {
          lastFlightNbr = preferences.getInt("nbr");
        }
        preferences.end();
        Serial.print("lastFlightNbr:");
        Serial.println(lastFlightNbr);
        if (!(lastFlightNbr < 1)) {
          inGraph = true;
          diplayedFlightNbr = 1;
          printFlightNbr(diplayedFlightNbr);
        }
      } else {
        inGraph = false;
        tft.init();
        tft.fillScreen(TFT_BLACK);

        // Graph area is 200 pixels wide, 150 high, dark grey background
        gr.createGraph(200, 100, tft.color565(5, 5, 5));
        // x scale units is from 0 to 100, y scale units is 0 to 50
        gr.setGraphScale(0.0, 100.0, 0, 50.0);
      }
    }
    // If you press the button more than 10s it will erase all flights
    if (time >= 10000) {
      inGraph = false;
      tft.init();
      tft.fillScreen(TFT_BLACK);

      // Graph area is 200 pixels wide, 150 high, dark grey background
      gr.createGraph(200, 100, tft.color565(5, 5, 5));
      // x scale units is from 0 to 100, y scale units is 0 to 50
      gr.setGraphScale(0.0, 100.0, 0, 50.0);
      Serial.println("Erasing flights!!!");

      // erasing flights
      preferences.begin("flights", false);
      preferences.clear();
      preferences.end();
      currentFileNbr = 0;
    }
  });

  btnDwn.setClickHandler([](Button2 & b) {
    // Button Down fast
    // this will navigate between flights
    Serial.println("Button Down fast"); // It's called upCmd because it increases the index of an array. Visually that would mean the selector goes downwards.
    if (inGraph) {
      preferences.begin("flights", false);
      long lastFlightNbr = 0;
      bool doesExist = preferences.isKey("nbr");
      if (doesExist) {
        lastFlightNbr = preferences.getInt("nbr");
      }
      //Make sure we have no reach the last flight
      if (lastFlightNbr > diplayedFlightNbr) {
        diplayedFlightNbr ++;
        Serial.print("Flight:");
        Serial.println(diplayedFlightNbr);
      } else {
        // if not lets go back to the first one if it exists
        if (!(lastFlightNbr < 1)) {
          diplayedFlightNbr = 1;
        }
      }
      preferences.end();
      printFlightNbr(diplayedFlightNbr);
    }
  });
}

/*
   button_loop()

*/
void button_loop()
{
  // Check for button presses
  btnUp.loop();
  btnDwn.loop();
}

/*
   setup()

*/
void setup() {
  // initialise the connection
  Wire.begin(IMU_SDA, IMU_SCL);
  Serial.begin(115200);
  Serial.println("Starting");
  pinMode(14, OUTPUT);
  digitalWrite(14, HIGH);

  //Disable backlight hold on
  gpio_hold_dis((gpio_num_t)PIN_LCD_BL);
  pinMode(PIN_LCD_BL, OUTPUT);
  digitalWrite(PIN_LCD_BL, LOW);

  tft.init();
  tft.fillScreen(TFT_BLACK);
  tft.setSwapBytes(true);
  if (!bmp.begin(0)) {
    Serial.println("Could not find a valid BMP085 sensor, check wiring!");
    /* tft.drawString("Barometric sensor does not work", 6, 135);
      while (1) {
      }  */
  }
  tft.setRotation(2);
  tft.pushImage(6, 0, 128, 128, bear_altimeters128x128);
  tft.drawString("Bear Altimeter", 6, 135);
  tft.drawString("ver 1.0", 6, 145);
  tft.drawString("Copyright", 6, 155);
  tft.drawString("Boris du Reau", 6, 165);
  tft.drawString("2012-2024", 6, 175);
  tft.drawString("Initializing....", 6, 185);

  // init Kalman filter
  KalmanInit();
  // let's do some dummy altitude reading
  // to initialise the Kalman filter
  for (int i = 0; i < 50; i++) {
    ReadAltitude();
  }
  //let's read the launch site altitude
  float sum = 0;
  for (int i = 0; i < 10; i++) {
    sum += ReadAltitude();
    delay(50);
  }
  initialAltitude = (long)(sum / 10.0);
  preferences.begin("flights", false);
  bool doesExist = preferences.isKey("nbr");
  int flightCount = 0;
  if (doesExist) {
    flightCount = preferences.getInt("nbr");
    currentFileNbr = flightCount + 1;
  } else  {
    preferences.putInt("nbr", 0);
    currentFileNbr = 0;
  }
  preferences.end();

  button_init();

  tft.setRotation(3);
  tft.fillScreen(TFT_BLACK);
  tft.drawString("altitude", 6, 0);

  // Graph area is 100 pixels wide, 100 high, dark grey background
  gr.createGraph(100, 100, tft.color565(5, 5, 5));
  // x scale units is from 0 to 100, y scale units is 0 to 50
  gr.setGraphScale(0.0, 100.0, 0, 50.0);

  tft.setTextColor(TFT_WHITE, TFT_BLACK);
  tft.fillScreen(TFT_BLACK);
  tft.setSwapBytes(true);
  tft.setTextFont(1);
  tft.setTextSize(1);
}

/*
   loop()

*/
void loop() {

  char readVal = ' ';
  int i = 0;

  char commandbuffer[100];

  while ( readVal != ';')
  {
    button_loop();

    currAltitude = (long)ReadAltitude() - initialAltitude;
    if (liftOff)
      SendTelemetry(millis() - initialTime, 200);
    if (!(currAltitude > liftoffAltitude) )
    {
      if (!inGraph) {
        SendTelemetry(0, 500);
        tft.setCursor (0, STATUS_HEIGHT_BAR);

        if (BL.getBatteryVolts() >= MIN_USB_VOL) {
          drawingText("Chrg");
          tft_output(ICON_POS_X, 0, ICON_WIDTH, 36, (uint16_t*) battery_01);
          delay(500);
          tft_output(ICON_POS_X, 0, ICON_WIDTH, 36, (uint16_t*) battery_02);
          delay(500);
          tft_output(ICON_POS_X, 0, ICON_WIDTH, 36, (uint16_t*) battery_03);
          delay(500);
          tft_output(ICON_POS_X, 0, ICON_WIDTH, 36, (uint16_t*) battery_04);
          delay(500);
          tft_output(ICON_POS_X, 0, ICON_WIDTH, 36, (uint16_t*) battery_05);
          delay(500);
        } else {
          int batteryLevel = BL.getBatteryChargeLevel();
          if (batteryLevel >= 80) {
            tft_output(ICON_POS_X, 0, 70, 36, (uint16_t*) battery_04);
          } else if (batteryLevel < 80 && batteryLevel >= 50 ) {
            tft_output(ICON_POS_X, 0, 70, 36, (uint16_t*) battery_03);
          } else if (batteryLevel < 50 && batteryLevel >= 20 ) {
            tft_output(ICON_POS_X, 0, 70, 36, (uint16_t*) battery_02);
          } else if (batteryLevel < 20 ) {
            tft_output(ICON_POS_X, 0, 70, 36, (uint16_t*) battery_01);
          }
          drawingText(String(batteryLevel) + "%");
        }
        char Altitude [15];
        currAltitude = (long)ReadAltitude() - initialAltitude;
        sprintf(Altitude, "Altitude = %i meters    ", currAltitude );
        tft.setCursor (0, STATUS_HEIGHT_BAR);
        tft.println("                                     ");
        tft.println(Altitude);
      }


      while (Serial.available())
      {
        readVal = Serial.read();
        if (readVal != ';' )
        {
          if (readVal != '\n')
            commandbuffer[i++] = readVal;
        }
        else
        {
          commandbuffer[i++] = '\0';
          break;
        }
      }
    }
    else {
      Serial.println("Recording!!!!");
      recordAltitude();
    }
  }
  interpretCommandBuffer(commandbuffer);
}


/*
   ReadAltitude()

*/

float ReadAltitude()
{
  return KalmanCalc(bmp.readAltitude());
}

/*

   enter_sleep()

*/
void enter_sleep()
{
  tft.fillScreen(TFT_BLACK);
  tft.setRotation(0);
  tft.drawString("turning off...", 6, 185);
  digitalWrite(4, LOW);
  delay(2000);
  //set display goto deep sleep 100uA
  tft.writecommand(0x10);

  // High level off the backlight
  digitalWrite(PIN_LCD_BL, HIGH);
  //Enable backlight hold on high level
  gpio_hold_en((gpio_num_t)PIN_LCD_BL);
  pinMode(BUTTON_GPIO, INPUT_PULLUP);
  rtc_gpio_hold_en(BUTTON_GPIO);
  esp_sleep_enable_ext1_wakeup(GPIO_SEL_0, ESP_EXT1_WAKEUP_ALL_LOW);
  esp_deep_sleep_start();
}

void printFlightNbr(int diplayedFlightNbr) {
  tft.fillScreen(TFT_BLACK);
  //tft.setRotation(0);
  preferences.begin("flights", false);
  char flight[15] = "";
  sprintf(flight, "flight%i", diplayedFlightNbr);

  char flightDuration[15] = "";
  sprintf(flightDuration, "duration%i", diplayedFlightNbr);

  char flightApogee[15] = "";
  sprintf(flightApogee, "aduration%i", diplayedFlightNbr);

  Serial.println(flight);
  bool doesExist = preferences.isKey(flight);
  if (doesExist) {
    char apogeeText[50];
    int apogee = preferences.getInt(flight);
    sprintf(apogeeText, "%i m      ", apogee);

    char flightDurationText[50];
    long duration = preferences.getLong(flightDuration);
    sprintf(flightDurationText, "%.2f secs      ", (float)duration / (float)1000);

    char timeToApogeeText[50];
    long apogeeDuration = preferences.getLong(flightApogee);
    sprintf(timeToApogeeText, "%.2f secs      ", (float)apogeeDuration / (float)1000);

    tft.setCursor (0, 0);
    tft.setTextSize(2);

    tft.print("Flight ");
    tft.println(diplayedFlightNbr);
    tft.setTextSize(1);
    tft.println(" ");
    tft.println("Apogee: ");
    tft.println(apogeeText);
    tft.println(" ");
    tft.println("Flight duration: ");
    tft.println(flightDurationText);
    tft.println(" ");
    tft.println("Time to apogee: ");
    tft.println(timeToApogeeText);
    Serial.println("Flight exist");
    Serial.println(apogee);
  } else {

    tft.setCursor (0, STATUS_HEIGHT_BAR);
    tft.println("no flights");
    Serial.println("Flight does not exist");
  }
  preferences.end();
}

/*
   recordAltitude()


*/
void recordAltitude()
{
  long recordingTimeOut = 120 * 1000;

  exitRecording = false;

  lastAltitude = 0;

  while (!exitRecording)
  {
    //read current altitude
    currAltitude = (ReadAltitude() - initialAltitude);

    if ((currAltitude > liftoffAltitude) && !liftOff)
    {
      liftOff = true;
      SendTelemetry(0, 200);
      // save the time
      initialTime = millis();
    }
    unsigned long prevTime = 0;
    long prevAltitude = 0;
    // loop until we have reach an altitude of 3 meter
    while (liftOff)
    {
      unsigned long currentTime;
      unsigned long diffTime;

      currAltitude = (ReadAltitude() - initialAltitude);

      currentTime = millis() - initialTime;

      prevAltitude = currAltitude;
      SendTelemetry(currentTime, 200);
      diffTime = currentTime - prevTime;
      prevTime = currentTime;
      //display
      char Altitude [15];
      currAltitude = (long)ReadAltitude() - initialAltitude;
      sprintf(Altitude, "Altitude = %i meters    ", currAltitude );
      tft.setCursor (0, STATUS_HEIGHT_BAR);
      tft.println("Recording in progress .....");
      tft.println(Altitude);

      // detect apogee
      if (currAltitude < lastAltitude) {
        measures = measures - 1;
        if (measures == 0)
        {
          apogeeAltitude = lastAltitude;
          timeToApogee = currentTime;
        }
      }
      else
      {
        lastAltitude = currAltitude;
        measures = 5; //config.nbrOfMeasuresForApogee;
      }

      // detect landing
      if ((canRecord  && (currAltitude < 10) && (millis() - timeToApogee) > 2000) || (canRecord  && (millis() - initialTime) > recordingTimeOut) )
      {
        //end loging
        liftOff = false;

        preferences.begin("flights", false);
        bool doesExist = preferences.isKey("nbr");
        int flightCount = 0;
        if (!doesExist) {
          preferences.putInt("nbr", flightCount);
        }
        flightCount = preferences.getInt("nbr");
        char flight[15];
        sprintf(flight, "flight%i", flightCount + 1);
        char flightDuration[15];
        sprintf(flightDuration, "duration%i", flightCount + 1);
        char flightApogeeDuration[15];
        sprintf(flightApogeeDuration, "aduration%i", flightCount + 1);
        //save flight apogee
        preferences.putInt(flight, apogeeAltitude);
        preferences.putLong(flightDuration, millis() - initialTime);
        preferences.putLong(flightApogeeDuration, timeToApogee);
        preferences.putInt("nbr", flightCount + 1);

        preferences.end();
        lastAltitude = 0;
        apogeeAltitude = 0;

        exitRecording = true;
        /*if (currentFileNbr < 25)
          currentFileNbr ++;*/
      }
    } // end while (liftoff)
  } //end while(recording)
}

/*

   This interprets menu commands. This can be used in the commend line or
   this is used by the Android console

   Commands are as folow:
   a  get all flight data
   b  get altimeter config
   c  toggle continuity on and off
   d  reset alti config
   e  erase all saved flights
   f  FastReading on
   g  FastReading off
   h  hello. Does not do much
   i  unused
   k  folowed by a number turn on or off the selected output
   l  list all flights
   m  followed by a number turn main loop on/off. if number is 1 then
      main loop in on else turn it off
   n  Return the number of recorded flights in the EEprom
   o  requesting test trame
   r  followed by a number which is the flight number.
      This will retrieve all data for the specified flight
   s  write altimeter config
   t  reset alti config (why?)
   w  Start or stop recording
   x  delete last curve
   y  followed by a number turn telemetry on/off. if number is 1 then
      telemetry in on else turn it off
   z  send gps raw data

*/
void interpretCommandBuffer(char *commandbuffer) {
  //get all flight data
  if (commandbuffer[0] == 'a')
  {
    Serial.print(F("$start;\n"));
    int i;
    ///todo
    /*for (i = 0; i < logger.getLastFlightNbr() + 1; i++)
      {
      logger.printFlightData(i);
      }*/
    Serial.print(F("$end;\n"));
  }
  //get altimeter config
  else if (commandbuffer[0] == 'b')
  {
    Serial.print(F("$start;\n"));

    printAltiConfig();
    Serial.print(F("$end;\n"));
  }
  //toggle continuity on and off
  else if (commandbuffer[0] == 'c')
  {

  }
  //reset alti config this is equal to t why do I have 2 !!!!
  else if (commandbuffer[0] == 'd')
  {
    /*defaultConfig();
      writeConfigStruc();
      initAlti();*/
  }
  //this will erase all flight
  else if (commandbuffer[0] == 'e')
  {

    /*logger.clearFlightList();
      logger.writeFlightList();*/
    currentFileNbr = 0;
    //currentMemaddress = 201;
  }
  //FastReading
  else if (commandbuffer[0] == 'f')
  {
    /*FastReading = true;
      Serial.print(F("$OK;\n"));
      SerialCom.print(F("$OK;\n"));*/

  }
  //FastReading off
  else if (commandbuffer[0] == 'g')
  {
    /*FastReading = false;
      Serial.print(F("$OK;\n"));
      SerialCom.print(F("$OK;\n"));*/
  }
  //hello
  else if (commandbuffer[0] == 'h')
  {
    Serial.print(F("$OK;\n"));
    //SerialCom.print(F("$OK;\n"));
  }
  // unused
  else if (commandbuffer[0] == 'i')
  {
    //exit continuity mode
  }
  //turn on or off the selected output
  else if (commandbuffer[0] == 'k')
  {

  }
  //list all flights
  else if (commandbuffer[0] == 'l')
  {
    //logger.printFlightList();
  }

  //mainloop on/off
  else if (commandbuffer[0] == 'm')
  {
    if (commandbuffer[1] == '1') {
#ifdef SERIAL_DEBUG
      //SerialCom.print(F("main Loop enabled\n"));
#endif
      //mainLoopEnable = true;
    }
    else {
#ifdef SERIAL_DEBUG
      //SerialCom.print(F("main loop disabled\n"));
#endif
      //mainLoopEnable = false;
    }

    Serial.print(F("$OK;\n"));
    //SerialCom.print(F("$OK;\n"));
  }
  //Number of flight
  else if (commandbuffer[0] == 'n')
  {
    char flightData[30] = "";
    char temp[9] = "";

    Serial.print(F("$start;\n"));
    //SerialCom.print(F("$start;\n"));

    strcat(flightData, "nbrOfFlight,");
    //sprintf(temp, "%i,", logger.getLastFlightNbr() + 1 );
    strcat(flightData, temp);
    unsigned int chk = msgChk(flightData, sizeof(flightData));
    sprintf(temp, "%i", chk);
    strcat(flightData, temp);
    strcat(flightData, ";\n");

    Serial.print("$");
    Serial.print(flightData);
    Serial.print(F("$end;\n"));
  }
  // send test tram
  else if (commandbuffer[0] == 'o')
  {

    /*Serial.print(F("$start;\n"));
      SerialCom.print(F("$start;\n"));

      sendTestTram();
      Serial.print(F("$end;\n"));
      SerialCom.print(F("$end;\n"));*/
  }
  //altimeter config param
  //write  config
  else if (commandbuffer[0] == 'p')
  {
    /*if (writeAltiConfigV2(commandbuffer)) {
      Serial.print(F("$OK;\n"));
      SerialCom.print(F("$OK;\n"));
      }
      else {
      Serial.print(F("$KO;\n"));
      SerialCom.print(F("$KO;\n"));
      }*/
  }
  else if (commandbuffer[0] == 'q')
  {
    /*writeConfigStruc();
      readAltiConfig();
      initAlti();
      Serial.print(F("$OK;\n"));
      SerialCom.print(F("$OK;\n"));*/
  }
  //this will read one flight
  else if (commandbuffer[0] == 'r')
  {
    char temp[3];

    temp[0] = commandbuffer[1];
    if (commandbuffer[2] != '\0')
    {
      temp[1] = commandbuffer[2];
      temp[2] = '\0';
    }
    else
      temp[1] = '\0';

    if (atol(temp) > -1)
    {
      Serial.print(F("$start;\n"));

      //logger.printFlightData(atoi(temp));

      Serial.print(F("$end;\n"));
    }
    else
      Serial.println(F("not a valid flight"));
  }
  //write altimeter config
  else if (commandbuffer[0] == 's')
  {

  }
  //reset config and set it to default
  else if (commandbuffer[0] == 't')
  {
    //reset config
    /*defaultConfig();
      writeConfigStruc();
      initAlti();
      SerialCom.print(F("config reseted\n"));*/
  }
  // Recording
  else if (commandbuffer[0] == 'w')
  {
    //SerialCom.println(F("Recording \n"));
    recordAltitude();
  }
  //delete last curve
  else if (commandbuffer[0] == 'x')
  {
    /*logger.eraseLastFlight();
      logger.readFlightList();
      long lastFlightNbr = logger.getLastFlightNbr();
      if (lastFlightNbr < 0)
      {
      currentFileNbr = 0;
      currentMemaddress = 201;
      }
      else
      {
      currentMemaddress = logger.getFlightStop(lastFlightNbr) + 1;
      currentFileNbr = lastFlightNbr + 1;
      }
      canRecord = logger.CanRecord();*/
  }

  //telemetry on/off
  else if (commandbuffer[0] == 'y')
  {
    if (commandbuffer[1] == '1') {
      //SerialCom.print(F("Telemetry enabled\n"));
      telemetryEnable = true;
    }
    else {
      //SerialCom.print(F("Telemetry disabled\n"));
      telemetryEnable = false;
    }
    Serial.print(F("$OK;\n"));
  }


  // empty command
  else if (commandbuffer[0] == ' ')
  {
    Serial.print(F("$K0;\n"));
  }
  else
  {
    //SerialCom.print(F("$UNKNOWN;"));
    //SerialCom.println(commandbuffer[0]);
  }
}

unsigned int msgChk( char * buffer, long length ) {

  long index;
  unsigned int checksum;

  for ( index = 0L, checksum = 0; index < length; checksum += (unsigned int) buffer[index++] );
  return (unsigned int) ( checksum % 256 );
}

/*

   Print altimeter config to the Serial line

*/
void printAltiConfig()
{
  char altiConfig[120] = "";
  char temp[10] = "";

  strcat(altiConfig, "alticonfig,");

  //Unit
  sprintf(temp, "%i,", 0);
  strcat(altiConfig, temp);
  //beepingMode
  sprintf(temp, "%i,", 0);
  strcat(altiConfig, temp);
  //output1
  sprintf(temp, "%i,", 0);
  strcat(altiConfig, temp);
  //output2
  sprintf(temp, "%i,", 0);
  strcat(altiConfig, temp);
  //output3
  sprintf(temp, "%i,", 0);
  strcat(altiConfig, temp);
  //supersonicYesNo
  sprintf(temp, "%i,", 0);
  strcat(altiConfig, temp);
  //mainAltitude
  sprintf(temp, "%i,", 0);
  strcat(altiConfig, temp);
  //AltimeterName
  strcat(altiConfig, BOARD_FIRMWARE);
  strcat(altiConfig, ",");
  //alti major version
  sprintf(temp, "%i,", MAJOR_VERSION);
  strcat(altiConfig, temp);
  //alti minor version
  sprintf(temp, "%i,", MINOR_VERSION);
  strcat(altiConfig, temp);
  //output1 delay
  sprintf(temp, "%i,", 0);
  strcat(altiConfig, temp);
  //output2 delay
  sprintf(temp, "%i,", 0);
  strcat(altiConfig, temp);
  //output3 delay
  sprintf(temp, "%i,", 0);
  strcat(altiConfig, temp);
  //Beeping frequency
  sprintf(temp, "%i,", 0);
  strcat(altiConfig, temp);
  sprintf(temp, "%i,", 0);
  strcat(altiConfig, temp);
  sprintf(temp, "%i,", 0);
  strcat(altiConfig, temp);
  sprintf(temp, "%i,", 0);
  strcat(altiConfig, temp);
  sprintf(temp, "%i,", 0);
  strcat(altiConfig, temp);
  sprintf(temp, "%lu,", 0);
  strcat(altiConfig, temp);
  sprintf(temp, "%i,", 0);
  strcat(altiConfig, temp);
  sprintf(temp, "%i,", 0);
  strcat(altiConfig, temp);
  sprintf(temp, "%i,", 0);
  strcat(altiConfig, temp);
  //output4
  sprintf(temp, "%i,", 0);
  strcat(altiConfig, temp);
  //output4 delay
  sprintf(temp, "%i,", 0);
  strcat(altiConfig, temp);
  //Lift off altitude
  sprintf(temp, "%i,", 0);
  strcat(altiConfig, temp);
  //Battery type
  sprintf(temp, "%i,", 0);
  strcat(altiConfig, temp);
  // recording timeout
  sprintf(temp, "%i,", 0);
  strcat(altiConfig, temp);
  //altiID
  sprintf(temp, "%i,", 0);
  strcat(altiConfig, temp);
  //useTelemetryPort
  sprintf(temp, "%i,", 0);
  strcat(altiConfig, temp);
  unsigned int chk = 0;
  chk = msgChk( altiConfig, sizeof(altiConfig) );
  sprintf(temp, "%i;\n", chk);
  strcat(altiConfig, temp);
  Serial.print("$");
  Serial.print(altiConfig);
  //SerialCom.print("$");

  // the following will slow down the connection speed so that it works better with telemetry module
  // or bluetooth module with no buffer
  //if (config.useTelemetryPort == 1){
  for (int j = 0; j < sizeof(altiConfig); j++) {
    //SerialCom.print(altiConfig[j]);
    delay(2);
  }
  /*}
    else
    SerialCom.print(altiConfig);*/
}


/*
   SendTelemetry(long sampleTime, int freq)
   Send telemety so that we can plot the flight

*/
void SendTelemetry(long sampleTime, int freq) {
  char altiTelem[150] = "";
  char temp[10] = "";

  if (telemetryEnable && (millis() - lastTelemetry) > freq) {
    lastTelemetry = millis();
    int val = 0;
    //check liftoff
    int li = 0;
    if (liftOff)
      li = 1;


    int landed = 0;
    if ( liftOff && currAltitude < 10)
      landed = 1;

    strcat(altiTelem, "telemetry," );
    sprintf(temp, "%i,", currAltitude);
    strcat(altiTelem, temp);
    sprintf(temp, "%i,", li);
    strcat(altiTelem, temp);
    sprintf(temp, "%i,", -1);
    strcat(altiTelem, temp);
    sprintf(temp, "%i,", apogeeAltitude);
    strcat(altiTelem, temp);
    sprintf(temp, "%i,", -1);
    strcat(altiTelem, temp);
    sprintf(temp, "%i,", -1);
    strcat(altiTelem, temp);
    sprintf(temp, "%i,", landed);
    strcat(altiTelem, temp);
    sprintf(temp, "%i,", sampleTime);
    strcat(altiTelem, temp);
    strcat(altiTelem, "-1,");
    strcat(altiTelem, "-1,");
    strcat(altiTelem, "-1,");
    strcat(altiTelem, "-1,");

    dtostrf(BL.getBatteryVolts(), 4, 2, temp);
    strcat(altiTelem, temp);
    strcat(altiTelem, ",");

    // temperature
    float temperature;
    temperature = bmp.readTemperature();
    sprintf(temp, "%i,", (int)temperature );
    strcat(altiTelem, temp);

    //sprintf(temp, "%i,", (int)(100 * ((float)logger.getLastFlightEndAddress() / endAddress)) );
    strcat(altiTelem, temp);
    //sprintf(temp, "%i,", logger.getLastFlightNbr() + 1 );
    strcat(altiTelem, temp);

    //drogueFiredAltitude
    sprintf(temp, "%i,", -1);
    strcat(altiTelem, temp);

    unsigned int chk;
    chk = msgChk(altiTelem, sizeof(altiTelem));
    sprintf(temp, "%i", chk);
    strcat(altiTelem, temp);
    strcat(altiTelem, ";\n");

    Serial.print("$");
    Serial.print(altiTelem);
  }
}
