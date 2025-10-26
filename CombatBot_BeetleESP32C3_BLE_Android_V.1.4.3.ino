// Version
String version = "1.4.3";
/******************************************************/
/*   CombatBot for Beetle ESP32-C3 - BLE - Android    */
/******************************************************

  Free script designed to provide the platform for hobbyists and educational use as a starting point for
  controlling 150-450 g Combat Robots with Android smartphone/tablet via Bluetooth Low Energy (BLE).
  
  Free Bot Controller Android app is available on our GitHub page.
  
  Script and controller can also be used to control basic Arduino based bots with servo.

  *****
  FEATURES:
  *****
  - High accuracy tank steering with center brake.
  - Channel mixing between ch1(fwd/bwd) and ch2(left/right) channels.
  - Drive motor rotation inverting and trim.
  - Supports DC motors using HR8833 or TB6612FNG DC motor driver or Brushless motors using ESC's for driving.
  - Support for single servo or brushless motor controlled by ESC as weapon motor.
  - 1-2S LiPo battery voltage monitoring.
  - Low voltage guard.
  - Failsafe & ouf of range protection.
  - Customizable AI slot (ch4, void runAI).
  - Buildin automatic support in Custom AI slot for flippers using DFRobot VL6180X ToF Distance Ranging Sensor.
  - Weapon/servo presets, drive motor directions and trim levels can be set from script presets.
  - Bot name, weapon/servo presets, drive motor directions and trim levels can be updated straight from controller.
  - Signal and variable debugging through serial monitor.
  - Onboard led indicates error-, standby-, bluetooth- and weapon statuses as followed:
      - Blink, dim/slow     =   Standby
      - Blink, bright/fast  =   Error: Wrong weapon parameter / Telemetry not enabled when using Low voltage guard / Battery voltage not recognized by telemetry / Telemetry voltage divider pin overload (Happens when ground resistor is broken or cut loose and pin receives battery voltage.)
      - Solid, dim          =   Bluetooth connected
      - Solid, bright       =   Weapon signal active
      
  *****
  HARDWARE EXAMPLE (150g With DC Motors):
  *****
  - DFRobot - Beetle ESP32-C3
  - DFRobot - Fermion: HR8833 Thumbnail Sized DC Motor Driver 2x1.5A or TB6612FNG 2x1.2A DC Motor Driver
  - DFRobot - Fermion: VL6180X ToF Distance Ranging Sensor 5-100mm (Optional, for flippers.)
  - Rebuild Robotics - ESP32-C3 CombatBot Expansion Board (Prototype) or 5V BEC to convert battery voltage for boards
  - Rebuild Robotics - TinySwitch (Prototype)
  - Rebuild Robotics - TinyLED (Prototype)
  - 6V 600/1000RPM Geared N10/N20 DC motors
  - Servo for flipper or brushless motor and ESC for spinning weapon
  - 2S 180-300mAh LiPo battery
  - 22AWG Wires for drive motors, calculate main power and ESC needs separately
  - BT2.0 Connector pair (Replace battery JST with BT2.0 male connector. Remember to prevent battery shortcut!)
  - 10kOhm pull-down resistor for pin 6 with HR8833 and TB6612 (Included in ESP32-C3 CombatBot Expansion Board)

  *****
  OUTPUTS:
  *****
  - Driving:
    - DC motors:          2 x PWM + 2 digital outputs
    - Brushless motors:   2 x PWM outputs
  - Weapon ESC or servo:  1 x PWM output
  - Weapon PWM signals are generated with servo library.
  - If using brushless ESC's they has to be configured separately!
  
  *****
  HOW TO INSTALL:
  *****
  1. Installing Arduino IDE, Board manager and libraries:
    1.1. Install Arduino IDE from https://www.arduino.cc/en/software. It is the main program what you will use to manage this script and connection to ESP.
    1.2. Install board manager by following guide in wiki: https://wiki.dfrobot.com/SKU_DFR0868_Beetle_ESP32_C3 .
    1.3. Install proper Arduino library versions mentioned below.
  2. Installing script:
    2.1. Download script from Githubs button: "Code" > Download Zip and unzip it.
    2.2. Set up pins and presets from script.
    2.3. Set up board manager presets as mentioned below.
    2.4. Upload script as mentioned below.
  3. Controller:
    3.1. Install controller from: https://github.com/RebuildRobotics/
    3.2. Connect and have fun.
      
  *****
  ARDUINO IDE BOARD MANAGER & LIBRARIES:
  *****
  Tested stable versions:
  - Board:                    esp32 by Espressif Systems v.3.3.0  Install from Arduino IDE      (Uses ESP32 core 3.x)
  - Servo/Weapon control:     ESP32Servo v.3.0.9                  Install from Arduino IDE
  - Distance sensor:          DFRobot_VL6180X v.1.0.1             Install from Arduino IDE
  
  *****
  UPLOADING AND SERIAL MONITORING:
  *****
  - Do not connect ESP into USB cable when battery is connected!
  - Presets needed in Arduino IDE for serial monitoring and uploading script:
    - Tools > Board: > esp32 > DFRobot Beetle ESP32-C3
    - Tools > USB CDC On Boot > Enabled
    - Tools > Upload Speed > 115200
    - Tools > Erase All Flash Before Sketch Upload > Enabled = Erases all bot presets from Eeprom / Disabled = Keeps bot presets in memory.
      
  When updating new version of script flash should always be erased!

  *****
  PRESETS:
  *****
  - Basic presets can be changed from controller or from script below.
  - Presets will be saved into Eeprom when updated from controller.
  - Presets saved in Eeprom will be erased in upload if "Tools > Erase All Flash Before Sketch Upload" is enabled.
  
  *****
  DEBUG:
  *****
  - Debug mode and serial monitoring can be enabled from below by changing "#define DEBUG" to true and reuploading script to ESP.
  - You might have to reset ESP once from button to get clean serial monitor output in start.
  - If facing problems uploading script and having "exit status 2..." error keep Arduino IDE open, plug ESP off from USB port, keep pressing Boot button while plugging ESP back into computer,
    reupload script and let go off from the Boot button when uploading has been started. https://wiki.dfrobot.com/SKU_DFR0868_Beetle_ESP32_C3

  *****  
  COMMUNICATION:
  *****
  - Script uses 2,4GHz Bluetooth Low Energy (BLE) network to receive data from mobile phone controller as string.
  - Can handle incoming bluetooth messages in 20ms intervals.
  - String format ch1:ch2:ch3:ch4\n .
  - ch1 = fwd/bwd speed, ch2 = left/right speed, ch3 = servo angle or weapon speed, ch4 = AI on/off.
  - ch1 and ch2 values are in scale of -100 to 100, ch3 and ch4 0 to 100. 0 is stop. Signals are converted to PWM value scale 0 to 255. Negative numbers are absoluted.

  *****
  FAILSAFE & OUT OF RANGE:
  *****
  - Script includes failsafe and out of range functions to prevent up accidents and bot running away.
  - Functions stops bot from using AI, weapon and motors when controllers signal is lost or bot is out of range.
  - Using failsafe is highly recommended in robotics and it's required in combat robotics. Same kind of functions are used in normal RC transmitters.
  - Failsafe is deactivated when controller is connected into bot, so if you go out of range you have to reconnect into bot for to controlling it.
  - Remember to set pins correctly, if they are not set right script and failsafe won't work properly!

  *****
  LOW VOLTAGE GUARD:
  *****
 - Protects battery by activating failsafe when battery is running low while keeping controller connected to bot.
 - Failsafe will be deactivated when voltage becomes higher than shutdown level.
 - If facing problems with voltage drops and guard is stopping motors with close to charged battery try to use quality connectors like BT2.0 or XT-30 (not JST), proper cable sizes and possible capacitor near weapons power drain.
    
  *****
  CONTROLLER:
  *****
  - Android: Rebuild Robotics - Bot Controller (Download from: https://github.com/RebuildRobotics).
  - Iphone: Not available, hopefully some day.
  - Controller is specially made for controlling combat robots and Arduino based bots.
  - Bot name, weapon/servo presets, drive motor directions and trim levels can be updated straight from controller.
  - When using script with controller primary version numbers should always match (e.g. Controller: 1.2 - Bot Script: 1.2.1 vice versa).

  *****
  SAFETY NOTICE:
  *****
  - Combat robotics is fun and extremely educating but dangerous hobby, always think safety first!
  - Do not keep battery connected at the same time when USB is connected into powersource!
  - Accidentally motor spins will happen when ESP is powered, script is uploading or wrong board or library version is installed!
    Motor spins at startup can be prevented by using pull-up/pull down resistors or our designed ESP32-C3 CombatBot Expansion Board and tested IDE library versions.
  - Configure ESC properly before connecting controller into bot, or it causes serious hazard!
  - Test bot with precaution and only weapon motor attached, not weapon itself!
  - Do not use any other board manager or library versions than those which are mentioned as tested and safe ones in this scripts read me area!
    It might cause serious injuries because board functionalities are changing. Future updates to this script includes fresher and tested information from later versions.
  - Remember always check that you have correct versions installed before updating script into board.
  - Modify script only if you know what you are doing!
  - Script has been tested only in close range at Combat arenas and inside. When driving outside use precaution.

  *****
  SUPPORTING PROJECTS:
  *****
  We do not search any financial benefit from scripts and apps made, we just want to offer platforms to help you to get started with robotics and learn how things are made.
  You are free to support other projects which are used in our programs and have.
  
  *****
  SOURCES:
  *****
  Hardware:
    - https://wiki.dfrobot.com/SKU_DFR0868_Beetle_ESP32_C3
    - https://wiki.dfrobot.com/2x1.2A_DC_Motor_Driver__TB6612FNG__SKU__DRI0044
    - https://wiki.dfrobot.com/Dual_1.5A_Motor_Driver_-_HR8833_SKU__DRI0040
    - https://wiki.dfrobot.com/DFRobot_VL6180X_TOF_Distance_Ranging_Sensor_Breakout_Board_SKU_SEN0427

  Driving:
    - https://youtu.be/Bx0y1qyLHQ4?si=U1tk3dPtz3ZfaxqV
    - https://gist.github.com/ShawnHymel/ccc28335978d5d5b2ce70a2a9f6935f4

  This script has been made respecting the regulations of combat robotics and they should be usable all around the world.
  Script builders and component manufacturers are not responsiple from possible damages.
  Third party script writers hasn't got nothing to do with this project except I'm using their libraries.

  Copyright (C) Ville Ollila (RoboticsIsForNerds, Rebuild Robotics).
  
  This work is licensed under the Creative Commons Attribution-NonCommercial 4.0 International License.
  To view a copy of this license, visit http://creativecommons.org/licenses/by-nc/4.0/
  
  Examples used are linked in functions.

  Third party copyrights:
  ESP32Servo Copyright (C) 2017 John K. Bennett

  Bug reports through https://github.com/RebuildRobotics /program/Issues. Other contacts related this program: support@rebuildrobotics.fi.
*/
// -------------------- Verify board -------------------
#if !defined(ARDUINO_ESP32C3_DEV)
#error This code is build for ESP32-C3! Check Board manager settings.
#endif
/******************************************************/
/* ------------------ BASIC PRESETS ----------------- */
/******************************************************/
// --------------------- Debug mode --------------------
#define DEBUG                       false     // To enable debug mode and serial monitoring change to true, to disable false.
// -------------------- Default name -------------------
String NAME = "CombatBot";                    // Advertised name // max. 19 characters
// -------------------- Drive motors -------------------
// Motor driver type
#define HR8833                                // HR8833 / TB6612 / BRUSHL
// Rotation
bool INV_MOT_L = false;                       // Invert left drive motor rotation
bool INV_MOT_R = false;                       // Invert right drive motor rotation
// Trim
byte TRIM_L = 100;                            // Left motors rotation value for trim // 0 = zero rotation, 100 = full rotation
byte TRIM_R = 100;                            // Right motors rotation value for trim // 0 = zero rotation, 100 = full rotation
// Channel mixing
bool CHMIX = true;                            // Defines are fwd/bwd(ch1) and left/right(ch2) channel signals mixed together
// ----------------- Weapon ESC / Servo ----------------
bool USE_BIDIR = false;                       // Use BiDirectional servo/weapon signal // true = signal width is from -100 to 100, false = 0 to 100
bool USE_ESC = false;                         // Is script using ESC or servo // true = For spinners, signal is sent continuously to ESC. false = For flippers and grabbers, signal is sent to servo only when value has been changed.
// Servo angles / Brushless ESC speed controlled by Servo library (0-180° // 90° = 50%, 180° = 100% etc.) // Weapon(normal mode): 0° = Stop, 1-180° = run // Servo/Weapon(BiDir): 0° = Left, 180° = Right, 90° = Middle.
byte SRV_ANG_IDLE = 0;                        // Idle angle // Has to be 0 when using ESC and NOT using BiDirectional signal!
byte SRV_ANG_MAX = 180;                       // Max angle  // Has to be bigger than 0 when using ESC and NOT using BiDirectional signal!
byte SRV_ANG_MIN = 0;                         // Min angle  // Used only in BiDirectional mode
// --------------------- Telemetry ---------------------
#define useTelemetry                true      // Use battery voltage monitoring // Set false if voltage divider is not attached, otherwise bot is sending random values to controller and it will launch low battery alarm
#define PIN_VDIV                    0         // Voltage divider pin
#define monitorVoltageInterval      5000      // Delay in ms // Do not change smaller than 15sec, ADC and bluetooth can't handle it!
// ----------------- Low voltage guard -----------------
#define useLowVoltageGuard          true      // Use battery low voltage guard // Stops drive- and weapon motors if low voltage limit is reached // Telemetry has to be enabled when used.
#define lowVoltageLimit             0         // Voltage monitors low level limit in percents (6.5V - 8.4V = 0 - 100%)
// ---------- Distance sensor / AI (Optional) ----------
#define useSensor                   false     // Define is optional VL6180X distance sensor used
#define triggerRange                20        // Distance sensors trigger range in millimeters
#define sensorScanFreq              20        // Sensor scan and weapon angle change interval in ms
// --------------------- Connection --------------------
#define timeoutDelay                1000      // Timeout delay // If packages are not received from controller after delay stop and disconnect bot from all controllers.
/******************************************************/
/* ----------------- PWM PARAMETERS ----------------- */
/******************************************************/
// Motordrivers
#if defined(BRUSHL)
#define MIN_MICROS_MD               500       // min. 500us
#define MAX_MICROS_MD               2500      // max. 2500us
#endif
// Servo // Corona CS-939MG 600/2300us // BristolBotBuilders 2S HV High Speed Metal Geared Antweight Servo 500/2500us // DMS-MG90 500/2500us
#define MIN_MICROS_SERVO            500       // min. 500us
#define MAX_MICROS_SERVO            2500      // max. 2500us
// Brushless ESC (Default BLHeli S Littlebee 20A values)
#define MIN_MICROS_ESC              1148      // min. 500us
#define MAX_MICROS_ESC              1832      // max. 2500us
/******************************************************/
/* -------------- PIN & PWM PARAMETERS -------------- */
/******************************************************
 Pin layouts, pwm parameters etc. below are designed for to either connect motor drivers listed in info directly into DFRobots ESP32-C3 or when using Rebuild Robotics - ESP32-C3 CombatBot Expansion Board (Prototype).
 Presets are not necessary to change if so, but be sure that they are right.
 If connecting HR8833 or TB6612 motor driver into ESP directly, use 10kOhm pull-down resistor in pin 6. This prevents motor from spinning in startup and upload!
******************************************************/
// -------------------- Drive motors -------------------
// PWM channel parameters for DC motor drivers (Using ESP LED PWM)
#if defined(HR8833) || defined(TB6612)
#define CH_PWM_IA1                  2         // Left motor
#define CH_PWM_IA2                  4         // Left motor
#define CH_PWM_IB1                  3         // Right motor
#define CH_PWM_IB2                  5         // Right motor
#define FREQ_PWM_MD                 500       // PWM frequency
#define RESO_PWM_MD                 8         // 8-bit resolution
#endif
// Output pins // Use 10kOhm pull-down resistor in pin 6 with HR8833 and TB6612!
#if defined(HR8833) || defined(TB6612)
#define PIN_MD_IB1                  4         // Right motor
#define PIN_MD_IB2                  1         // Right motor
#endif
#if defined(HR8833)
#define PIN_MD_IA1                  5         // Left motor
#define PIN_MD_IA2                  6        // Left motor
#elif defined(TB6612)
#define PIN_MD_IA1                  6         // Left motor
#define PIN_MD_IA2                  5         // Left motor
#elif defined(BRUSHL)
#define PIN_M1                      1         // Left motor
#define PIN_M2                      4         // Right motor
#endif
// ----------------- Weapon ESC / Servo ----------------
#define PIN_WEP                     7         // Weapon pin (Servo / Brushless ESC) // Also allocates PWM channel 0, this is done by ESP32Servo library.
// -------------------- OnBoard LED --------------------
#define PIN_LED_ONBOARD             10        // OnBoard led pin
#if defined(HR8833) || defined(TB6612)
#define CH_PWM_LED                  1         // PWM channel for OnBoard led pulse generation with DC motor setup (ESP32Servo library is reserving channel 0 for weapon and channels 2-5 for drive motors).
#elif defined(BRUSHL)
#define CH_PWM_LED                  3         // PWM channel for OnBoard led pulse generation with brushless setup (ESP32Servo library is reserving channels 0-2 for weapon and drive motors).
#endif
#define FREQ_PWM_LED                5000      // PWM frequency
#define RESO_PWM_LED                8         // 8-bit resolution
/******************************************************/
/* -------------------- BLUETOOTH ------------------- */
/******************************************************/
#define SERVICE_UUID                "6E400001-B5A3-F393-E0A9-E50E24DCCA9E" // UART service UUID
#define CHARACTERISTIC_UUID_RX      "6E400002-B5A3-F393-E0A9-E50E24DCCA9E"
#define CHARACTERISTIC_UUID_TX      "6E400003-B5A3-F393-E0A9-E50E24DCCA9E"
/******************************************************/
/* ------------------- END OF PRESETS --------------- */
/******************************************************/
// ---------------------- Libraries --------------------
// Eeprom
#include "EEPROM.h"
// BluetoothLE
#include <BLEDevice.h>
#include <BLEServer.h>
#include <BLEUtils.h>
#include <BLE2902.h>
// Servos
#include <ESP32Servo.h>
Servo weapon;
#if defined(BRUSHL)
Servo motorLeft;
Servo motorRight;
#endif
// Distance sensor
#if defined(useSensor)
#include <DFRobot_VL6180X.h>
DFRobot_VL6180X VL6180X;
#endif
// ---------------------- Values -----------------------
// OnBoard LED
byte ledDuty;
unsigned long ledBlinkDelay;
bool ledOn;
unsigned long ledToggled;
// EEPROM
#define EEPROM_SIZE 32  // Amount of bytes(=rows) in EEPROM. Name takes 20bytes, 1 byte is for eeprom itself and 10 is for presets.
byte varsCount;
// Telemetry
unsigned long voltageLevel;
unsigned long voltageMonitored;
unsigned long lowVoltageDetected;
byte cells; // 1 or 2 // 1S or 2S LiPo battery // Defined in setup from batteryvoltage
bool batteryEmpty = false;
bool lowVoltage;
// Bluetooth
BLEServer *pServer = NULL;
BLECharacteristic *pTxCharacteristic;
bool deviceConnected = false;
unsigned long lastUpdate;
// Channel values
int ch1;  // Drive channel: Forward/Backward
int ch2;  // Drive channel: Left/Right
int ch3;  // Weapon/Servo channel
int ch4;  // AI channel
// Previous channel values
int ch1Prev;
int ch2Prev;
int ch3Prev;
int ch4Prev;
// AI
bool AIActive = false;
// Weapon
bool weaponActive = false;
int angPrev = -1;
unsigned long angleSetTime;
// Distance sensor
uint8_t range;
// Failsafe
bool failsafeActive = true;
// ---------------------- Onboard led ------------------
void setLed(byte duty, int delay)
{
  ledDuty = duty;
  ledBlinkDelay = delay;
  ledcWriteChannel(CH_PWM_LED, ledDuty);
  (ledDuty > 0) ? ledOn = true : ledOn = false;
  if (ledBlinkDelay > 0) ledToggled = millis();
}

void blinkLed()
{
  if (ledBlinkDelay > 0 && (millis() - ledToggled) > ledBlinkDelay)
  {
    (ledOn) ? ledcWriteChannel(CH_PWM_LED, 0) : ledcWriteChannel(CH_PWM_LED, ledDuty);
    ledOn = !ledOn;
    ledToggled = millis();
  }
}
// ---------------- Parameters error check -------------
String errors;
void checkPresetErrors()
{
  // Name too long
  if (NAME.length() > 19)
    errors += "Warning: Name too long! Max. 19 characters\n";
  // Invalid motor driver type
  #if !defined(HR8833) && !defined(TB6612) && !defined(BRUSHL)
    errors += "Warning: Incorrect motor driver type!\n";
  #endif
  // Wrong cell count
  if (useTelemetry && cells == 0)
    errors += "Warning: Battery voltage not recognized!\n";
  // Low voltage guard - telemetry disabled
  if (!useTelemetry && useLowVoltageGuard)
    errors += "Warning: Telemetry has to be enabled when using Low voltage guard!\n";
  // Wrong weapon parameter
  if ((USE_ESC && !USE_BIDIR && SRV_ANG_IDLE != 0) || (USE_ESC && USE_BIDIR && (SRV_ANG_IDLE == 0 || SRV_ANG_IDLE == 180)))
    errors += "Warning: Incorrect weapon idle angle!\n";
  // Servo value out of range
  if (SRV_ANG_MIN < 0 || SRV_ANG_MIN > 180 || SRV_ANG_IDLE < 0 || SRV_ANG_IDLE > 180 || SRV_ANG_MAX < 0 || SRV_ANG_MAX > 180)
    errors += "Warning: Servo value out of range!\n";
  // Trim out of range
  if (TRIM_L < 0 || TRIM_L > 100 || TRIM_R < 0 || TRIM_R > 100)
    errors += "Warning: Trim level out of range!\n";
}

// ---------------------- Bluetooth --------------------
// Connect/disconnect
class MyServerCallbacks : public BLEServerCallbacks
{
  void onConnect(BLEServer *pServer)
  {
    if (DEBUG) Serial.println("Bluetooth connected");
    deviceConnected = true;
    setLed(50, 0);
    failsafe(false);
    if (DEBUG) printBotData();
  };

  void onDisconnect(BLEServer *pServer)
  {
    if (DEBUG) Serial.println("Bluetooth disconnected");
    deviceConnected = false;
    setLed(50, 1000);
    failsafe(true);
    if (DEBUG) printBotData();
    delay(500);
    pServer->startAdvertising();
  }
};
// Data receiving
class MyCallbacks : public BLECharacteristicCallbacks
{
  void onWrite(BLECharacteristic *pCharacteristic)
  {
    // Get BLEdata
    String BLEData = pCharacteristic->getValue();
    // Process BLEData
    if (BLEData != "")
    {
      // Incoming controls starting with number
      if (!failsafeActive && (isDigit(BLEData.charAt(0)) || isPunct(BLEData.charAt(0))))
      {
        // Unpack
        int ind1 = BLEData.indexOf(':');
        ch1 = BLEData.substring(0, ind1).toInt();
        int ind2 = BLEData.indexOf(':', ind1 + 1);
        ch2 = BLEData.substring(ind1 + 1).toInt();
        int ind3 = BLEData.indexOf(':', ind2 + 1);
        ch3 = BLEData.substring(ind2 + 1).toInt();
        int ind4 = BLEData.indexOf('\n', ind3 + 1);
        ch4 = BLEData.substring(ind3 + 1).toInt();
        // Mix driving channels & invert ch1 because of controller fwd is negative not positive
        int throttleLeft;   // Left motor speed
        int throttleRight;  // Right motor speed
        if (CHMIX)
        {
          throttleLeft = -ch1 + ch2; // Original: y + x
          throttleRight = -ch1 - ch2; // Original: y - x
        }
        else // No channel mixing
        {
          if (abs(ch1) >= abs(ch2)) // Forward/backward
          {
            throttleLeft = -ch1;
            throttleRight = -ch1;
            
          }
          else // Full turns
          {
            throttleLeft = ch2;
            throttleRight = -ch2;
          }
        }
        // Constrain
        throttleLeft = constrain(throttleLeft, -100, 100);
        throttleRight = constrain(throttleRight, -100, 100);
        // Trim
        throttleLeft = round(throttleLeft * ((float)TRIM_L / 100));
        throttleRight = round(throttleRight * ((float)TRIM_R / 100));
        // Convert to PWM
        #if defined(HR8833) || defined(TB6612)
          throttleLeft = map(throttleLeft, -100, 100, -255, 255);
          throttleRight = map(throttleRight, -100, 100, -255, 255);
          throttleLeft = constrain(throttleLeft, -255, 255);
          throttleRight = constrain(throttleRight, -255, 255);
        #endif
        // Convert to Angle & invert rotation
        #if defined(BRUSHL)
          throttleLeft = (INV_MOT_L) ? map(throttleLeft, -100, 100, 180, 0) : map(throttleLeft, -100, 100, 0, 180);
          throttleRight = (INV_MOT_R) ? map(throttleRight, -100, 100, 0, 180) : map(throttleRight, -100, 100, 180, 0);
          throttleLeft = constrain(throttleLeft, 0, 180);
          throttleRight = constrain(throttleRight, 0, 180);
        #endif
        // Run drive motors
        #if defined(HR8833) || defined(TB6612) || defined(BRUSHL)
          if (!batteryEmpty || (batteryEmpty && (ch1Prev != 0 || ch2Prev != 0)))
            drive(throttleLeft, throttleRight);
        #endif
        // Servo / Brushless Weapon ESC // Send continuous signal (ESC) or send signal only if value has been changed (Servo)
        if (
          (USE_ESC && !batteryEmpty && (ch3 != 0 || (ch3 == 0 && ch3Prev != 0))) ||
          (!USE_ESC && !batteryEmpty && ch3 != ch3Prev) ||
          (batteryEmpty && ch3Prev != 0)
        ) {
          // Convert weapon channel signal to servo position
          int srvAng;
          if (USE_BIDIR)
          {
            srvAng = map(ch3, -100, 100, SRV_ANG_MIN, SRV_ANG_MAX);
            srvAng = constrain(srvAng, (SRV_ANG_MIN < SRV_ANG_MAX) ? SRV_ANG_MIN : SRV_ANG_MAX, (SRV_ANG_MIN < SRV_ANG_MAX) ? SRV_ANG_MAX : SRV_ANG_MIN);
          }
          else
          {
            srvAng = map(ch3, 0, 100, SRV_ANG_IDLE, SRV_ANG_MAX);
            srvAng = constrain(srvAng, (SRV_ANG_IDLE < SRV_ANG_MAX) ? SRV_ANG_IDLE : SRV_ANG_MAX, (SRV_ANG_IDLE < SRV_ANG_MAX) ? SRV_ANG_MAX : SRV_ANG_IDLE);
          }
          // Set angle
          setWeaponAngle(srvAng);
        }
        // AI
        if (ch4 != ch4Prev)
        {
          if (ch4 == 0) // AI Off
            setAI(false);
          if (ch4 == 100 && !batteryEmpty) // AI On
            setAI(true);
        }
        // Data updated
        lastUpdate = millis();
      }
      // Incoming data starting with letter // Presets
      if (isAlpha(BLEData.charAt(0)))
      {
        // Send presets to controller when requested
        if (BLEData == "GET_PRESETS"){ sendPresets(); }
        // Get presets from controller one at time
        else
        {
          // Unpack
          int ind1 = BLEData.indexOf('=');
          String presetID = BLEData.substring(0, ind1);
          int ind2 = BLEData.indexOf('\n', ind1 + 1);
          int presetValue = BLEData.substring(ind1 + 1).toInt();
          // Update to variables
          if (presetID == "MIX")
          {
            CHMIX = presetValue;
            varsCount++;
          }
          else if (presetID == "BI")
          {
            USE_BIDIR = presetValue;
            varsCount++;
          }
          else if (presetID == "ESC")
          {
            USE_ESC = presetValue;
            varsCount++;
          }
          else if (presetID == "S_ID")
          {
            if (!USE_ESC || (USE_ESC && !USE_BIDIR && presetValue == 0) || (USE_ESC && USE_BIDIR))
              SRV_ANG_IDLE = constrain(presetValue, 0, 180);
            varsCount++;
          }
          else if (presetID == "S_MA")
          {
            SRV_ANG_MAX = constrain(presetValue, 0, 180);
            varsCount++;
          }
          else if (presetID == "S_MI")
          {
            SRV_ANG_MIN = constrain(presetValue, 0, 180);
            varsCount++;
          }
          else if (presetID == "I_L")
          {
            INV_MOT_L = presetValue;
            varsCount++;
          }
          else if (presetID == "I_R")
          {
            INV_MOT_R = presetValue;
            varsCount++;
          }
          else if (presetID == "T_L")
          {
            TRIM_L = constrain(presetValue, 0, 100);
            varsCount++;
          }
          else if (presetID == "T_R")
          {
            TRIM_R = constrain(presetValue, 0, 100);
            varsCount++;
          }
          else // Name
          {
            presetID.remove((presetID.length() - 1), 1);  // Remove \n from end of the string
            NAME = (presetID.length() > 19) ? presetID.substring(0, 19) : presetID;
            varsCount++;
          }
          // Save and confirm
          if (varsCount == 11)
          {
            // Update variables to EEPROM
            EEPROM_Save();
            varsCount = 0;
            // Update variables to controller
            sendPresets();
            (deviceConnected) ? setLed(50, 0) : setLed(50, 1000);
          }
        }
      }
      // Print debug data
      if (DEBUG) printBotData();
    }
  }
};
// ------------------------ Setup ----------------------
void setup()
{
  // Init eeprom
  EEPROM.begin(EEPROM_SIZE);  // Start EEPROM
  // Return data from EEPROM : Save default data to EEPROM if empty
  (EEPROM_Exists()) ? EEPROM_Load() : EEPROM_Save();
  // Set OnBoard led PWM
  #if defined(CH_PWM_LED)
    ledcAttachChannel(PIN_LED_ONBOARD, FREQ_PWM_LED, RESO_PWM_LED, CH_PWM_LED);
  #endif
  // Init BLE
  BLEBegin();
  // Allocate timers for servos
  ESP32PWM::allocateTimer(0);
  ESP32PWM::allocateTimer(1);
  ESP32PWM::allocateTimer(2);
  ESP32PWM::allocateTimer(3);
  // Init weapon
  weapon.setPeriodHertz(50);  // 50hz
  // Init brushless drive motors
  #if defined(BRUSHL)
    motorLeft.setPeriodHertz(50);
    motorRight.setPeriodHertz(50);
  #endif
  // Stop drive motors (Notice: At default ESP pin6 is HIGH in powerup and after that. This causes left motor to keep spinning until pin status has been changed. ESP powerup motor spinning can be prevented with pull-up/pull-down resistors if not using CombatBot Expansion Board.)
  stopMotors();
  // Disable Motor Driver output pins
  disableOutputs();
  // Start AI stopped
  setAI(false);
  // Init battery monitor
  if (useTelemetry)
  {
    // Voltage monitor // Uses voltage divider with 56kOhm resistor in GND and 100kOhm resistor in POS
    pinMode(PIN_VDIV, INPUT);
    analogSetPinAttenuation(PIN_VDIV, ADC_11db); // ESP32-C3 range 0-2.5V // https://docs.espressif.com/projects/arduino-esp32/en/latest/api/adc.html#analogsetattenuation
    // Init LiPo cell count from battery voltage (Using voltage divider)
    int mA = analogReadMilliVolts(PIN_VDIV);
    if (mA >= 1000 && mA < 1550) // 2,8 - 4,3 V
      cells = 1;
    else if (mA >= 2150 && mA <= 3100) // 6,0 - 8,65V
      cells = 2;
  }
  // Start debug mode
  if (DEBUG)
  {
    // Start serial monitor connection and wait connection
    Serial.begin(115200);
    while (!Serial);
    delay(10);
    // Add debug header text
    Serial.printf("--------------------------------------\nCombatBot for Beetle ESP32-C3 V.%s\n----------- [ DEBUG MODE ] -----------\n", version);
  }
  // Check parameter errors
  checkPresetErrors();
  // Init distance sensor
  if (useSensor && !(VL6180X.begin()))
  {
    if (DEBUG) errors += "Warning: Distance sensor not found!\n";
  }
  // Print data
  if (DEBUG)
  {
    if (errors == ""){ printBotData(); }
    else
    {
      Serial.println(errors);
      Serial.println("--------- [ Setup failed ] ---------");
    }
  }
  // Indicate parameter errors
  while (errors != "")
    (ledBlinkDelay > 0) ? blinkLed() : setLed(255, 100);  // Blink led and do not continue if errors found
  // Add debug footer text
  if (DEBUG)
    Serial.println("--------- [ Setup complete ] ---------");
  // Set led indication to standby
  setLed(50, 1000);
}
// ---------------------- Main loop --------------------
void loop()
{
  // Check has controller data been received and stop after delay if not
  runWatchdog();
  // Onboard led blink
  blinkLed();
  // Voltage monitoring
  if (useTelemetry && millis() - voltageMonitored >= monitorVoltageInterval)
    monitorVoltage();
  // Run AI
  if (deviceConnected && AIActive && !batteryEmpty)
    runAI();
  // Slow down loop, otherwise runWatchdog() check will fail
  delay(1);
}
// ---------------------- Functions --------------------
// Init BLE // Based on the BLE_server, BLE_client examples coming in https://github.com/espressif/arduino-esp32 library.
void BLEBegin()
{
  // Create the BLE Device
  String stdStr(NAME.c_str(), NAME.length());
  BLEDevice::init(stdStr);
  // Create the BLE Server
  pServer = BLEDevice::createServer();
  pServer->setCallbacks(new MyServerCallbacks());
  // Create the BLE Service
  BLEService *pService = pServer->createService(SERVICE_UUID);
  // Create a BLE Characteristic
  BLECharacteristic *pRxCharacteristic = pService->createCharacteristic(
    CHARACTERISTIC_UUID_RX,
    BLECharacteristic::PROPERTY_WRITE);
  pRxCharacteristic->setCallbacks(new MyCallbacks());
  pTxCharacteristic = pService->createCharacteristic(
    CHARACTERISTIC_UUID_TX,
    BLECharacteristic::PROPERTY_NOTIFY | BLECharacteristic::PROPERTY_READ);
  pTxCharacteristic->addDescriptor(new BLE2902());
  // Start the service
  pService->start();
  // Start advertising
  BLEAdvertising *pAdvertising = BLEDevice::getAdvertising();
  pAdvertising->addServiceUUID(SERVICE_UUID);
  pAdvertising->setScanResponse(true);
  BLEDevice::startAdvertising();
  if (DEBUG) Serial.println("Waiting client ...");
}

void EEPROM_Save()
{
  EEPROM.write(0, 1);  // Boolean for EEPROM_Exists()
  EEPROM.write(1, CHMIX);
  EEPROM.write(2, USE_BIDIR);
  EEPROM.write(3, USE_ESC);
  EEPROM.write(4, SRV_ANG_IDLE);
  EEPROM.write(5, SRV_ANG_MAX);
  EEPROM.write(6, SRV_ANG_MIN);
  EEPROM.write(7, INV_MOT_L);
  EEPROM.write(8, INV_MOT_R);
  EEPROM.write(9, TRIM_L);
  EEPROM.write(10, TRIM_R);
  EEPROM_writeString(11, NAME);
  EEPROM.commit();
}

void EEPROM_Load()
{
  CHMIX = EEPROM.read(1);
  USE_BIDIR = EEPROM.read(2);
  USE_ESC = EEPROM.read(3);
  SRV_ANG_IDLE = EEPROM.read(4);
  SRV_ANG_MAX = EEPROM.read(5);
  SRV_ANG_MIN = EEPROM.read(6);
  INV_MOT_L = EEPROM.read(7);
  INV_MOT_R = EEPROM.read(8);
  TRIM_L = EEPROM.read(9);
  TRIM_R = EEPROM.read(10);
  NAME = EEPROM_readString(11);
}

bool EEPROM_Exists()
{
  if (EEPROM.read(0) == 1) { return true; }
  else { return false; }
}

void EEPROM_writeString(int addr, const String &str) // Based on example: https://stackoverflow.com/questions/70531251/error-while-writing-retrieving-string-in-eeprom
{
  byte lenght = str.length();
  EEPROM.write(addr, lenght);
  for (int i = 0; i < lenght; i++)
    EEPROM.write(addr + 1 + i, str[i]);
  EEPROM.commit();
}

String EEPROM_readString(int addr) // Based on example: https://stackoverflow.com/questions/70531251/error-while-writing-retrieving-string-in-eeprom
{
  int strLenght = EEPROM.read(addr);
  char data[strLenght + 1];
  for (int i = 0; i < strLenght; i++)
    data[i] = EEPROM.read(addr + 1 + i);
  data[strLenght] = '\0';
  return String(data);
}

void sendPresets()
{
  pTxCharacteristic->setValue((String) "MIX=" + (byte)CHMIX + ":\n"); // Set
  pTxCharacteristic->notify(); // Send
  delay(10);
  pTxCharacteristic->setValue((String) "BI=" + (byte)USE_BIDIR + ":\n");
  pTxCharacteristic->notify();
  delay(10);
  pTxCharacteristic->setValue((String) "ESC=" + (byte)USE_ESC + ":\n");
  pTxCharacteristic->notify();
  delay(10);
  pTxCharacteristic->setValue((String) "S_ID=" + SRV_ANG_IDLE + ":\n");
  pTxCharacteristic->notify();
  delay(10);
  pTxCharacteristic->setValue((String) "S_MA=" + SRV_ANG_MAX + ":\n");
  pTxCharacteristic->notify();
  delay(10);
  pTxCharacteristic->setValue((String) "S_MI=" + SRV_ANG_MIN + ":\n");
  pTxCharacteristic->notify();
  delay(10);
  pTxCharacteristic->setValue((String) "I_L=" + (byte)INV_MOT_L + ":\n");
  pTxCharacteristic->notify();
  delay(10);
  pTxCharacteristic->setValue((String) "I_R=" + (byte)INV_MOT_R + ":\n");
  pTxCharacteristic->notify();
  delay(10);
  pTxCharacteristic->setValue((String) "T_L=" + (byte)TRIM_L + ":\n");
  pTxCharacteristic->notify();
  delay(10);
  pTxCharacteristic->setValue((String) "T_R=" + (byte)TRIM_R + ":\n");
  pTxCharacteristic->notify();
  delay(10);
}

void drive(int spdL, int spdR)
{
  // Fermion: TB6612FNG
  #if defined(TB6612)
    // Left motor direction
    if (spdL > 0) // Forward
    {
      (INV_MOT_L) ? digitalWrite(PIN_MD_IA2, 1) : digitalWrite(PIN_MD_IA2, 0);
    }
    else if (spdL < 0) // Backward
    {
      (INV_MOT_L) ? digitalWrite(PIN_MD_IA2, 0) : digitalWrite(PIN_MD_IA2, 1);
    }
    else // Brake
    {
      digitalWrite(PIN_MD_IA2, 1);
    }
    // Right motor direction
    if (spdR > 0) // Forward
    {
      (INV_MOT_R) ? digitalWrite(PIN_MD_IB2, 1) : digitalWrite(PIN_MD_IB2, 0);
    }
    else if (spdR < 0) // Backward
    {
      (INV_MOT_R) ? digitalWrite(PIN_MD_IB2, 0) : digitalWrite(PIN_MD_IB2, 1);
    }
    else // Brake
    {
      digitalWrite(PIN_MD_IB2, 1);
    }
    // Speed
    ledcWriteChannel(CH_PWM_IA1, abs(spdL));
    ledcWriteChannel(CH_PWM_IB1, abs(spdR));
  #endif
  // Fermion: HR8833
  #if defined(HR8833)
    // Left motor direction
    if (spdL > 0) // Forward
    {
      if (INV_MOT_L)
      {
        ledcWriteChannel(CH_PWM_IA1, 255 - abs(spdL));
        ledcWriteChannel(CH_PWM_IA2, 255);
      }
      else
      {
        ledcWriteChannel(CH_PWM_IA2, 255 - abs(spdL));
        ledcWriteChannel(CH_PWM_IA1, 255);
      }
    }
    else if (spdL < 0) // Backward
    {
      if (INV_MOT_L)
      {
        ledcWriteChannel(CH_PWM_IA2, 255 - abs(spdL));
        ledcWriteChannel(CH_PWM_IA1, 255);
      }
      else
      {
        ledcWriteChannel(CH_PWM_IA1, 255 - abs(spdL));
        ledcWriteChannel(CH_PWM_IA2, 255);
      }
    }
    else // Brake
    {
      ledcWriteChannel(CH_PWM_IA1, 255);
      ledcWriteChannel(CH_PWM_IA2, 255);
    }
    // Right motor direction
    if (spdR > 0) // Forward
    {
      if (INV_MOT_R)
      {
        ledcWriteChannel(CH_PWM_IB1, 255 - abs(spdR));
        ledcWriteChannel(CH_PWM_IB2, 255);
      }
      else
      {
        ledcWriteChannel(CH_PWM_IB2, 255 - abs(spdR));
        ledcWriteChannel(CH_PWM_IB1, 255);
      }
    }
    else if (spdR < 0) // Backward
    {
      if (INV_MOT_R)
      {
        ledcWriteChannel(CH_PWM_IB2, 255 - abs(spdR));
        ledcWriteChannel(CH_PWM_IB1, 255);
      }
      else
      {
        ledcWriteChannel(CH_PWM_IB1, 255 - abs(spdR));
        ledcWriteChannel(CH_PWM_IB2, 255);
      }
    }
    else // Brake
    {
      ledcWriteChannel(CH_PWM_IB1, 255);
      ledcWriteChannel(CH_PWM_IB2, 255);
    }
  #endif
  // Brushless motors
  #if defined(BRUSHL)
    motorLeft.write(spdL);
    motorRight.write(spdR);
  #endif
  // Previous values
  ch1Prev = ch1;
  ch2Prev = ch2;
}

void setWeaponAngle(int angle)
{
  // Rotate servo
  weapon.write(angle);
  // Change build in led state and send confirm to controller if channel value has been changed
  if (angle == SRV_ANG_IDLE)
  {
    (deviceConnected) ? setLed(50, 0) : setLed(50, 1000);
    weaponActive = false;
    pTxCharacteristic->setValue("WEP=0:\n");
    pTxCharacteristic->notify();
  }
  else
  {
    setLed(255, 0);
    weaponActive = true;
    pTxCharacteristic->setValue("WEP=1:\n");
    pTxCharacteristic->notify();
  }
  ch3Prev = ch3;
  angPrev = angle;
  angleSetTime = millis();
}

void enableOutputs()
{
  // Drive Motors failsafe
  #if defined(HR8833)
    // Attach Motor Driver pins to pwm channels
    ledcAttachChannel(PIN_MD_IA1, FREQ_PWM_MD, RESO_PWM_MD, CH_PWM_IA1);
    ledcAttachChannel(PIN_MD_IA2, FREQ_PWM_MD, RESO_PWM_MD, CH_PWM_IA2);
    ledcAttachChannel(PIN_MD_IB1, FREQ_PWM_MD, RESO_PWM_MD, CH_PWM_IB1);
    ledcAttachChannel(PIN_MD_IB2, FREQ_PWM_MD, RESO_PWM_MD, CH_PWM_IB2);
  #endif
  #if defined(TB6612)
    // Attach Motor Driver pins to pwm channels
    ledcAttachChannel(PIN_MD_IA1, FREQ_PWM_MD, RESO_PWM_MD, CH_PWM_IA1);
    ledcAttachChannel(PIN_MD_IB1, FREQ_PWM_MD, RESO_PWM_MD, CH_PWM_IB1);
    // Set direction pins as output
    pinMode(PIN_MD_IA2, OUTPUT);
    pinMode(PIN_MD_IB2, OUTPUT);
  #endif
  #if defined(BRUSHL)
    // Enable motor signals
    motorLeft.attach(PIN_M1, MIN_MICROS_MD, MAX_MICROS_MD);
    motorRight.attach(PIN_M2, MIN_MICROS_MD, MAX_MICROS_MD);
  #endif
  // Enable weapon signal
  weapon.attach(PIN_WEP, (USE_ESC) ? MIN_MICROS_ESC : MIN_MICROS_SERVO, (USE_ESC) ? MAX_MICROS_ESC : MAX_MICROS_SERVO);
}

void disableOutputs()
{
  // Drive Motors failsafe
  #if defined(HR8833)
    // Detach Motor Driver pins from pwm channels
    ledcDetach(PIN_MD_IA1);
    ledcDetach(PIN_MD_IA2);
    ledcDetach(PIN_MD_IB1);
    ledcDetach(PIN_MD_IB2);
    // Set pins as input to stop motors from rotating
    pinMode(PIN_MD_IA1, INPUT);
    pinMode(PIN_MD_IB1, INPUT);
    pinMode(PIN_MD_IA2, INPUT);
    pinMode(PIN_MD_IB2, INPUT);
  #endif
  #if defined(TB6612)
    // Detach Motor Driver pins from pwm channels
    ledcDetach(PIN_MD_IA1);
    ledcDetach(PIN_MD_IB1);
    // Set pins as input to stop motors from rotating
    pinMode(PIN_MD_IA1, INPUT);
    pinMode(PIN_MD_IB1, INPUT);
    pinMode(PIN_MD_IA2, INPUT);
    pinMode(PIN_MD_IB2, INPUT);
  #endif
  #if defined(BRUSHL)
    motorLeft.detach();
    motorRight.detach();
  #endif
  // Disable weapon signal
  weapon.detach();
}

void stopMotors()
{
  // Fermion: TB6612FNG
  #if defined(TB6612)
    ledcWriteChannel(CH_PWM_IA1, 0);
    ledcWriteChannel(CH_PWM_IB1, 0);
    digitalWrite(PIN_MD_IA2, 1);
    digitalWrite(PIN_MD_IB2, 1);
  #endif
  // Fermion: HR8833
  #if defined(HR8833)
    ledcWriteChannel(CH_PWM_IA1, 255);
    ledcWriteChannel(CH_PWM_IA2, 255);
    ledcWriteChannel(CH_PWM_IB1, 255);
    ledcWriteChannel(CH_PWM_IB2, 255);
  #endif
  // Brushless motors
  #if defined(BRUSHL)
    motorLeft.write(90);
    motorRight.write(90);
  #endif
}

void resetChannels()
{
  ch1 = 0;
  ch2 = 0;
  ch3 = 0;
  ch4 = 0;
  ch1Prev = ch1;
  ch2Prev = ch2;
  ch3Prev = ch3;
  ch4Prev = ch4;
}

void failsafe(bool active)
{
  if (active) // Enable, stops all
  {
    stopMotors();
    if (USE_ESC) setWeaponAngle(SRV_ANG_IDLE);
    setAI(false);
    disableOutputs();
    resetChannels();
    failsafeActive = true;
  }
  else // Disable, allow all
  {
    enableOutputs();
    failsafeActive = false;
  }
}

void monitorVoltage()
{
  // Turn battery voltage into percents // Using voltage divider in PIN 0 with 100kOhm resistor in POS and 56kOhm in GND
  voltageLevel = analogReadMilliVolts(PIN_VDIV);  // https://docs.espressif.com/projects/arduino-esp32/en/latest/api/adc.html
  if (cells == 1)
    voltageLevel = map(voltageLevel, 1150, 1500, 0, 100);
  else if (cells == 2)
    voltageLevel = map(voltageLevel, 2350, 3000, 0, 100);
  else
    voltageLevel = 0;
  voltageLevel = constrain(voltageLevel, 0, 100);
  if (deviceConnected)
  {
    pTxCharacteristic->setValue((String) "BAT=" + voltageLevel + ":\n");
    pTxCharacteristic->notify();
  }
  // Low voltage guard
  if (useLowVoltageGuard)
  {
    if (voltageLevel <= lowVoltageLimit && !lowVoltage)
    {
      lowVoltageDetected = millis();
      lowVoltage = true;
    }
    else if (voltageLevel > lowVoltageLimit && lowVoltage)
    {
      lowVoltage = false;
    }
    if (lowVoltage && millis() - lowVoltageDetected > 10000 && !batteryEmpty)
    {
      batteryEmpty = true;
      failsafe(true);
    }
    else if (!lowVoltage && batteryEmpty)
    {
      batteryEmpty = false;
      if (deviceConnected) failsafe(false);
    }
  }
  voltageMonitored = millis();
}

void setAI(bool active)
{
  AIActive = active;
  // Send confirm to controller
  (AIActive) ? pTxCharacteristic->setValue("AI=1:\n") : pTxCharacteristic->setValue("AI=0:\n");
  pTxCharacteristic->notify();
  ch4Prev = ch4;
}

// Custom ch4 channel slot, designed for to run optional AI. Uses signal from 0 to 100
void runAI()
{
  // Distance sensor for semiautomatic flipper
  if (useSensor)
  {
    if (millis() - angleSetTime >= sensorScanFreq) // Trigger after delay
    {
      range = VL6180X.rangePollMeasurement();
      if (range > 0 && range < 255 && range <= triggerRange && !weaponActive) // Set weapon to max angle
        setWeaponAngle(SRV_ANG_MAX);
      else if ((range == 0 || range >= 255 || range > triggerRange) && weaponActive) // Return to idle position
        setWeaponAngle(SRV_ANG_IDLE);
      else
        angleSetTime = millis();
    }
    if (DEBUG) // Print errors
    {
      uint8_t status = VL6180X.getRangeResult();
      switch (status)
      {
        case VL6180X_EARLY_CONV_ERR:
          errors += "RANGE ERR: ECE check failed!\n";
          break;
        case VL6180X_MAX_CONV_ERR:
          errors += "RANGE ERR: System did not converge before the specified max!\n";
          break;
        case VL6180X_IGNORE_ERR:
          errors += "RANGE ERR: Ignore threshold check failed!\n";
          break;
        case VL6180X_MAX_S_N_ERR:
          errors += "RANGE ERR: Measurement invalidated!\n";
          break;
        case VL6180X_RAW_Range_UNDERFLOW_ERR:
          errors += "RANGE ERR: RESULT_RANGE_RAW < 0!\n";
          break;
        case VL6180X_RAW_Range_OVERFLOW_ERR:
          errors += "RESULT_RANGE_RAW is out of range!\n";
          break;
        case VL6180X_Range_UNDERFLOW_ERR:
          errors += "RANGE ERR: RESULT__RANGE_VAL < 0!\n";
          break;
        case VL6180X_Range_OVERFLOW_ERR:
          errors += "RANGE ERR: RESULT__RANGE_VAL is out of range!\n";
          break;
        default:
          "RANGE ERR: System err!\n";
          break;
      }
      delay(1000);
    }
  }
}

void printBotData()
{
  Serial.printf(
    "BOT: Battery: %d%%, Distance: %dmm, Weapon: %d, Wep.angle: %d, AI: %d, TRIM_L: %d, TRIM_R: %d\n",
    voltageLevel,
    range,
    weaponActive,
    angPrev,
    AIActive,
    TRIM_L,
    TRIM_R);
}

void runWatchdog()
{
  if ((ch1 != 0 || ch2 != 0 || ch3 != 0 || ch4 != 0) && (millis() - lastUpdate) > timeoutDelay)
  {
    failsafe(true);
    if (DEBUG) Serial.println("[!] No data received");
  }
}