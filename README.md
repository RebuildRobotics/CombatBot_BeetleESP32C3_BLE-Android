****************************************
# CombatBot_BeetleESP32C3_BLE-Android V.1.4.1
****************************************

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
    - Brushless motors:   2 x PPM outputs
  - Weapon ESC or servo:  1 x PPM output
  - PPM signals are generated with servo library.
  - If using brushless ESC's they has to be configured separately!
  
  *****
  HOW TO INSTALL:
  *****
  1. Installing Arduino IDE, Board manager and libraries:
      1. Install Arduino IDE from https://www.arduino.cc/en/software. It is the main program what you will use to manage this script and connection to ESP.
      2. Install board manager by following guide in wiki: https://wiki.dfrobot.com/SKU_DFR0868_Beetle_ESP32_C3 .
      3. Install proper Arduino library versions mentioned below.
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
  - Board:                    esp32 by Espressif Systems v.3.2.0  Install from Arduino IDE      (Uses ESP32 core 3.x)
  - Servo/Weapon control:     ESP32Servo v.3.0.6                  Install from Arduino IDE
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
  - Basic presets can be changed from controller or from script.
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
