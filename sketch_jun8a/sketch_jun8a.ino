#include <Keypad.h>
#include "Wire.h"
#include <AccelStepper.h>
#include <LiquidCrystal_I2C.h>
#include "I2C_eeprom.h"
#include <string.h>
#include <backup.h> // Include the backup header for STM32
//#include <LiquidCrystal.h>
// Limit switches
#define LIMIT_SWITCH_HOME PB12
#define LIMIT_SWITCH_YA PB13
#define LIMIT_SWITCH_YB PB14

// Stepper motors setup
#define STEP_PIN_Y PB11
#define DIR_PIN_Y PC13
#define ENABLE_PIN_Y PB15

#define STEP_PIN_X PB10
#define DIR_PIN_X PB1
#define ENABLE_PIN_X PB0
AccelStepper stepperX(AccelStepper::DRIVER, STEP_PIN_X, DIR_PIN_X);

// Keypad setup
const byte ROWS = 4;  //four rows
const byte COLS = 4;  //four columns
char keys[ROWS][COLS] = {
  { '1', '2', '3', 'A' },
  { '4', '5', '6', 'B' },
  { '7', '8', '9', 'C' },
  { '*', '0', '#', 'D' }
};
byte colPins[ROWS] = { PA3, PA2, PA1, PA0 };  // Connect to the row pinouts of the keypad
byte rowPins[COLS] = { PA7, PA6, PA5, PA4 };   // Connect to the column pinouts of the keypad
Keypad kpd = Keypad(makeKeymap(keys), rowPins, colPins, ROWS, COLS);

unsigned long loopCount;
unsigned long startTime;
String msg;



I2C_eeprom ee(0x50, I2C_DEVICESIZE_24LC32);
#define startAddress 0
int menuValue[8] = { -1, -1, -1, -1, -1, -1, -1, -1 };

//Power Failure setup
#define powerFailurePin  PA10
#define POWER_STAT_ADDRESS 100
bool hasPowerFailed = false;
unsigned long lastTime;
static int count = 500;
unsigned long keyPressTime = 0;
#define longPressDuration = 1000UL  // 1 second for long press
unsigned long lastDisplayTime = 0;
static int speed = 1;
//float accl;
static bool hommingAllowed = false;
static bool enable = false;
static bool setZero = false;
long lastXposition;
static bool dir = true;
static bool dirHold = false;
static bool play = false;
static bool pause = false;
// Menu items
const char* menuItems[] PROGMEM= {
  "X_speed",
  "X_Accl",
  "Y_speed",
  "Y_Accl",
  "Zero_Pos",
  "Pitch_Dist",
  "No_Of_Cuts",
};
const int menuItemCount = sizeof(menuItems) / sizeof(menuItems[0]);
int topMenuItem = 0;     // Index of the top menu item currently displayed
int selectedItem = 0;    // Index of the selected item in the menu
int cursorPosition = 0;  // Position of the cursor on the screen (0-3)
int menuItemIndex;
bool numlock = false;
String inputBuffer = "";

enum screenPage {
  SCREEN_PROGRM,
  SCREEN_RUN,
  SCREEN_JOG
};
enum State {
  STATE_XSPEED,
  STATE_XACCELERATION,
  STATE_YSPEED,
  STATE_YACCELERATION,
  STATE_ZEROPOS,
  STATE_PITCH,
  STATE_CUTS,
  STATE_SAVE
};
screenPage currentScreen = SCREEN_JOG;
State currentState = STATE_XSPEED;

// LCD setup
LiquidCrystal_I2C lcd(0x27, 20, 4);
//const int rs = PA11, en = PA12, d4 = PA15, d5 = PB3, d6 = PB4, d7 = PB11;
//LiquidCrystal lcd(PA11, PA12, PA15, PB3, PB4, PB11);

byte downArrow[] = {
  B00100,
  B00100,
  B00100,
  B00100,
  B00100,
  B11111,
  B01110,
  B00100
};
byte upArrow[]  = {
  B00100,
  B01110,
  B11111,
  B00100,
  B00100,
  B00100,
  B00100,
  B00100
};
byte rightArrow[]  = {
  B00000,
  B00100,
  B00010,
  B11111,
  B00010,
  B00100,
  B00000,
  B00000
};
byte leftArrow[] = {
  B00000,
  B00100,
  B01000,
  B11111,
  B01000,
  B00100,
  B00000,
  B00000
};
byte homeSymbol[]  = {
  B00000,
  B00100,
  B01110,
  B11111,
  B01110,
  B01110,
  B01110,
  B00000
};
byte pauseSymbol[] = {
  B00000,
  B00100,
  B00110,
  B00111,
  B00110,
  B00100,
  B00000,
  B00000
};
byte playSymbol[] = {
  B00000,
  B01010,
  B01010,
  B01010,
  B01010,
  B01010,
  B00000,
  B00000
};
byte cancelSymbol[] = {
  B00000,
  B10001,
  B01010,
  B00100,
  B01010,
  B10001,
  B00000,
  B00000
};
enum symbol {
  DOWN_SYMBOL,
  UP_SYMBOL,
  RIGHT_SYMBOL,
  LEFT_SYMBOL,
  HOME_SYMBOL,
  PLAY_SYMBOL,
  PAUSE_SYMBOL,
  CANCEL_SYMBOL
};
enum motorStatus {
  NO_MOVING,
  MOVING_TO_HOME,
  MOVING_TO_EDGE,
  AT_HOME,
  AT_EDGE
};
enum run {
  GO_HOME,
  GO_ZERO,
  MOVE_Y_TO_EDGE,
  MOVE_Y_TO_HOME,
  INCREMENT_COUNTER,
  FINISH,
};
int currentCut = 0;
run runningState = GO_HOME;
run pausedState = GO_HOME;
motorStatus currntXMotorStatus = NO_MOVING;
motorStatus currenYMotorStatus = NO_MOVING;
void setup() {
  ////Serial.begin(115200);
  ///////////////// LCD INIT and creating custom characters here///////////////
  lcd.init();
  lcd.clear();
  lcd.backlight();
  lcd.createChar(UP_SYMBOL, upArrow);
  lcd.createChar(DOWN_SYMBOL, downArrow);
  lcd.createChar(RIGHT_SYMBOL, rightArrow);
  lcd.createChar(LEFT_SYMBOL, leftArrow);
  lcd.createChar(HOME_SYMBOL, homeSymbol);
  lcd.createChar(PLAY_SYMBOL, playSymbol );
  lcd.createChar(PAUSE_SYMBOL, pauseSymbol);
  lcd.createChar(CANCEL_SYMBOL, cancelSymbol);

  lcd.clear();
  lcd.setCursor(0, 1);
  lcd.print(F(">>>>>Welcome to<<<<<"));
  delay(500);
  lcd.setCursor(0, 2);
  lcd.print("Electrogoal Machines");
  delay(500);
  lcd.clear();
  
  //------------- LCD INIT and creating custom characters here--------------//
  ////////////////// EEPOM init and checking and reading value///////////////
  //Wire.begin();
  ee.begin();
  if (!ee.isConnected()) {
    //Serial.println(F("ERROR: Can't find eeprom (stopped)..."));
    lcd.print(F("ERROR EEPROM"));
    while (1);
  }
  readIntArrayFromEEPROM(menuValue, startAddress);
  //--------- EEPOM init and checking and reading value-----------------//
  /////////////////////////// RTC BEGIN and chck if working//////////////////
    // Initialize RTC and backup memory
  enableBackupDomain();
//  uint16_t data = readBackupRegister(BKP_DR1);
  //------------------- RTC BEGIN and chck if working---------------------//
  ///////////////setting the limit push button //////////////////////////
  pinMode(LIMIT_SWITCH_HOME, INPUT_PULLUP);
  pinMode(LIMIT_SWITCH_YA, INPUT_PULLUP);
  pinMode(LIMIT_SWITCH_YB, INPUT_PULLUP);
  //------------setting the limit push button --------------------------//.

  /////////// setting stepper motor x with configuration /////////////////
  stepperX.setMaxSpeed(menuValue[STATE_XSPEED]);
  stepperX.setAcceleration(menuValue[STATE_XACCELERATION]);
  stepperX.setEnablePin(ENABLE_PIN_X);
  stepperX.setPinsInverted(false, false, true);
  stepperX.enableOutputs();
  //----------setting stepper motor x with configuration------------------//

  ////////// setting the motor Y with configuration ///////////////////////
  pinMode(ENABLE_PIN_Y, OUTPUT);
  pinMode(STEP_PIN_Y, OUTPUT);
  pinMode(DIR_PIN_Y, OUTPUT);
  digitalWrite(ENABLE_PIN_Y, LOW);
  digitalWrite(DIR_PIN_Y, LOW);
  //--------- setting the motor Y with configuration ---------------------//

  ////////////////////HOMING MOTORS///////////////////////////////////////
  lcd.clear();
  lcd.setCursor(0, 0);
  lcd.print("Homming Y Motor:");
  hommingY();
  lcd.print("Done");
  lcd.setCursor(0, 1);
  lcd.print("Homming X Motor:");
  hommingX();
  //-------------------HOMING MOTORS-------------------------------------//

  /////////////////checking if electricity  has gone//////////////////////
  
  int  data = getBackupRegister(LL_RTC_BKP_DR1);
  if (data != 0) {
    hasPowerFailed = true;
    lcd.clear();
    lcd.print("PowerFailed before");
    lcd.setCursor(0,1);
    lcd.print("last cutting:");
    lcd.print(data);
    lcd.setCursor(0, 2);
    lcd.print("continue");
    lcd.setCursor(14, 2);
    lcd.print("cancel");
    
    
    lcd.setCursor(0, 3);
    lcd.print("press A");
    lcd.setCursor(13, 3);
    lcd.print("press D");
  
    while(hasPowerFailed){
      char button = kpd.getKey();
      if(button == 'A'){
        stepperX.setMaxSpeed(menuValue[STATE_XSPEED]);
        stepperX.setAcceleration(menuValue[STATE_XACCELERATION]);
        stepperX.moveTo((data * menuValue[STATE_PITCH])+menuValue[STATE_ZEROPOS]);
        currntXMotorStatus = MOVING_TO_EDGE;
      }
      if(button == 'D'){
        hasPowerFailed = false;
        currentCut = 0;
        setBackupRegister(LL_RTC_BKP_DR1,0);
      }
      stepperX.run();
      if(stepperX.distanceToGo()==0 && currntXMotorStatus == MOVING_TO_EDGE){
        currntXMotorStatus = NO_MOVING;
        currentScreen = SCREEN_RUN;
        play = true;
        pause = false;
        runningState = MOVE_Y_TO_EDGE;
        currenYMotorStatus = AT_HOME;
        currentCut =  data;
        hasPowerFailed = false;
        }
    }
    //delay(1000);
  } else {
   // hasPowerFailed = false;
    lcd.clear();
    lcd.print("no powerFailure before");
    delay(1000);
  }
  
  kpd.addEventListener(keypadEvent);
  lcd.clear();lcd.setCursor(0, 0);lcd.print("currntCut");lcd.print(currentCut);
  delay(1000);
  lcd.clear();
  attachInterrupt(digitalPinToInterrupt(LIMIT_SWITCH_HOME), homeLimitSwitch, FALLING);
  updateScreen('\0');
}

bool homeButtonTriggered = false;
bool scalling = false;
void homeLimitSwitch() {
  if (hommingAllowed) {
    homeButtonTriggered = true;
    
  }
}
void loop() {

  char key = kpd.getKey();
  if (key) {
    //Serial.println(key);
  }
  if(currentScreen != SCREEN_RUN){
  if (currntXMotorStatus == MOVING_TO_EDGE && !numlock) {
    stepperX.moveTo(stepperX.currentPosition() + speed);
    stepperX.run();
  } else if (currntXMotorStatus == MOVING_TO_HOME && !numlock) {
    stepperX.moveTo(stepperX.currentPosition() - speed);
    stepperX.run();
  } else if (currntXMotorStatus == MOVING_TO_EDGE && numlock) {
    if (stepperX.distanceToGo() == 0) {
      currntXMotorStatus = NO_MOVING;
      updateScreen('\0');
    }
    stepperX.run();
  } else if (currntXMotorStatus == MOVING_TO_HOME && numlock) {
    if (stepperX.distanceToGo() == 0) {
      currntXMotorStatus = NO_MOVING;
      updateScreen('\0');
    }
    stepperX.run();
  }




  if (hommingAllowed && digitalRead(LIMIT_SWITCH_HOME) == LOW) {
    hommingX();
    hommingAllowed = false;
    homeButtonTriggered = false;
  }
 

  if (scalling && millis() - lastTime > count) {
    count = count - 5;
    speed = speed + 3;
    if (speed > 1500) {
      speed = 1500;
      scalling = false;
    }
    if (count <= 0) {
      scalling = false;
    }
    lastTime = millis();
  }
  // if(millis()-lastDisplayTime > 1000){

  //   updateScreen(key);
  //   lastDisplayTime = millis();
  // }
  if (currenYMotorStatus == MOVING_TO_HOME) {
    if (stateYa() == HIGH) {
      digitalWrite(DIR_PIN_Y, 0);
      analogWriteFrequency(menuValue[STATE_YSPEED]);
      analogWrite(STEP_PIN_Y, 0);
      currenYMotorStatus = AT_HOME;
    }
  }
  if (currenYMotorStatus == MOVING_TO_EDGE) {
    if (stateYb() == HIGH) {
      digitalWrite(DIR_PIN_Y, 0);
      analogWriteFrequency(menuValue[STATE_YSPEED]);
      analogWrite(STEP_PIN_Y, 0);
      currenYMotorStatus = AT_EDGE;
    }
  }

}
  ////Serial.println(currenYMotorStatus);
  if (currentScreen == SCREEN_RUN) {
    if (play) {
      switch (runningState) {
        case GO_HOME:
          ////Serial.println("GOHOME");
          updateScreen(key);
          currenYMotorStatus = MOVING_TO_HOME;
          
          updateScreen(key);
          hommingY();
          currenYMotorStatus = AT_HOME;
          currntXMotorStatus = MOVING_TO_HOME;
          updateScreen(key);
          hommingX();
          runningState = GO_ZERO;
          updateScreen(key);
          ////Serial.println("GOHOME end");
          break;
        case GO_ZERO:
          
          
          ////Serial.println("GOZERO");
          currntXMotorStatus = MOVING_TO_EDGE;
          stepperX.setMaxSpeed(menuValue[STATE_XSPEED]);
          stepperX.setAcceleration(menuValue[STATE_XACCELERATION]);
          stepperX.moveTo(menuValue[STATE_ZEROPOS]);
          stepperX.run();
          if(stepperX.distanceToGo()==0){
            currentCut=0;
            runningState = MOVE_Y_TO_EDGE;
            currntXMotorStatus = NO_MOVING;
            ////Serial.println("GOZERO end");
          }
          
          break;
        case MOVE_Y_TO_EDGE:
          
          if (currenYMotorStatus == AT_HOME) {
            digitalWrite(DIR_PIN_Y, 1);
            analogWriteFrequency(menuValue[STATE_YSPEED]);
            analogWrite(STEP_PIN_Y, 127);
            ////Serial.println("MOVE_Y_TO_EDGE");
            currenYMotorStatus = MOVING_TO_EDGE;
            
            updateScreen(key);
          }
          if (currenYMotorStatus == MOVING_TO_EDGE && stateYb() ) {
             // //Serial.println("changing direction");
              digitalWrite(DIR_PIN_Y, 0);
              analogWriteFrequency(menuValue[STATE_YSPEED]);
              analogWrite(STEP_PIN_Y, 127);
              currenYMotorStatus = MOVING_TO_HOME;
              runningState = MOVE_Y_TO_HOME;
              
              updateScreen(key);
              ////Serial.println("MOVE_Y_TO_HOME");
          }
          break;
        case MOVE_Y_TO_HOME:
          ////Serial.println("MOVE_Y_TO_HOME");
          if(currenYMotorStatus == MOVING_TO_HOME && stateYa()){
              updateScreen(key);
              digitalWrite(DIR_PIN_Y, 0);
              analogWriteFrequency(menuValue[STATE_YSPEED]);
              analogWrite(STEP_PIN_Y, 0);
              digitalWrite(DIR_PIN_Y, 1);
              analogWriteFrequency(menuValue[STATE_YSPEED]);
              analogWrite(STEP_PIN_Y, 255);
              currenYMotorStatus = AT_HOME;
              runningState = INCREMENT_COUNTER;
              currentCut++;
              if(currentCut >= menuValue[STATE_CUTS])
                  {
                    hommingX();
                    runningState = FINISH;
                    currentCut = 0;
                    setBackupRegister(LL_RTC_BKP_DR1, (uint32_t)currentCut);

                    //updateScreen('\0');
                  }
              setBackupRegister(LL_RTC_BKP_DR1, (uint32_t)currentCut);
              //writeBackupRegister(BKP_DR1,currentCut); // Example data
              updateScreen('\0');
            
          }
          ////Serial.println("MOVE_Y_TO_HOME end");
          break;
        case INCREMENT_COUNTER:  //and check if it is last state or not if yes then go home other wise repeat from move y to edge
              ////Serial.println("INCREMENT_COUNTER"); 
              if(currntXMotorStatus == NO_MOVING){
                stepperX.setMaxSpeed(menuValue[STATE_XSPEED]);
                stepperX.setAcceleration(menuValue[STATE_XACCELERATION]);
                stepperX.moveTo(stepperX.currentPosition()+menuValue[STATE_PITCH]);
                currntXMotorStatus = MOVING_TO_EDGE;
                updateScreen(key);
              }
              stepperX.run();
              if(stepperX.distanceToGo()==0 && currntXMotorStatus == MOVING_TO_EDGE){
                  currntXMotorStatus = NO_MOVING;
                  
                  if(currentCut >= menuValue[STATE_CUTS])
                  {
                    hommingX();
                    runningState = FINISH;
                    currentCut = 0;
                    setBackupRegister(LL_RTC_BKP_DR1, (uint32_t)currentCut);

                    updateScreen('\0');
                  }else{
                    runningState = MOVE_Y_TO_EDGE;

                  }

              }
              ////Serial.println("INCREMENT_COUNTER end");
              
            
          break;
          case FINISH:
            play = false;
            currentCut = 0;
            runningState = GO_HOME;
            currentScreen = SCREEN_JOG;
            updateScreen('\0');
          break;
      }
    } 
    if(pause == true) {
      
      switch(pausedState){
        case GO_ZERO:
          
        break;
        case MOVE_Y_TO_EDGE:
        break;
        case MOVE_Y_TO_HOME:
        break;
        case INCREMENT_COUNTER:
        break;
        case FINISH:
        break;
      }
      
    }
  }
  // //Serial.print(currentScreen);
  // //Serial.print(" pause ");
  // //Serial.println(play ? "true" : "false");


}  // End loop


// Taking care of some special events.
void keypadEvent(KeypadEvent key) {
  switch (kpd.getState()) {

    case PRESSED:
      switch (currentScreen) {
        case SCREEN_JOG:

          if (numlock == true) {
            // Handle numeric input
            if (key >= '0' && key <= '9') {
              // Append the digit to the input buffer
              inputBuffer += key;
            }
            if (key == '*') {
            }
            if (key == '#') {
              hommingAllowed = false;
              currntXMotorStatus = MOVING_TO_EDGE;
              speed = inputBuffer.toInt();
              if (speed >= 0) {
                hommingAllowed = false;
                currntXMotorStatus = MOVING_TO_EDGE;
              } else {
                hommingAllowed = true;
                currntXMotorStatus = MOVING_TO_HOME;
              }
              stepperX.moveTo(stepperX.currentPosition() + speed);
              ////Serial.println(speed);
            }

            if (key == 'D') {
              numlock = false;
            }

          } else {
            dir = true;
            inputBuffer = "";
            if (key == '4') {
              hommingAllowed = false;
              currntXMotorStatus = MOVING_TO_EDGE;
              speed = 1;
            }
            if (key == '6') {
              hommingAllowed = true;
              speed = 1;
              currntXMotorStatus = MOVING_TO_HOME;
            }
            if (key == '5') {
              hommingAllowed = false;
              hommingX();
            }
            if (key == '2') {
              if (currenYMotorStatus != AT_HOME && !stateYa()) {
                currenYMotorStatus = MOVING_TO_HOME;
                digitalWrite(DIR_PIN_Y, 0);
                analogWriteFrequency(menuValue[STATE_YSPEED]);
                analogWrite(STEP_PIN_Y, 127);
              }
            }
            if (key == '8') {
              if (currenYMotorStatus != AT_EDGE && !stateYb()) {
                currenYMotorStatus = MOVING_TO_EDGE;
                digitalWrite(DIR_PIN_Y, 1);
                analogWriteFrequency(menuValue[STATE_YSPEED]);
                analogWrite(STEP_PIN_Y, 127);
              }
            }
            if (key == '0') {
              menuValue[STATE_ZEROPOS] = stepperX.currentPosition();
              updateScreen(key);
            }
            if (key == 'A') {
              play = false;
              pause = true;

              runningState = GO_HOME;
              currentScreen = SCREEN_RUN;
            }
            if (key == 'B') {
              readIntArrayFromEEPROM(menuValue, startAddress);
              currentScreen = SCREEN_PROGRM;
            }

            if (key == 'C') {
              enable = !enable;
              if (enable) {
                stepperX.disableOutputs();
                digitalWrite(ENABLE_PIN_Y, HIGH);
              } else {
                stepperX.enableOutputs();
                digitalWrite(ENABLE_PIN_Y, LOW);
              }
            }
            if (key == 'D') {
              speed = 0;
              inputBuffer = "";
              numlock = true;
            }
          }
          break;

        case SCREEN_PROGRM:
          if (key >= '0' && key <= '9') {
            // Append the digit to the input buffer
            inputBuffer += key;
            menuValue[selectedItem] = inputBuffer.toInt();
            updateScreen(key);
          }
          if (key == 'A') {
            inputBuffer = "";
            if (selectedItem > 0) {
              selectedItem--;
              if (cursorPosition > 0) {
                cursorPosition--;
              } else {
                topMenuItem--;
              }
            }
          }

          if (key == 'B') {
            inputBuffer = "";
            if (selectedItem < menuItemCount - 1) {
              selectedItem++;
              if (cursorPosition < 3) {
                cursorPosition++;
              } else {
                topMenuItem++;
              }
            }
          }
          if (key == 'C') {
          }
          if (key == 'D') {
            saveIntArrayToEEPROM(menuValue, startAddress);
            inputBuffer = "";
            currentScreen = SCREEN_JOG;
          }
          if (key = '*') {
          }
          if (key = '#') {
          }
          break;

        case SCREEN_RUN:
          if (key != '\0') {
          }
          if (key == 'A') {
           
            if(pause){
              switch (runningState) {
              case MOVE_Y_TO_EDGE:
                if(stateYb()){
                  digitalWrite(DIR_PIN_Y, 1);
                  analogWriteFrequency(menuValue[STATE_YSPEED]);
                  analogWrite(STEP_PIN_Y, 0);
                }else{
                  digitalWrite(DIR_PIN_Y, 1);
                  analogWriteFrequency(menuValue[STATE_YSPEED]);
                  analogWrite(STEP_PIN_Y, 127);
                }
                
              break;
              case MOVE_Y_TO_HOME:
                if(stateYa()){
                  digitalWrite(DIR_PIN_Y, 0);
                  analogWriteFrequency(menuValue[STATE_YSPEED]);
                  analogWrite(STEP_PIN_Y, 0);
                }else{
                  digitalWrite(DIR_PIN_Y, 0);
                  analogWriteFrequency(menuValue[STATE_YSPEED]);
                  analogWrite(STEP_PIN_Y, 127);
                }
                
              break;

              }
              pause = false;
              play = true;
            }
          }
          if (key == 'B'){
            
            if(play){
              switch (runningState) {
              case MOVE_Y_TO_EDGE:
                digitalWrite(DIR_PIN_Y, 0);
                analogWriteFrequency(menuValue[STATE_YSPEED]);
                analogWrite(STEP_PIN_Y, 0);
              break;
              case MOVE_Y_TO_HOME:
                digitalWrite(DIR_PIN_Y, 0);
                analogWriteFrequency(menuValue[STATE_YSPEED]);
                analogWrite(STEP_PIN_Y, 0);
              break;

              }
              play = false;
              pause = true;
            }

          }
          if (key == 'D') {

              play = false;
              currentCut = 0;
              setBackupRegister(LL_RTC_BKP_DR1, (uint32_t)currentCut);
              runningState = GO_HOME;
              if(currntXMotorStatus != NO_MOVING){
                stepperX.stop();
                currntXMotorStatus = NO_MOVING;
              }
              if(currenYMotorStatus != NO_MOVING){
                digitalWrite(DIR_PIN_Y, 0);
                analogWriteFrequency(menuValue[STATE_YSPEED]);
                analogWrite(STEP_PIN_Y, 0);
                currenYMotorStatus = NO_MOVING;
              }
              currentScreen = SCREEN_JOG;
            
          }
          break;
      }
      updateScreen(key);
      break;

    case RELEASED:
      switch (currentScreen) {
        case SCREEN_JOG:
          if (key == '*') {
            if (dirHold) {
              dirHold = false;
            } else {
              if (inputBuffer.length() > 0) {
                inputBuffer.remove(inputBuffer.length() - 1);
              }
            }
            updateScreen(key);
          }
          if (key == '4' || key == '6') {
            scalling = false;
            count = 500;
            speed = 0;
          }
          if (key == '2' || key == '8') {
            digitalWrite(DIR_PIN_Y, 1);
            analogWriteFrequency(menuValue[STATE_YSPEED]);
            analogWrite(STEP_PIN_Y, 0);
            if (currenYMotorStatus == MOVING_TO_EDGE || currenYMotorStatus == MOVING_TO_HOME) {
              currenYMotorStatus = NO_MOVING;
            }
          }
          updateScreen(key);
          break;

        case SCREEN_PROGRM:
          if (key == '*') {
            if (dirHold) {
              dirHold = false;
            } else {
              if (inputBuffer.length() > 0) {
                inputBuffer.remove(inputBuffer.length() - 1);
                menuValue[selectedItem] = inputBuffer.toInt();
              }
            }
            updateScreen(key);
          }
          break;
        case SCREEN_RUN:

          break;
      }
      break;

    case HOLD:
      switch (currentScreen) {

        case SCREEN_JOG:
          if (key == '4' || key == '6') {

            updateScreen(key);
            scalling = true;
            lastTime = millis();
          }
          if (key == '*') {
            dirHold = true;
            if (inputBuffer.startsWith("-")) {
              inputBuffer = inputBuffer.substring(1);  // Remove the negative sign
            } else {
              inputBuffer = "-" + inputBuffer;  // Add the negative sign
            }
            updateScreen(key);
          }
          if (key == '#') {
          }
          if (key == 'C') {
          }

          break;

        case SCREEN_PROGRM:
          if (key == '*') {
            dirHold = true;
            if (inputBuffer.startsWith("-")) {
              inputBuffer = inputBuffer.substring(1);  // Remove the negative sign
              menuValue[selectedItem] = inputBuffer.toInt();
            } else {
              inputBuffer = "-" + inputBuffer;  // Add the negative sign.
              menuValue[selectedItem] = inputBuffer.toInt();
            }
            updateScreen(key);
          }
          break;
        case SCREEN_RUN:
          
          
        break;
      }
      break;
  }
}

void hommingX() {
   if (currntXMotorStatus == AT_HOME) {
     return;
   }
  //updateScreen('5');
  lcd.clear();
  lcd.print("Homming X Motor:");
  // Set the motors' speed and acceleration based on menu values
  stepperX.setMaxSpeed(menuValue[STATE_XSPEED]);
  stepperX.setAcceleration(menuValue[STATE_XACCELERATION]);
  currntXMotorStatus = MOVING_TO_HOME;

  //Serial.println("Starting homing routine...");
  // Move towards the limit switch at a slow speed
  stepperX.setSpeed(-menuValue[STATE_XSPEED]);  // Adjust speed as necessary
  // Move to hit the limit switch
  while (digitalRead(LIMIT_SWITCH_HOME) == HIGH) {
    stepperX.runSpeed();
    //  //Serial.print("Moving towards switch, current position: ");
    // //Serial.println(stepperX.currentPosition());
  }
  // Stop the motor
  stepperX.stop();
  delay(100);  // Small delay to ensure the motor stops completely
  //Serial.println("Limit switch hit.");
  // Move forward until the limit switch is released
  stepperX.setSpeed(100);  // Adjust speed as necessary
  lcd.print("done");
  while (digitalRead(LIMIT_SWITCH_HOME) == LOW) {
    stepperX.runSpeed();
    //   //Serial.print("Moving away from switch, current position: ");
    // //Serial.println(stepperX.currentPosition());
  }
  //lcd.print("D");
  // Stop the motor
  stepperX.stop();
  delay(100);  // Small delay to ensure the motor stops completely
  ////Serial.println("Limit switch released.");
  // Set the current position as zero
  stepperX.setCurrentPosition(0);
  //  //Serial.println("Homing complete. Current position set to 0.");
  //lcd.print("K");
  currntXMotorStatus = AT_HOME;
  updateScreen('\0');
}
void hommingY() {
  if (currenYMotorStatus == AT_HOME) {
    return;
  }
  currenYMotorStatus = MOVING_TO_HOME;
  digitalWrite(DIR_PIN_Y, 0);
  analogWriteFrequency(menuValue[STATE_YSPEED]);
  analogWrite(STEP_PIN_Y, 127);
  while (!stateYa()) {
    ////Serial.print("Moving Y motor towards switch, current position: ");
  }
  digitalWrite(DIR_PIN_Y, 0);
  analogWriteFrequency(menuValue[STATE_YSPEED]);
  analogWrite(STEP_PIN_Y, 0);
  delay(100);  // Small delay to ensure the motor stops completely
  currenYMotorStatus = AT_HOME;
}

// Function to save integer array to EEPROM
void saveIntArrayToEEPROM(int* array, int startIndex) {
  int size = sizeof(menuValue) / sizeof(int);
  int address = startIndex;
  for (int i = 0; i < size; i++) {
    ee.writeBlock(address, (uint8_t*)&array[i], sizeof(int));
    address += sizeof(int);
  }
}
// Function to read integer array from EEPROM
void readIntArrayFromEEPROM(int* array, int startIndex) {
  int size = sizeof(menuValue) / sizeof(int);
  int address = startIndex;
  for (int i = 0; i < size; i++) {
    ee.readBlock(address, (uint8_t*)&array[i], sizeof(int));
    address += sizeof(int);
  }
}
bool stateYa(void) {
  return !digitalRead(LIMIT_SWITCH_YA);
}

bool stateYb(void) {
  return !digitalRead(LIMIT_SWITCH_YB);
}

void updateScreen(KeypadEvent key) {
 // lcd.init();
 // lcd.clear();
  if (hasPowerFailed) {

  } else {
    switch (currentScreen) {
      case SCREEN_JOG:
        lcd.clear();
        lcd.setCursor(0, 0);
        lcd.print("A>Run ");
        lcd.print(menuValue[STATE_ZEROPOS]);
        lcd.setCursor(0, 1);
        lcd.print("B>Values");
        lcd.setCursor(0, 2);
        lcd.print("C>");
        lcd.print(enable ? "Enable" : "Disable");
        lcd.setCursor(15, 0);
        lcd.write(UP_SYMBOL);
        lcd.print((key == '2' && kpd.getState() == PRESSED) ? " " : "2");
        lcd.setCursor(12, 1);
        lcd.write(LEFT_SYMBOL);
        lcd.print((key == '4' && kpd.getState() == PRESSED) ? " " : "4");
        lcd.setCursor(15, 1);
        lcd.write(HOME_SYMBOL);
        lcd.print((key == '5' && kpd.getState() == PRESSED) ? " " : "5");
        lcd.setCursor(18, 1);
        lcd.write(RIGHT_SYMBOL);
        lcd.print((key == '6' && kpd.getState() == PRESSED) ? " " : "6");
        lcd.setCursor(15, 2);
        lcd.write(DOWN_SYMBOL);
        lcd.print((key == '8' && kpd.getState() == PRESSED) ? " " : "8");
        lcd.setCursor(0, 3);
        lcd.print("D>X");
        lcd.print(numlock ? "<" : ":");
        lcd.print(numlock ? inputBuffer : String(stepperX.currentPosition()));
        lcd.print(numlock ? ">" : "");
        lcd.setCursor(12, 3);
        lcd.print("Y:");
        lcd.print("E");
        switch (currenYMotorStatus) {
          case NO_MOVING:
            lcd.print("----");
            break;
          case AT_EDGE:
            lcd.print("<---");
            break;
          case AT_HOME:
            lcd.print("--->");
            break;
          case MOVING_TO_EDGE:
            lcd.print("<<<<");
            break;
          case MOVING_TO_HOME:
            lcd.print(">>>>");
            break;
        }
        lcd.print("H");
        break;
      case SCREEN_PROGRM:
        lcd.clear();
        for (int i = 0; i < 4; i++) {
          int menuItemIndex = topMenuItem + i;
          if (menuItemIndex < menuItemCount) {
            lcd.setCursor(0, i);
            if (menuItemIndex == selectedItem) {
              lcd.print(">");
            } else {
              lcd.print(" ");
            }
            lcd.print(menuItems[menuItemIndex]);
            lcd.print(":");
            lcd.print(menuValue[menuItemIndex]);
          }
        }
        // Display up arrow if not at the first item
        if (topMenuItem > 0) {
          lcd.setCursor(19, 0);
          lcd.write(UP_SYMBOL);  // Up arrow
        }
        // Display down arrow if not at the last item
        if (topMenuItem + 4 < menuItemCount) {
          lcd.setCursor(19, 3);
          lcd.write(DOWN_SYMBOL);  // Down arrow
        }
        break;
      case SCREEN_RUN:
        lcd.clear();
          lcd.setCursor(0, 0);
          if(play && runningState != FINISH){
            lcd.print("Cutting:");
          }else if( !play && runningState == FINISH){
            lcd.print("Done:");
          }else {
            lcd.print("Paused:");
          }
          lcd.print(currentCut);
          lcd.setCursor(18, 0);
          lcd.print("A");
          lcd.write(PLAY_SYMBOL);
          lcd.setCursor(0, 1);
          lcd.print("No of Cuts:");
          lcd.print(menuValue[STATE_CUTS]);
          lcd.setCursor(18,1);
          lcd.print("B");
          lcd.write(PAUSE_SYMBOL);
          lcd.setCursor(0, 2);
          lcd.print("Pitch:");
          lcd.print(menuValue[STATE_PITCH]);
          lcd.setCursor(18,2);
          lcd.print("D");
          lcd.write(CANCEL_SYMBOL);
          lcd.setCursor(0, 3);
          lcd.print("X");
          lcd.print(numlock ? "<" : ":");
          lcd.print(numlock ? inputBuffer : String(stepperX.currentPosition()));
          lcd.print(numlock ? ">" : "");
          lcd.setCursor(12, 3);
          lcd.print("Y:");
          lcd.print("E");
          switch (currenYMotorStatus) {
            case NO_MOVING:
              lcd.print("----");
              break;
            case AT_EDGE:
              lcd.print("<---");
              break;
            case AT_HOME:
              lcd.print("--->");
              break;
            case MOVING_TO_EDGE:
              lcd.print("<<<<");
              break;
            case MOVING_TO_HOME:
              lcd.print(">>>>");
              break;
          }
          lcd.print("H");
        break;
    }
  }
}
// void writeToBackupRegister(uint16_t reg, uint32_t value) {
//   if (reg < 1 || reg > 20) return;
//   HAL_RTCEx_BKUPWrite(&hrtc, reg, value);
// }

// uint32_t readFromBackupRegister(uint16_t reg) {
//   if (reg < 1 || reg > 20) return 0;
//   return HAL_RTCEx_BKUPRead(&hrtc, reg);
// }