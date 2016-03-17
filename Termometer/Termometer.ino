#include <OneWire.h>
#include <DallasTemperature.h>
#include <Wire.h> 
#include <LiquidCrystal_I2C.h>
#include <SM.h>
#include <StopWatch.h>

struct Button {
  uint8_t pin;
  unsigned long timer;
};

struct TemperatureAlarm {
  uint8_t target;
  uint8_t lastValue;
  bool set;
  bool alertOnHigher;
  bool triggered;
};

struct TimerAlarm {
  uint8_t h;
  uint8_t m;
  uint8_t s;
  bool set;
  bool triggered;
};

//Initialize temp probes
#define ONE_WIRE_BUS 2 //probes connected on pin 2
#define TEMPERATURE_PRECISION 12 // DS18B20 probe resolution - 9=0,5 10=0,25, 11=0,125, 12=0,0625
OneWire oneWire(ONE_WIRE_BUS);
DallasTemperature sensors(&oneWire);

uint8_t numberOfDevices; //number of probes found on the wire
DeviceAddress probes[3]; //Array of addresses to the probes

// Initialize LCD on address 0x27
LiquidCrystal_I2C lcd(0x27, 2, 1, 0, 4, 5, 6, 7, 3, POSITIVE);

//custom alarm symbol
byte alarmSymbol[8] = {
  B00100,
  B01010,
  B01110,
  B01010,
  B00000,
  B00000,
  B00000,
  B00000
};

const unsigned long updateTimerInterval       = 1000; //update timer every 1 sec.
const unsigned long updateTemperatureInterval = 2000; //update temperatures every 2 sec.
const unsigned long buttonLongPressTime       = 1000; //1 sec press to reset timer

unsigned long timerLastUpdated = 0;
unsigned long tempLastUpdated = 0;

StopWatch timer;

//Buttons
Button buttonTimer = {A1, 0};
Button buttonAlarm = {A0, 0};

//buzzer connected on pin 3
uint8_t buzzer = 3;

//State prototype - needed because of a change introduced in Arduino 1.6.6
void sDefault();

//Define StateMachine
SM sm(sDefault);

//Alarms
TemperatureAlarm tempAlarms[] = {{0,0,false,false,false},
                                 {0,0,false,false,false},
                                 {0,0,false,false,false}};
TimerAlarm timerAlarm = {0,0,0,false,false};

bool alarmActive =    false;
bool alarmTriggered = false;
uint8_t alarmCursor = 1;
uint8_t buzzerState = LOW;

unsigned long alarmLastChecked = 0;
unsigned long buzzerTimer = 0;

uint8_t lcdBacklightState = HIGH;

void setup() {
  //initialize input/output pins
  pinMode(buttonTimer.pin, INPUT); 
  pinMode(buttonAlarm.pin, INPUT); 
  pinMode(buzzer, OUTPUT);

  // initialize the lcd
  lcd.begin(2,16);  
  lcd.createChar(0, alarmSymbol);
  lcd.clear(); 
  lcd.setBacklight(lcdBacklightState);

  // Start up the ds18b20 library
  sensors.begin();
  sensors.setWaitForConversion(false); //run in async mode - don't wait for probes
  numberOfDevices = sensors.getDeviceCount();
  for(int i=0;i<numberOfDevices;i++)
  {
    sensors.getAddress(probes[i], i);
    sensors.setResolution(probes[i],TEMPERATURE_PRECISION);
  }
  sensors.requestTemperatures();

  //Print start values to LCD
  lcd.setCursor(0,0);
  lcd.print(F("Velkommen til"));
  lcd.setCursor(0,1);
  lcd.print(F("Strandby Bryghus"));
  delay(2000);
  lcd.clear();
  lcd.setCursor(0,0);
  lcd.print(F("Initialiserer..."));
  lcd.setCursor(0,1);
  lcd.print(F("Fandt "));
  lcd.print(numberOfDevices,DEC);
  lcd.print(F(" prober."));
  delay(5000);
  lcd.clear();
  lcd.setCursor(8,1);
  lcd.print(F("0:00:00"));

}

void loop() {
  //Run state machine
  EXEC(sm);
}

//////////////////////////
//Functions 
//////////////////////////
void updateTimer() {
  int s = (timer.elapsed()/1000) % 60;
  int m = (timer.elapsed()/60000) % 60;
  int h = (timer.elapsed()/3600000);
  char buf[10];
  sprintf(buf,"%d:%02d:%02d",h,m,s);
  lcd.setCursor(8,1);
  lcd.print(buf);
  if(timerAlarm.set) {
    lcd.write((uint8_t)0); //custom alarm symbol
  }
}

void updateTemperatures() {
  //place cursor depending on probe number
  for(uint8_t i=0;i<numberOfDevices; i++) {
    if(i == 0) lcd.setCursor(0,0);
    if(i == 1) lcd.setCursor(8,0);
    if(i == 2) lcd.setCursor(0,1);
    //Print temperature
    float tempC = sensors.getTempC(probes[i]);
    lcd.print(i+1,DEC);
    lcd.print(F(":"));
    lcd.print(tempC,1);
    if(tempAlarms[i].set) {
      lcd.write((uint8_t)0); //custom alarm symbol
    }
    tempAlarms[i].lastValue = int(tempC);
  }
  // Send the command to get temperatures for next reading.
  sensors.requestTemperatures(); 
}

void resetTimer() {
  timer.stop();
  timer.reset();
  lcd.setCursor(8,1);
  lcd.print(F("0:00:00"));
}

void toggleTimer() {
  if(timer.isRunning()) {
    timer.stop();
  } else {
    timer.start();
  }
}

void toggleLCDBacklight() {
  lcdBacklightState = !lcdBacklightState;
  lcd.setBacklight(lcdBacklightState);
}

void updateAlarmScreen() {
  char buf[10];
  for(uint8_t i=0;i<numberOfDevices; i++) {
    if(i == 0) lcd.setCursor(0,0);
    if(i == 1) lcd.setCursor(8,0);
    if(i == 2) lcd.setCursor(0,1);
    sprintf(buf, "%d:%02d", i+1, tempAlarms[i].target);
    lcd.print(buf);
    if(tempAlarms[i].set) {
      lcd.write((uint8_t)0); //custom alarm symbol
    }
  }
  lcd.setCursor(8,1);
  sprintf(buf,"%d:%02d:%02d",timerAlarm.h,timerAlarm.m,timerAlarm.s);
  lcd.print(buf);
  if(timerAlarm.set) {
    lcd.write((uint8_t)0); //custom alarm symbol
  }
  setAlarmCursorPos();
  lcd.blink();
}

void setAlarmCursorPos() {
  //blink cursor on active alarm digit
  switch(alarmCursor) {
    case 1: lcd.setCursor(2,0); break;
    case 2: lcd.setCursor(3,0); break;
    case 3: lcd.setCursor(10,0); break;
    case 4: lcd.setCursor(11,0); break;
    case 5: lcd.setCursor(2,1); break;
    case 6: lcd.setCursor(3,1); break;
    case 7: lcd.setCursor(8,1); break;
    case 8: lcd.setCursor(10,1); break;
    case 9: lcd.setCursor(11,1); break;
    case 10: lcd.setCursor(13,1); break;
    case 11: lcd.setCursor(14,1); break;
  }
}

void increaseAlarmValue() {
  switch(alarmCursor) {
    case 1: tempAlarms[0].target += 10; break;
    case 2: tempAlarms[0].target ++;    break;
    case 3: tempAlarms[1].target += 10; break;
    case 4: tempAlarms[1].target ++;    break;
    case 5: tempAlarms[2].target += 10; break;
    case 6: tempAlarms[2].target ++;    break;
    case 7: timerAlarm.h ++;     break;
    case 8: timerAlarm.m += 10;  break;
    case 9: timerAlarm.m ++;     break;
    case 10: timerAlarm.s += 10; break;
    case 11: timerAlarm.s ++;    break;
  }
  //make sure values are still within valid range
  for(uint8_t i = 0;i<numberOfDevices;i++) {
    if(tempAlarms[i].target > 99) tempAlarms[i].target = tempAlarms[i].target % 10;
  }
  if(timerAlarm.h > 9) timerAlarm.h = timerAlarm.h % 10;
  if(timerAlarm.m > 59) timerAlarm.m = timerAlarm.m % 10;
  if(timerAlarm.s > 59) timerAlarm.s = timerAlarm.s % 10;
  updateAlarmScreen();
}

void moveAlarmCursor() {
  if (alarmCursor < numberOfDevices*2) {
    alarmCursor++; //next probe digit
  } else if (alarmCursor == numberOfDevices*2) {
    alarmCursor = 7; //no more probes - jump to alarm time
  } else if (alarmCursor < 11) {
    alarmCursor++; //next timer value
  } else {
    alarmCursor = 1; //back to first digit
  }
  setAlarmCursorPos();
}

void toggleAlarmState() {
  if(alarmCursor <= 6) {
    //cursor is at one of the temp alarms - find out which
    uint8_t i = 0;
    if(alarmCursor == 1 || alarmCursor == 2) {
      i = 0;
    } else if(alarmCursor == 3 || alarmCursor == 4) {
      i = 1;
    } else if(alarmCursor == 5 || alarmCursor == 6) {
      i = 2;
    }
    if(tempAlarms[i].set == false) {
      tempAlarms[i].set = true;
      if(tempAlarms[i].lastValue < tempAlarms[i].target) {
        tempAlarms[i].alertOnHigher = true;
      } else {
        tempAlarms[i].alertOnHigher = false;
      }
      alarmActive = true;
    } else {
      tempAlarms[i].set = false;
    }
  } else {
    //timer alarm
    timerAlarm.set = !timerAlarm.set;
    if(timerAlarm.set) alarmActive = true;
  }
  updateAlarmScreen();
}

void checkAlarm() {
  for(uint8_t i=0;i<numberOfDevices; i++) {
    if(tempAlarms[i].set) {
      float tempC = sensors.getTempC(probes[i]);
      if((tempAlarms[i].alertOnHigher && tempC >= tempAlarms[i].target) || (!tempAlarms[i].alertOnHigher && tempC <= tempAlarms[i].target)) {
        tempAlarms[i].triggered = true;
        alarmTriggered = true;
      }
    }
  }
  if(timer.isRunning() && timerAlarm.set) {
    uint8_t s = (timer.elapsed()/1000) % 60;
    uint8_t m = (timer.elapsed()/60000) % 60;
    uint8_t h = (timer.elapsed()/3600000);
    if(h == timerAlarm.h && m == timerAlarm.m && s >= timerAlarm.s) {
      timerAlarm.triggered = true;
      alarmTriggered = true;
    }
  }
}

void soundAlarm() {
  if(millis() - buzzerTimer > 500) {
    buzzerTimer = millis();
    buzzerState = !buzzerState;
    digitalWrite(buzzer,buzzerState);

    //blink alarm symbol
    for(uint8_t i=0;i<numberOfDevices;i++) {
      if(tempAlarms[i].triggered) {
        if(i == 0) lcd.setCursor(6,0);
        if(i == 1) lcd.setCursor(14,0);
        if(i == 2) lcd.setCursor(6,1);
        if(buzzerState == HIGH) {
          lcd.write((uint8_t)0);
        } else {
          lcd.print(F(" "));
        }
      }
    }
    if(timerAlarm.triggered) {
       lcd.setCursor(15,1);
      if(buzzerState == HIGH) {
        lcd.write((uint8_t)0);
      } else {
        lcd.print(F(" "));
      }
    }
  }
}

void silenceAlarm() {
  digitalWrite(buzzer, LOW);
  alarmTriggered = false;
  for(byte i=0;i<numberOfDevices; i++) {
    if(tempAlarms[i].triggered) {
      tempAlarms[i].triggered = false;
      tempAlarms[i].set = false;
      if(i == 0) lcd.setCursor(6,0);
      if(i == 1) lcd.setCursor(14,0);
      if(i == 2) lcd.setCursor(6,1);
      lcd.print(F(" "));
    }
  }
  if(timerAlarm.triggered) {
    timerAlarm.triggered = false;
    timerAlarm.set = false;
    lcd.setCursor(15,1);
    lcd.print(F(" "));
  }
}

//////////////////////////
// States
//////////////////////////
State sDefault() {
  if(millis() - timerLastUpdated > updateTimerInterval) {
    timerLastUpdated = millis();
    updateTimer();
  } 
  if(millis() - tempLastUpdated > updateTemperatureInterval) {
    tempLastUpdated = millis();
    updateTemperatures();
  }
  if(alarmActive && (millis() - alarmLastChecked > 500)) {
    alarmLastChecked = millis();
    if(!alarmTriggered) {
      checkAlarm();
    }
  }
  if(alarmTriggered) {
    soundAlarm();
  }
  if(analogRead(buttonTimer.pin) > 100) {
    buttonTimer.timer = millis();
    sm.Set(sTimerWait);
  }
  if(analogRead(buttonAlarm.pin) > 100) {
    buttonAlarm.timer = millis();
    sm.Set(sAlarmWait);
  }
}

State sTimerWait() {
  if(millis() - buttonTimer.timer > buttonLongPressTime) {
    //long press
    resetTimer();
    sm.Set(sTimerReleaseWait);
  } else if(analogRead(buttonTimer.pin) < 100) {
    //button released
    toggleTimer();
    sm.Set(sDefault);
  }
}

State sTimerReleaseWait() {
  //wait for button release
  if(analogRead(buttonTimer.pin) < 100) {
    buttonTimer.timer = 0;
    sm.Set(sDefault);
  }
}

State sAlarmWait() {
  if(alarmTriggered) {
    silenceAlarm();
    sm.Set(sAlarmReleaseWait);
  } else {
    if(millis() - buttonAlarm.timer > buttonLongPressTime) {
      toggleLCDBacklight();
      sm.Set(sAlarmReleaseWait);
    }
    if(analogRead(buttonAlarm.pin) < 100) {
      lcd.clear();
      alarmCursor = 1;
      updateAlarmScreen();
      sm.Set(sShowAlarm);
    }
  }
}

State sAlarmReleaseWait() {
  if(analogRead(buttonAlarm.pin) < 100) {
    sm.Set(sDefault);
  }
}

State sShowAlarm() {
  if(sm.Timeout(5000)) {
    //no activity for 5 sec - return to default state
    lcd.noBlink();
    updateTimer();
    updateTemperatures();
    sm.Set(sDefault);
  }
  if(analogRead(buttonTimer.pin) > 100) {
    buttonTimer.timer = millis();
    sm.Set(sTimerWait1);
  }
  if(analogRead(buttonAlarm.pin) > 100) {
    buttonAlarm.timer = millis();
    sm.Set(sAlarmWait1);
  }
}

State sTimerWait1() {
  if(analogRead(buttonTimer.pin) < 100) {
    increaseAlarmValue();
    sm.Set(sShowAlarm);
  }
}

State sAlarmWait1() {
  if(millis() - buttonAlarm.timer > buttonLongPressTime) {
    //long press
    toggleAlarmState();
    sm.Set(sTimerReleaseWait1);
  } else if(analogRead(buttonAlarm.pin) < 100) {
    //button released
    moveAlarmCursor();
    sm.Set(sShowAlarm);
  }
}

State sTimerReleaseWait1() {
  if(analogRead(buttonAlarm.pin) < 100) {
    sm.Set(sShowAlarm);
  }
}

