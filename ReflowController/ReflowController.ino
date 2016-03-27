// ----------------------------------------------------------------------------
// Reflow Oven Controller
// (c) 2016 Steve Smith
// (c) 2014 Karl Pitrich <karl@pitrich.com>
// (c) 2012-2013 Ed Simmons
// ----------------------------------------------------------------------------
//
// Controller
//
// 128x160 LCD spi controller
// rotary encoder/push button
// MAX6675 K type thermocouple
// SSR drive active low
// Arduino 328P UNO or Duemilanove
// ----------------------------------------------------------------------------

const char *ver = "SRS 0.9(i)";

// ----------------------------------------------------------------------------

// #define PIDTUNE 0 // 
// #define DEBUG

#define DEFAULT_LOOP_DELAY  70 // should be about 16% less for 60Hz mains

#include <avr/eeprom.h>
#include <EEPROM.h>
#include <PID_v1.h>
#include <Adafruit_GFX.h>
#include <Adafruit_ST7735.h>
#include <SPI.h>
#include <Menu.h>
#include <TimerOne.h>
#include <ClickEncoder.h>
#include <max6675.h>
#include "helpers.h"
#include <FlexiTimer2.h>

#ifdef PIDTUNE
#include <PID_AutoTune_v0.h>
#endif

#define  TICK_PERIOD       10   // xx ms for zero crossing ISR
#define  CYCLE_PERIOD      10   // TICK PERIODS
#define  DISPLAY_PERIOD    25   // TICK PERIODS

static int ticksPerSec = 1000 / TICK_PERIOD;

#define  MAX_START_TEMP    50  // maximum temperature where a new reflow session will be allowed to start
#define  NUM_PHASES         5  // number of phases in a profile

#define DEFAULT_HEATER_Kp 4.25
#define DEFAULT_HEATER_Ki 0.25
#define DEFAULT_HEATER_Kd 6.50

// ----------------------------------------------------------------------------
// Hardware Configuration 
// ----------------------------------------------------------------------------

// 1.8" TFT via SPI

#define LCD_CS             8
#define LCD_RST            9
#define LCD_DC             10
#define LCD_SDA            11
#define LCD_CLK            13

// Thermocouple via S/W SPI

#define THERMOCOUPLE1_CS   4
#define THERMOCOUPLE1_DO   5
#define THERMOCOUPLE1_CLK  6

#define PIN_HEATER         2 // SSR for the heater

// Rotary encoder with switch

#define ENCODER_A          A0 // Common to GND
#define ENCODER_B          A1
#define ENCODER_SW         A2 // Common to GND

#define ENCODER_CLICKS     4  // 

#define SOUNDER            A5 // alert to open oven door for rapid temp drop

// ----------------------------------------------------------------------------
#define WITH_SPLASH 1
// ----------------------------------------------------------------------------

volatile uint32_t timerTicks     = 0;
volatile uint32_t zeroCrossTicks = 0;

const uint8_t     maxProfiles = 4;
uint16_t          profileDuration = 300;
uint16_t          profilePeak     = 220;

char buf[20]; // generic char buffer

// ----------------------------------------------------------------------------
// state machine

typedef enum {
  Phase1 = 0, // RampToSoak
  Phase2,     // Soak
  Phase3,     // RampUp
  Phase4,     // Peak
  Phase5,     // Rampdown
  CoolDown,
  Complete = 20,

  Idle,     
  Settings,
  Edit,

  UIMenuEnd,
  Tune = 30
} State;

State     currentState      = Idle;
State     nextState         = Idle;
State     previousState     = Idle;
bool      stateChanged      = false;
uint32_t  stateChangedTicks = 0;
int       activeProfileId   = 0;
int       idleTemp          = 50; // temperature at which to leave the oven to safely cool naturally

struct ReflowPhase {
  char    Name[12];
  int     StartTemperatureC;
  int     EndTemperatureC;
  double  Rate;
  int     DurationS;
  boolean Rising;
  State   NextState;
};

struct ReflowProfile {
  ReflowPhase Phases[NUM_PHASES];
  uint8_t     checksum;
};

ReflowProfile activeProfile;

ReflowProfile defaultProfile = {
  { // Zone         Start(C)   Exit(C)   Rate    Duration,   Rising  Next
    { "Pre-heat",    50,      150,      1.0,    160,        true,    Phase2},
    { "Soak",       150,      180,      0.6,     60,        true,    Phase3},
    { "ReflowPeak", 180,      220,     10.0,     40,        true,    Phase4},
    { "ReflowCool", 220,      180,     10.0,      4,        false,   Phase5},
    { "Cool",       180,       50,      1.5,    120,        false,   CoolDown},
  },
  0
};

// EEPROM offsets
const uint16_t offsetProfileNum = maxProfiles * sizeof(ReflowProfile) + 2; // one byte
const uint16_t offsetPidConfig  = maxProfiles * sizeof(ReflowProfile) + 3; // sizeof(PID_t)

// ----------------------------------------------------------------------------

uint32_t startCycleZeroCrossTicks;
uint32_t lastUpdate        = 0;
uint32_t lastDisplayUpdate = 0;

// ----------------------------------------------------------------------------

double lastTemp = 0;

struct Thermocouple {
  double  temperature;
  uint8_t stat;
};

Thermocouple A;

MAX6675 sensor(THERMOCOUPLE1_CLK,
               THERMOCOUPLE1_CS,
               THERMOCOUPLE1_DO);

// ----------------------------------------------------------------------------

void readThermocouple(struct Thermocouple* input) {
  double t;

  t = sensor.readCelsius();
  input->temperature = t;
  
  if (isnan(t)) {
    // uh oh, no thermocouple attached!
    input->stat = 1;
    input->temperature = lastTemp;
  }
  lastTemp = input->temperature;
}

// ----------------------------------------------------------------------------
// UI
// NB: Adafruit GFX ASCII-Table is bogous: https://github.com/adafruit/Adafruit-GFX-Library/issues/22
//

Adafruit_ST7735  tft = Adafruit_ST7735(LCD_CS, LCD_DC, LCD_RST);
ClickEncoder     Encoder(ENCODER_A, ENCODER_B, ENCODER_SW, ENCODER_CLICKS);
Menu::Engine     Engine;

int16_t       encMovement;
int16_t       encAbsolute;
int16_t       encLastAbsolute       = -1;
const uint8_t menuItemsVisible      = 8;
const uint8_t menuItemHeight        = 14;
bool          menuUpdateRequest     = true;
bool          initialProcessDisplay = false;
int16_t       editPhase = 0;

#define       PROMPT_Y 100
#define       EXIT_Y   114

// ----------------------------------------------------------------------------

// track menu item state to improve render performance
typedef struct {
  const Menu::Item_t  *mi;
  uint8_t             pos;
  bool                current;
} LastItemState_t;

LastItemState_t currentlyRenderedItems[menuItemsVisible];

// ----------------------------------------------------------------------------

void clearLastMenuItemRenderState() {
  for (uint8_t i = 0; i < menuItemsVisible; i++) {
    currentlyRenderedItems[i].mi      = NULL;
    currentlyRenderedItems[i].pos     = 0xff;
    currentlyRenderedItems[i].current = false;
  }
}

// ----------------------------------------------------------------------------

extern const Menu::Item_t miEditPhase,
                          miStartTemp, 
                          miEndTemp, 
                          miRate, 
                          miDuration, 
                          miLoadProfile, 
                          miSaveProfile,
                          miPidSettingP, 
                          miPidSettingI, 
                          miPidSettingD;

// ----------------------------------------------------------------------------
// PID

uint8_t heaterValue;
double  Setpoint;
double  Input;
double  Output;

typedef struct {
  double Kp;
  double Ki;
  double Kd;
} PID_t;

PID_t heaterPID = {DEFAULT_HEATER_Kp,
                   DEFAULT_HEATER_Ki,
                   DEFAULT_HEATER_Kd};

PID PID(&Input, &Output, &Setpoint, heaterPID.Kp, heaterPID.Ki, heaterPID.Kd, DIRECT);

#ifdef PIDTUNE

PID_ATune PIDTune(&Input, &Output);

double aTuneStep       =  50,
       aTuneNoise      =   2,
       aTuneStartValue =  50; // is set to Output, i.e. 0-100% of Heater

unsigned int aTuneLookBack = 30;
#endif

// ----------------------------------------------------------------------------

bool menuExit(const Menu::Action_t a) {
  clearLastMenuItemRenderState();
  editPhase = 0;
  Engine.lastInvokedItem  = &Menu::NullItem;
  menuUpdateRequest       = false;
  return false;
}

// ----------------------------------------------------------------------------

bool menuDummy(const Menu::Action_t a) {
  return true;
}

// ----------------------------------------------------------------------------

void printDouble(double val, uint8_t precision = 1) {
  ftoa(buf, val, precision);
  tft.print(buf);
}

// ----------------------------------------------------------------------------

void getItemValuePointer(const Menu::Item_t *mi, double **d, int16_t **i) {
  if (mi == &miEditPhase)     *i = &editPhase;
  if (mi == &miStartTemp)     *i = &activeProfile.Phases[editPhase].StartTemperatureC;
  if (mi == &miEndTemp)       *i = &activeProfile.Phases[editPhase].EndTemperatureC;
  if (mi == &miRate)          *d = &activeProfile.Phases[editPhase].Rate;
  if (mi == &miDuration)      *i = &activeProfile.Phases[editPhase].DurationS;
  if (mi == &miPidSettingP)   *d = &heaterPID.Kp;
  if (mi == &miPidSettingI)   *d = &heaterPID.Ki;
  if (mi == &miPidSettingD)   *d = &heaterPID.Kd; 
}

// ----------------------------------------------------------------------------

bool isPidSetting(const Menu::Item_t *mi) {
  return mi == &miPidSettingP || mi == &miPidSettingI || mi == &miPidSettingD;
}

bool isRate(const Menu::Item_t *mi) {
  return mi == &miRate;
}

// ----------------------------------------------------------------------------

bool getItemValueLabel(const Menu::Item_t *mi, char *label) {
  int16_t *iValue = NULL;
  double  *dValue = NULL;

  getItemValuePointer(mi, &dValue, &iValue);

  if (isPidSetting(mi)) {
    ftoa(label, *dValue, 2); // need greater precision with pid values
  } else if (mi == &miStartTemp || mi == &miEndTemp) {
      itostr(label, *iValue, "\367C");
  } else if (mi == &miDuration) {
      itostr(label, *iValue, "s");
  } else if (mi == &miRate) {
      ftoa(label, *dValue, 1);
  } else if (mi == &miEditPhase) {
      itostr(label, *iValue, "");
  }
  return dValue || iValue;
}

// ----------------------------------------------------------------------------

bool editNumericalValue(const Menu::Action_t action) { 

  if (action == Menu::actionDisplay) {
    bool initial = (currentState != Edit);
    currentState = Edit;

    if (initial) {
      tft.setTextColor(ST7735_BLACK, ST7735_WHITE);
      tft.setCursor(10, PROMPT_Y);
      tft.print("Click to EDIT & SAVE");
      tft.setCursor(10, EXIT_Y);
      tft.print("Double click to EXIT");
      Encoder.setAccelerationEnabled(true);
    }

    for (uint8_t i = 0; i < menuItemsVisible; i++) {
      if (currentlyRenderedItems[i].mi == Engine.currentItem) {
        uint8_t y = currentlyRenderedItems[i].pos * menuItemHeight + 2;

        if (initial) {
          tft.fillRect(69, y - 1, 60, menuItemHeight - 2, ST7735_RED);
        }

        tft.setCursor(70, y);
        break;
      }
    }

    tft.setTextColor(ST7735_WHITE, ST7735_RED);

    int16_t *iValue = NULL;
    double  *dValue = NULL;
    getItemValuePointer(Engine.currentItem, &dValue, &iValue);

    if (isPidSetting(Engine.currentItem)) {
      double tmp;
      double factor = 100;
      
      if (initial) {
        tmp = *dValue;
        tmp *= factor;
        encAbsolute = (int16_t)tmp;
      }
      else {
        tmp = encAbsolute;
        tmp /= factor;
        *dValue = tmp;
      }      
    }
    else if (isRate(Engine.currentItem)) {
      double tmp;
      double factor = 10;
      
      if (initial) {
        tmp = *dValue;
        tmp *= factor;
        encAbsolute = (int16_t)tmp;
      }
      else {
        tmp = encAbsolute;
        tmp /= factor;
        *dValue = tmp;
      }      
    } else if (Engine.currentItem == &miEditPhase) {
      if (initial) encAbsolute = *iValue; else *iValue = encAbsolute;
      if (*iValue < 0) *iValue = 0;
      if (*iValue >= NUM_PHASES) *iValue = NUM_PHASES -1;
    } else {      
      if (initial) 
        encAbsolute = *iValue;
      else 
        *iValue = encAbsolute;
    }

    getItemValueLabel(Engine.currentItem, buf);
    tft.print(buf);
    tft.setTextColor(ST7735_BLACK, ST7735_WHITE);
  }

  if (action == Menu::actionParent || action == Menu::actionTrigger) {
    clearLastMenuItemRenderState();
    menuUpdateRequest = true;
    Engine.lastInvokedItem = &Menu::NullItem;

    if (currentState == Edit) { // leave edit mode, return to menu (
      if (isPidSetting(Engine.currentItem)) {
        savePID();
      }
      
      // don't autosave profile, so that one can do "save as" without overwriting the current profile

      currentState = Settings;
      Encoder.setAccelerationEnabled(false);
      return false;
    }

    return true;
  }
}

// ----------------------------------------------------------------------------

bool factoryReset(const Menu::Action_t action) {
#ifndef PIDTUNE
  if (action == Menu::actionDisplay) {
    bool initial = currentState != Edit;
    currentState = Edit;

    if (initial) { // TODO: add eyecandy: colors or icons
      tft.setTextColor(ST7735_BLACK, ST7735_WHITE);
      tft.setCursor(10, PROMPT_Y);
      tft.print("Click to confirm");
      tft.setCursor(10, EXIT_Y);
      tft.print("Doubleclick to exit");
    }
  }

  if (action == Menu::actionTrigger) { // do it
    factoryReset();
    tft.fillScreen(ST7735_WHITE);
    Engine.navigate(Engine.getParent());
    return false;
  }

  if (action == Menu::actionParent) {
    if (currentState == Edit) { // leave edit mode only, returning to menu
      currentState = Settings;
      clearLastMenuItemRenderState();
      return false;
    }
  }
#endif // PIDTUNE
}

// ----------------------------------------------------------------------------

void saveProfile(unsigned int targetProfile, bool quiet = false);

// ----------------------------------------------------------------------------

bool saveLoadProfile(const Menu::Action_t action) {
#ifndef PIDTUNE
  bool isLoad = Engine.currentItem == &miLoadProfile;
  if (action == Menu::actionDisplay) {
    bool initial = currentState != Edit;
    currentState = Edit;

    tft.setTextColor(ST7735_BLACK, ST7735_WHITE);

    if (initial) {
      encAbsolute = activeProfileId;      
      tft.setCursor(10, EXIT_Y);
      tft.print("Doubleclick to exit");
    }

    if (encAbsolute > maxProfiles) encAbsolute = maxProfiles;
    if (encAbsolute <  0) encAbsolute =  0;

    tft.setCursor(10, PROMPT_Y);
    tft.print("Click to ");
    tft.print((isLoad) ? "load " : "save ");
    tft.setTextColor(ST7735_WHITE, ST7735_RED);
    tft.print(encAbsolute);
  }

  if (action == Menu::actionTrigger) {
    (isLoad) ? loadProfile(encAbsolute) : saveProfile(encAbsolute);
    tft.fillScreen(ST7735_WHITE);
    Engine.navigate(Engine.getParent());
    return false;
  }

  if (action == Menu::actionParent) {    
    if (currentState == Edit) { // leave edit mode only, returning to menu
      currentState = Settings;
      clearLastMenuItemRenderState();
      return false;
    }
  }
#endif // PIDTUNE
}

// ----------------------------------------------------------------------------

void toggleAutoTune();

bool cycleStart(const Menu::Action_t action) {
  if (action == Menu::actionDisplay) {
    startCycleZeroCrossTicks = zeroCrossTicks;
    menuExit(action);

#ifndef PIDTUNE    
    calcProfile();
    currentState = Phase1;
#else
    toggleAutoTune();
#endif
    initialProcessDisplay = false;
    menuUpdateRequest = false;
  }

  return true;
}

// ----------------------------------------------------------------------------

void renderMenuItem(const Menu::Item_t *mi, uint8_t pos) {

  bool isCurrent = Engine.currentItem == mi;
  uint8_t y = pos * menuItemHeight + 2;

  if (currentlyRenderedItems[pos].mi == mi 
      && currentlyRenderedItems[pos].pos == pos 
      && currentlyRenderedItems[pos].current == isCurrent) 
  {
    return; // don't render the same item in the same state twice
  }

  tft.setCursor(10, y);

  // menu cursor bar
  tft.fillRect(8, y - 2, tft.width() - 16, menuItemHeight, isCurrent ? ST7735_BLUE : ST7735_WHITE);
  if (isCurrent) tft.setTextColor(ST7735_WHITE, ST7735_BLUE);
  else tft.setTextColor(ST7735_BLACK, ST7735_WHITE);

  tft.print(Engine.getLabel(mi));

  // show values if in-place editable items
  if (getItemValueLabel(mi, buf)) {
    tft.print(' '); tft.print(buf); tft.print("   ");
  }

  // mark items that have children
  if (Engine.getChild(mi) != &Menu::NullItem) {
    tft.print(" \x10   "); // 0x10 -> filled right arrow
  }

  currentlyRenderedItems[pos].mi = mi;
  currentlyRenderedItems[pos].pos = pos;
  currentlyRenderedItems[pos].current = isCurrent;
}

// ----------------------------------------------------------------------------
// Name,                    Label,          Next,         Previous,        Parent,        Child,           Callback

MenuItem(miExit,         "",              Menu::NullItem,   Menu::NullItem, Menu::NullItem, miCycleStart,   menuExit);

#ifndef PIDTUNE
MenuItem(miCycleStart,   "Start Cycle",   miEditProfile,    Menu::NullItem, miExit,         Menu::NullItem, cycleStart);
#else
MenuItem(miCycleStart,   "Start Autotune",miPidSettings,    Menu::NullItem, miExit,         Menu::NullItem, cycleStart);
#endif

MenuItem(miEditProfile,  "Edit Profile",  miLoadProfile,    miCycleStart,   miExit,         miEditPhase,    menuDummy);
  MenuItem(miEditPhase,    "Phase    ",     miStartTemp,      Menu::NullItem, miEditProfile,  Menu::NullItem, editNumericalValue);
  MenuItem(miStartTemp,    "StartTemp",     miEndTemp,        miEditPhase,    miEditProfile,  Menu::NullItem, editNumericalValue);
  MenuItem(miEndTemp,      "End Temp ",     miDuration,       miStartTemp,    miEditProfile,  Menu::NullItem, editNumericalValue);
  MenuItem(miDuration,     "Duration ",     miRate,           miEndTemp,      miEditProfile,  Menu::NullItem, editNumericalValue);
  MenuItem(miRate,         "Rate     ",     Menu::NullItem,   miDuration,     miEditProfile,  Menu::NullItem, editNumericalValue);

MenuItem(miLoadProfile,  "Load Profile",  miSaveProfile,    miEditProfile,  miExit,         Menu::NullItem, saveLoadProfile);
MenuItem(miSaveProfile,  "Save Profile",  miPidSettings,    miLoadProfile,  miExit,         Menu::NullItem, saveLoadProfile);
MenuItem(miPidSettings,  "PID Settings",  miFactoryReset,   miSaveProfile,  miExit,         miPidSettingP,  menuDummy);
  MenuItem(miPidSettingP,  "Heater Kp",     miPidSettingI,    Menu::NullItem, miPidSettings,  Menu::NullItem, editNumericalValue);
  MenuItem(miPidSettingI,  "Heater Ki",     miPidSettingD,    miPidSettingP,  miPidSettings,  Menu::NullItem, editNumericalValue);
  MenuItem(miPidSettingD,  "Heater Kd",     Menu::NullItem,   miPidSettingI,  miPidSettings,  Menu::NullItem, editNumericalValue);
MenuItem(miFactoryReset, "Factory Reset", Menu::NullItem,   miPidSettings,  miExit,         Menu::NullItem, factoryReset);

// ----------------------------------------------------------------------------

#define NUMREADINGS 3

typedef struct {
  double temp;
  uint16_t ticks;
} Temp_t;

Temp_t airTemp[NUMREADINGS];

double readingsT1[NUMREADINGS]; // the readings used to make a stable temp rolling average
double rampRate = 0;            // the result that is displayed
double totalT1 = 0;             // the running total
double averageT1 = 0;           // the average
uint8_t index = 0;              // the index of the current reading

// ----------------------------------------------------------------------------
// Ensure that Solid State Relays are off when starting
//
void setupRelayPins(void) {
  DDRD  |= (1 << PIN_HEATER); // output
//  PORTD &= ~(1 << PIN_HEATER); // off ACTIVE HIGH
  PORTD |=  (1 << PIN_HEATER); // off ACTIVE LOW
}

void killRelayPins(void) {
  Timer1.stop();
  FlexiTimer2::stop();
//  PORTD &= ~(1 << PIN_HEATER);  // off ACTIVE HIGH
  PORTD |=  (1 << PIN_HEATER); // off ACTIVE LOW
}

// ----------------------------------------------------------------------------
// wave packet control: only turn the solid state relais on for a percentage 
// of complete sinusoids (i.e. 1x 360°)

typedef struct Channel_s {
  volatile uint8_t target; // percentage of on-time
  int32_t          startW;  // start of window (in zerocrossticks)
  int32_t          next;   // when the next change in output shall occur  
  bool             action; // hi/lo active
  uint8_t pin;             // io pin of solid state relais
} Channel_t;

Channel_t heaterChannel =   { 0, 0, 0, false, PIN_HEATER }; // PD2 == Arduino Pin 2

// ----------------------------------------------------------------------------
// Zero Crossing ISR

void zeroCrossingIsr(void) {

  zeroCrossTicks++;

  // shift on/off window right
  if ((zeroCrossTicks - heaterChannel.startW) > 100) {
    heaterChannel.startW += 100;
  }

  // If active turn on for a number of interrupts until target reached then turn off for remainder

  if (heaterChannel.target > 0) {
    if ((zeroCrossTicks - heaterChannel.startW) <= heaterChannel.target) {
      PORTD &= ~(1 << heaterChannel.pin); // less than target, turn on
//      PORTD |= (1 << heaterChannel.pin); // less than target, turn on
    } else {
      PORTD |= (1 << heaterChannel.pin);  // greater than target, turn off
//      PORTD &= ~(1 << heaterChannel.pin);  // greater than target, turn off
    }
  } else {
    PORTD |= (1 << heaterChannel.pin);    // ZERO so turn off
//    PORTD &= ~(1 << heaterChannel.pin);    // ZERO so turn off
  }
}

// ----------------------------------------------------------------------------
// timer interrupt handling

void timerIsr(void) { // ticks with 100µS

  // handle encoder + button
  if (!(timerTicks % 10)) {
    Encoder.service(); // needs to be called once per millisecond
  }
  timerTicks++;
}

// ----------------------------------------------------------------------------

void abortWithError(int error) {

  killRelayPins();
  
  tft.setTextColor(ST7735_WHITE, ST7735_RED);
  tft.fillScreen(ST7735_RED);
  tft.setCursor(10, 10);
  
  if (error < 9) {
    tft.println("Thermocouple Error");
    tft.setCursor(10, 30);
    switch (error) {
      case 0b001:
        tft.println("Open Circuit");
        break;
      case 0b010:
        tft.println("GND Short");
        break;
      case 0b100:
        tft.println("VCC Short");
        break;
    }
    tft.setCursor(10, 60);
    tft.println("Power off,");
    tft.setCursor(10, 75);
    tft.println("check connections");
  }
  else {
    tft.println("Temperature"); 
    tft.setCursor(10, 30);
    tft.println("following error");
    tft.setCursor(10, 45);
    tft.print("during ");
    tft.println((error == 10) ? "heating" : "cooling");
  }

  while (1) { //  stop
    ;
  }
}

// ----------------------------------------------------------------------------

void displayThermocoupleData(struct Thermocouple* input) {
  switch (input->stat) {
    case 0:
      printDouble(input->temperature);
      tft.print("\367C");
      break;
    case 1:
      tft.print("---");
      break;
  }
}

// ----------------------------------------------------------------------------

void alignRightPrefix(uint16_t v) {
  if (v < 1e2) tft.print(' '); 
  if (v < 1e1) tft.print(' ');
}

uint8_t pxPerS; // per tenths seconds
uint8_t pxPerC; // per hundredths degree

uint16_t xOffset; // used for wraparound on x axis

#define Y_STATUS 118 // start of botton status
#define Y_OFFSET  36 // end of top status

#define DURATION_HEADROOM 20
#define TEMPERATURE_HEADROOM 20

// ----------------------------------------------------------------------------
// 
// Refresh screen
//
// ----------------------------------------------------------------------------

void updateProcessDisplay() {
  const uint16_t h = Y_STATUS - Y_OFFSET - 2; 
  const uint16_t w = 160;
  uint16_t       dx, dy;
  uint16_t       y = 2;
  double         tmp;

  // header & initial view
  tft.setTextColor(ST7735_WHITE, ST7735_BLUE);

  if (!initialProcessDisplay) {
    initialProcessDisplay = true;

    tft.fillScreen(ST7735_WHITE);
    tft.fillRect(0, 0, tft.width(), menuItemHeight, ST7735_BLUE);
    tft.setCursor(2, y);
#ifndef PIDTUNE
    tft.print("Profile ");
    tft.print(activeProfileId);
#else
    tft.print("Tuning ");
#endif

    calcProfile(); // work out duration and peak temp

    tmp = (h * 100)/ ((uint8_t)profilePeak + TEMPERATURE_HEADROOM);
    pxPerC = (uint8_t)tmp;
    
    tmp = profileDuration + DURATION_HEADROOM; // expand X scale to be slightly longer than profile
    tmp = (w * 100) / tmp; 
    pxPerS = (uint8_t)tmp;

    Serial.print("profilePeak=");Serial.println(profilePeak);
    Serial.print("profileDuration=");Serial.println(profileDuration);
    Serial.print("pxPerC=");Serial.println(pxPerC);
    Serial.print("pxPerS=");Serial.println(pxPerS);
    
    // 50°C grid
    int16_t t = (uint16_t)profilePeak + TEMPERATURE_HEADROOM;
    for (uint16_t tg = 0; tg < t; tg += 50) {
      uint16_t l = h - (tg * pxPerC / 100) + Y_OFFSET;
      tft.drawFastHLine(0, l, 160, tft.Color565(0xe0, 0xe0, 0xe0));
    }
  } // end of first time display 

  // elapsed time
  uint16_t elapsed = (zeroCrossTicks - startCycleZeroCrossTicks) / ticksPerSec;
  tft.setCursor(125, y);
  alignRightPrefix(elapsed); 
  tft.print(elapsed);
  tft.print("s");

  y += menuItemHeight + 2;

  tft.setCursor(2, y);
  tft.setTextColor(ST7735_BLACK, ST7735_WHITE);

  // temperature
  tft.setTextSize(2);
  alignRightPrefix((int)A.temperature);
  displayThermocoupleData(&A);
  tft.setTextSize(1);

  Serial.print(","); // One column over if treated as CSV
  Serial.print(elapsed); Serial.print(", "); 
  Serial.print(Setpoint); Serial.print(", "); Serial.println(A.temperature);

#ifndef PIDTUNE
  // current state
  y -= 2;
  tft.setCursor(95, y);
  tft.setTextColor(ST7735_BLACK, ST7735_GREEN);
  
  switch (currentState) {
    case Phase1:
    case Phase2:
    case Phase3:
    case Phase4:
    case Phase5:
    case CoolDown:
      tft.print(activeProfile.Phases[currentState].Name);
      break;
      
    #define casePrintState(state) case state: tft.print(#state); break;
    casePrintState(Complete);
    default: tft.print((uint8_t)currentState); break;
  }
  tft.print("        "); // lazy: fill up space

  tft.setTextColor(ST7735_BLACK, ST7735_WHITE);
#endif

  // set point
  y += 10;
  tft.setCursor(95, y);
  tft.print("Sp:"); 
  alignRightPrefix((int)Setpoint); 
  printDouble(Setpoint);
  tft.print("\367C  ");

  // draw temperature curves

  if (xOffset >= elapsed) {
    xOffset = 0;
  }

  do { // x with wrap around
    dx = ((elapsed - xOffset) * pxPerS) / 100;
    if (dx > w) {
      xOffset = elapsed;
    }
  } while(dx > w);

  // temperature setpoint
  dy = h - ((uint16_t)Setpoint * pxPerC / 100) + Y_OFFSET;
  tft.drawPixel(dx, dy, ST7735_BLUE);

  // actual temperature
  dy = h - ((uint16_t)A.temperature * pxPerC / 100) + Y_OFFSET;
  tft.drawPixel(dx, dy, ST7735_RED);

  // bottom line
  y = Y_STATUS;

  // set values
  tft.setCursor(10, y);
  tft.print("\xef");
  alignRightPrefix((int)heaterValue); 
  tft.print((int)heaterValue);
  tft.print('%');

  tft.print("     \x12 "); // alternative: \x7f
  printDouble(rampRate);
  tft.print("\367C/s    ");
}

// ----------------------------------------------------------------------------

void setup() {
  setupRelayPins();
  
  // setup /CS line for thermocouple
  pinMode(THERMOCOUPLE1_CS, OUTPUT);
  digitalWrite(THERMOCOUPLE1_CS, HIGH);

  // setup /CS line for display
  pinMode(LCD_CS, OUTPUT);
  digitalWrite(LCD_CS, HIGH);

  pinMode(SOUNDER, OUTPUT);
  digitalWrite(SOUNDER, LOW);
  
  Serial.begin(57600);

  tft.initR(INITR_BLACKTAB);
  tft.setTextWrap(false);
  tft.setTextSize(1);
  tft.setRotation(1);

  if (firstRun()) {
    factoryReset();
    loadParameters(0);
  } 
  else {
    loadLastUsedProfile();
  }

  Timer1.initialize(100);
  Timer1.attachInterrupt(timerIsr);
  Timer1.start();
  FlexiTimer2::set(TICK_PERIOD, 1.0/1000.0, zeroCrossingIsr);
  FlexiTimer2::start();

#ifdef WITH_SPLASH
  // splash screen
  tft.fillScreen(ST7735_BLUE);
  tft.setCursor(10, 30);
  tft.setTextSize(2);
  tft.setTextColor(ST7735_WHITE);
  tft.print("Reflow");
  tft.setCursor(10, 48);
  tft.print("Controller");
  tft.setTextSize(1);
  tft.setCursor(52, 67);
  tft.print("v"); tft.print(ver);
  tft.setTextColor(ST7735_BLACK);

  digitalWrite(SOUNDER, HIGH);
  delay(250);
  digitalWrite(SOUNDER, LOW);
  
  delay(2000);
#endif

  tft.fillScreen(ST7735_WHITE);
  tft.setTextColor(ST7735_BLACK, ST7735_WHITE);

  readThermocouple(&A);

  if (A.stat != 0) {
    abortWithError(A.stat);
  }

  // initialize moving average filter
  for(int i = 0; i < NUMREADINGS; i++) {
    airTemp[i].temp = A.temperature;
  }

  loadPID();

  PID.SetOutputLimits(0, 100); // max output 100%
  heaterChannel.startW = zeroCrossTicks;
  PID.SetMode(AUTOMATIC);

  menuExit(Menu::actionDisplay); // reset to initial state
  Engine.navigate(&miCycleStart);
  currentState = Settings;
  menuUpdateRequest = true;
}

// ----------------------------------------------------------------------------
/* moving average
    int samples[8];

    total -= samples[i];
    samples[i] = A.temperature; // new value
    total += samples[i];

    i = (i + 1) % 8; // next position
    average = total >> 3; // == div by 8 */
// ----------------------------------------------------------------------------

uint32_t lastRampTicks;

void updateRampSetpoint(double rate, bool rising, int limitC) {
  if (zeroCrossTicks > lastRampTicks) {
    Setpoint += (rate / ticksPerSec * (zeroCrossTicks - lastRampTicks)) * (rising ? 1 : -1);
    lastRampTicks = zeroCrossTicks;
  }
  if (rising) {
    Setpoint = (Setpoint > limitC) ? limitC : Setpoint;
  } else {
    Setpoint = (Setpoint < limitC) ? limitC : Setpoint;
  }
}

// ----------------------------------------------------------------------------

#ifdef PIDTUNE
void toggleAutoTune() {
 if(currentState != Tune) { //Set the output to the desired starting frequency.
    currentState = Tune;

    Output = aTuneStartValue;
    PIDTune.SetNoiseBand(aTuneNoise);
    PIDTune.SetOutputStep(aTuneStep);
    PIDTune.SetLookbackSec((int)aTuneLookBack);
  }
  else { // cancel autotune
    PIDTune.Cancel();
    currentState = CoolDown;
  }
}
#endif // PIDTUNE

// ----------------------------------------------------------------------------

uint8_t thermocoupleErrorCount;
#define TC_ERROR_TOLERANCE 2 // allow for n consecutive errors due to noisy power supply before bailing out

// ----------------------------------------------------------------------------
// MAIN LOOP
// ----------------------------------------------------------------------------

void loop(void) 
{
  // --------------------------------------------------------------------------
  // handle encoder
  // --------------------------------------------------------------------------

  encMovement = Encoder.getValue();
  if (encMovement) {
    encAbsolute += encMovement;
    if (currentState == Settings) {
      Engine.navigate((encMovement > 0) ? Engine.getNext() : Engine.getPrev());
      menuUpdateRequest = true;
    }
  }

  // --------------------------------------------------------------------------
  // handle button
  // --------------------------------------------------------------------------

  switch (Encoder.getButton()) {
    case ClickEncoder::Clicked:
      if (currentState == Complete) { // at end of cycle; reset at click
        menuExit(Menu::actionDisplay); // reset to initial state
        Engine.navigate(&miCycleStart);
        currentState = Settings;
        menuUpdateRequest = true;
      }
      else if (currentState > Complete) {
        menuUpdateRequest = true;
        Engine.invoke();
      }
      else if (currentState < Complete) {
        currentState = CoolDown;
        PID.SetMode(MANUAL);
        Output = 0;
      }
      break;

    case ClickEncoder::DoubleClicked:
      if (currentState < UIMenuEnd) {
        if (Engine.getParent() != &miExit) {
          Engine.navigate(Engine.getParent());
          menuUpdateRequest = true;
        }
      }
      break;
  }

  // --------------------------------------------------------------------------
  // update current menu item while in edit mode
  // --------------------------------------------------------------------------

  if (currentState == Edit) {
    if (Engine.currentItem != &Menu::NullItem) {
      Engine.executeCallbackAction(Menu::actionDisplay);      
    }
  }

  // --------------------------------------------------------------------------
  // handle menu update
  // --------------------------------------------------------------------------

  if (menuUpdateRequest) {
    menuUpdateRequest = false;
    if (currentState < UIMenuEnd && 
        !encMovement && 
        currentState != Edit && 
        previousState != Edit) 
    { // clear menu on child/parent navigation
      tft.fillScreen(ST7735_WHITE);
    }  
    Engine.render(renderMenuItem, menuItemsVisible);
  }

  // --------------------------------------------------------------------------
  // track state changes
  // --------------------------------------------------------------------------

  if (currentState != previousState) {
    stateChangedTicks = zeroCrossTicks;
    stateChanged = true;
    previousState = currentState;
  }

  // --------------------------------------------------------------------------
  // Controller cycle
  // --------------------------------------------------------------------------

  if (zeroCrossTicks - lastUpdate >= CYCLE_PERIOD) {
    uint32_t deltaT = zeroCrossTicks - lastUpdate;
    lastUpdate      = zeroCrossTicks;

    // ------------------------------------------------------------------------
    // Temperature
    // ------------------------------------------------------------------------
    
    readThermocouple(&A); // should be sufficient to read it every 250ms or 500ms
  
    if (A.stat > 0) {
      thermocoupleErrorCount++;
    }
    else {
      thermocoupleErrorCount = 0;
    }

    if (thermocoupleErrorCount >= TC_ERROR_TOLERANCE) {
      abortWithError(A.stat);
    }

    // ------------------------------------------------------------------------
    // rolling average of the temp T1 and T2
    // ------------------------------------------------------------------------

    totalT1           -= readingsT1[index];         // subtract the last reading
    readingsT1[index] = A.temperature;
    totalT1           += A.temperature;             // add the reading to the total
    index             = (index + 1) % NUMREADINGS;  // next position
    averageT1         = totalT1 / NUMREADINGS;      // calculate the average temp

    // need to keep track of a few past readings in order to work out rate of rise
    for (int i = 1; i < NUMREADINGS; i++) { // iterate over all previous entries, moving them backwards one index
      airTemp[i - 1].temp = airTemp[i].temp;
      airTemp[i - 1].ticks = airTemp[i].ticks;     
    }

    airTemp[NUMREADINGS - 1].temp = averageT1; // update the last index with the newest average
    airTemp[NUMREADINGS - 1].ticks = deltaT;

    // ------------------------------------------------------------------------
    // calculate rate of temperature change
    // ------------------------------------------------------------------------

    uint32_t collectTicks;
    for (int i = 0; i < NUMREADINGS; i++) {
      collectTicks += airTemp[i].ticks;
    }
    rampRate = ((airTemp[NUMREADINGS - 1].temp - airTemp[0].temp) / collectTicks) * ticksPerSec;

    Input = airTemp[NUMREADINGS - 1].temp; // update the variable the PID reads

    // display update
    if (zeroCrossTicks - lastDisplayUpdate > DISPLAY_PERIOD) {
      lastDisplayUpdate = zeroCrossTicks;
      if ((currentState < Idle) or  // Active or Tuning
          (currentState == Tune)){
        updateProcessDisplay();
      }
    }

  // --------------------------------------------------------------------------
  // State machine
  // --------------------------------------------------------------------------

    switch (currentState) {

      // ----------------------------------------------------------------------

#ifndef PIDTUNE
      case Phase1:
      case Phase2: //Soak
      case Phase3: //Rampup
      case Phase4: //Peak
      case Phase5: //Rampdown

        if (stateChanged) {
          lastRampTicks        = zeroCrossTicks;
          stateChanged         = false;
          heaterChannel.startW = zeroCrossTicks;
          Setpoint             = activeProfile.Phases[currentState].StartTemperatureC;

          // Indicate sounder if need a rapid fall by opening the oven door
          
          if (not activeProfile.Phases[currentState].Rising &&
              activeProfile.Phases[currentState].Rate >= 2.0) {
            digitalWrite(SOUNDER, HIGH); 
          } else {
            digitalWrite(SOUNDER, LOW); 
          }
        }

        updateRampSetpoint(activeProfile.Phases[currentState].Rate,
                           activeProfile.Phases[currentState].Rising,
                           activeProfile.Phases[currentState].EndTemperatureC);

        // --------------------------------------------------------------------
        // check if Phase end temperature reached
        // check if Phase time complete
        // --------------------------------------------------------------------

        nextState = currentState;
        if ((zeroCrossTicks - stateChangedTicks) >= activeProfile.Phases[currentState].DurationS * ticksPerSec) {
          nextState = activeProfile.Phases[currentState].NextState;
        }

        currentState = nextState;
        break;
#endif

      case CoolDown:
        if (stateChanged) {
          stateChanged = false;
          Setpoint = idleTemp;
        }

        if (Input < (idleTemp + 5)) {
          currentState = Complete;
          PID.SetMode(MANUAL);
          Output = 0;
        }
      
#ifdef PIDTUNE
      case Tune:
        {
          Setpoint = 160.0;
          int8_t val = PIDTune.Runtime();

          if (val != 0) {
            currentState = CoolDown;
          }
          
          if (currentState != Tune) { // we're done, set the tuning parameters
            heaterPID.Kp = PIDTune.GetKp();
            heaterPID.Ki = PIDTune.GetKi();
            heaterPID.Kd = PIDTune.GetKd();
            
            savePID();

            tft.setCursor(40, 40);
            tft.print("Kp: "); tft.print((uint32_t)(heaterPID.Kp * 100));
            Serial.println();
            Serial.print("Kp: "); Serial.print((uint32_t)(heaterPID.Kp * 100));
            Serial.println();
            tft.setCursor(40, 52);
            tft.print("Ki: "); tft.print((uint32_t)(heaterPID.Ki * 100));
            Serial.print("Ki: "); Serial.print((uint32_t)(heaterPID.Ki * 100));
            Serial.println();
            tft.setCursor(40, 64);
            tft.print("Kd: "); tft.print((uint32_t)(heaterPID.Kd * 100));
            Serial.print("Kd: "); Serial.print((uint32_t)(heaterPID.Kd * 100));
            Serial.println();
          }
        }
        break;
#endif
    }
  }

  // safety check that we're not doing something stupid. 
  // if the thermocouple is wired backwards, temp goes DOWN when it increases
  // during cooling, the t962a lags a long way behind, hence the hugely lenient cooling allowance.
  // both of these errors are blocking and do not exit!
  //if (Setpoint > Input + 50) abortWithError(10); // if we're 50 degree cooler than setpoint, abort
  //if (Input > Setpoint + 50) abortWithError(20); // or 50 degrees hotter, also abort
  
  PID.Compute();
  heaterValue = Output;

  heaterChannel.target = heaterValue;
}

// ----------------------------------------------------------------------------

void memoryFeedbackScreen(uint8_t profileId, bool loading) {
  tft.fillScreen(ST7735_YELLOW);
  tft.setTextColor(ST7735_RED);
  tft.setCursor(10, 50);
  tft.print(loading ? "Loading" : "Saving");
  tft.print(" profile ");
  tft.print(profileId);  
}

// ----------------------------------------------------------------------------

void saveProfile(unsigned int targetProfile, bool quiet) {
#ifndef PIDTUNE
  activeProfileId = targetProfile;

  if (!quiet) {
    memoryFeedbackScreen(activeProfileId, false);
  }
  saveParameters(activeProfileId); // activeProfileId is modified by the menu code directly, this method is called by a menu action

  if (!quiet) delay(500);
#endif
}

// ----------------------------------------------------------------------------

void loadProfile(unsigned int targetProfile) {

  memoryFeedbackScreen(activeProfileId, true);
  bool ok = loadParameters(targetProfile);

  if (!ok) {
    tft.fillScreen(ST7735_RED);
    tft.setTextColor(ST7735_WHITE);
    tft.setCursor(10, 40);
    tft.print("Checksum error!");
    tft.setCursor(10, 60);
    tft.print("Review profile.");
    delay(5000);
  }

  // save in any way, as we have no undo
  activeProfileId = targetProfile;
  saveLastUsedProfile();
}

// ----------------------------------------------------------------------------

#define WITH_CHECKSUM 1

bool saveParameters(uint8_t profile) {
#ifndef PIDTUNE
  uint16_t offset = profile * sizeof(ReflowProfile);

  calcProfile(); // fix Rising flag

#ifdef WITH_CHECKSUM
   activeProfile.checksum = crc8((uint8_t *)&activeProfile, sizeof(ReflowProfile) - sizeof(uint8_t));
#endif

  do {
  } while (!(eeprom_is_ready()));
  eeprom_write_block(&activeProfile, (void *)offset, sizeof(ReflowProfile));
#endif
  
  return true;
}

// ----------------------------------------------------------------------------

bool loadParameters(uint8_t profile) {
  uint16_t offset = profile * sizeof(ReflowProfile);

  do {
  } while (!(eeprom_is_ready()));
  eeprom_read_block(&activeProfile, (void *)offset, sizeof(ReflowProfile));

  calcProfile(); // fix Rising flag and get peak and duration

#ifdef WITH_CHECKSUM
  return (activeProfile.checksum == crc8((uint8_t *)&activeProfile, sizeof(ReflowProfile) - sizeof(uint8_t)));
#else
  return true;  
#endif
}

// ----------------------------------------------------------------------------

void calcProfile() {
  profilePeak     = 0.0;
  profileDuration = 0;
  for (uint8_t i=0; i<NUM_PHASES; i++) {
    activeProfile.Phases[i].Rising = true;
    if (activeProfile.Phases[i].EndTemperatureC < activeProfile.Phases[i].StartTemperatureC) {
      activeProfile.Phases[i].Rising = false;
    }
    profileDuration += activeProfile.Phases[i].DurationS;
    profilePeak = (activeProfile.Phases[i].EndTemperatureC > profilePeak) ? activeProfile.Phases[i].EndTemperatureC : profilePeak;
  }
}

// ----------------------------------------------------------------------------

bool savePID() {
  do {
  } while (!(eeprom_is_ready()));
  
  eeprom_write_block(&heaterPID, (void *)offsetPidConfig, sizeof(PID_t));
  
  return true;
}

bool loadPID() {
  do {
  } while (!(eeprom_is_ready()));
  
  eeprom_read_block(&heaterPID, (void *)offsetPidConfig, sizeof(PID_t));
  
  return true;  
}

// ----------------------------------------------------------------------------

bool firstRun() { 
#ifndef PIDTUNE
  // if all bytes of a profile in the middle of the eeprom space are 255, we assume it's a first run
  unsigned int offset = 4 * sizeof(ReflowProfile);

  for (uint16_t i = offset; i < offset + sizeof(ReflowProfile); i++) {
    if (EEPROM.read(i) != 255) {
      return false;
    }
  }
#endif
  return true;
}

// ----------------------------------------------------------------------------

void makeDefaultProfile() {
   memcpy(&activeProfile, &defaultProfile, sizeof(defaultProfile));
}

// ----------------------------------------------------------------------------

void factoryReset() {
#ifndef PIDTUNE

  tft.fillScreen(ST7735_RED);
  tft.setTextColor(ST7735_WHITE);
  tft.setCursor(10, 50);
  tft.print("Clear EEPROM...");

  for (int i = 0 ; i < EEPROM.length() ; i++) {
    EEPROM.write(i, 0);
  }
  
  makeDefaultProfile();

  tft.fillScreen(ST7735_RED);
  tft.setTextColor(ST7735_WHITE);
  tft.setCursor(10, 50);
  tft.print("Resetting profiles...");
#endif

  // then save the same profile settings into all slots
  for (uint8_t i = 0; i <= maxProfiles; i++) {
    saveParameters(i);
  }

  heaterPID.Kp =  DEFAULT_HEATER_Kp; 
  heaterPID.Ki =  DEFAULT_HEATER_Ki;
  heaterPID.Kd =  DEFAULT_HEATER_Kd;
  savePID();

  activeProfileId = 0;
  saveLastUsedProfile();

  delay(500);
}

// ----------------------------------------------------------------------------

void saveLastUsedProfile() {
  EEPROM.write(offsetProfileNum, (uint8_t)activeProfileId & 0xff);
}

// ----------------------------------------------------------------------------

void loadLastUsedProfile() {
  activeProfileId = EEPROM.read(offsetProfileNum) & 0xff;
  loadParameters(activeProfileId);
}

// ----------------------------------------------------------------------------

