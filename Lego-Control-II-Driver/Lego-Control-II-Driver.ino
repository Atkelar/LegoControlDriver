/*
 * LEGO Control II - serial to interface adapter
 * 
 * PIN Layout for Arduino Uno, Nano, Mini:
 * 
 * We need: serial pins, two interrupt driven inputs and six outputs.
 * Ideally, all outputs support PWM. On the Arduino NANO, PIN 3 overlaps 
 * between PWM and interrupts. So we compromise by reversing the PWM signal
 * on reverse directions instead. This way, one PWM per motor channel is
 * enough.
 *                    Arduino PIN  ->  LEGO channel (pin)
 * PWM output:        9,10,5       ->   0, 2, 4
 * Digital output:    6,7,8        ->   1, 3, 5
 * Input signal:      2,3          ->   6, 7
 * 
 * Motor channel to Arduino PINs:
 *    A: 6-9
 *    B: 7-10
 *    C: 8-5
 *    
 *
 * The outputs on the LEGO channels 0-5 can be paired up as motor channels
 * or turned on/off individually for lamps. A nice feature would also be a
 * lamp-dimmer via PWM on the PWM channels.
 * 
 * The two input channels only support input in general, via the sensor 
 * bricks (or similar hacks) So only the outputs need to be configured/calibrated.
 *
 * The overall protocol for the chip is a serial IO, driven via ASCII control commands.
 * A control command initiates a specific action on the controller, via a command queue.
 * Note that the queue is handled according to the timer of the arduino. That way we
 * are not depending on the driving PC's accuracy and reaction times, but can run motors
 * for specific times at the lowest level. The downside is that individual commands
 * cannot be interrupted due to the delay function.
 *
 * To use a motor channel (0-2) we need to feed in calibration data first. This is generated
 * by the "cal" command per channel. The data is not stored in the arduino, to facilitate 
 * software based management of different motors/controllers/etc. The calibration data will
 * return - if successful - a list of values to be passed back into the "init motor" command.
 * 
 * 
 *
 */

// pre-include initialization... according to TimerInterrupt library.

// If we use a MEGA2560 based board, we have to use Timer1, the UNO
// requires Timer2 - otherwise the selected PWM channels won't work.
#if ARDUINO_AVR_MEGA2560
#define USE_TIMER_1     true
#define MYITIMER ITimer1
#elif ARDUINO_AVR_UNO
#define USE_TIMER_2     true
#define MYITIMER ITimer2
#endif

#include <TimerInterrupt.h>
#include <ISR_Timer.h>


// overall "variable" constants for different compilations.
// The version will be reported to the host during reset
#define DRIVERVERSIONSTRING "1.1"
// Maximum length of input commands. i.e. length of byte buffer for one command line.
// Longer commands will lead to an "overlow" error and will be ignored to prevent
// overflows in the data structures.
// The "mot" command takes calibration data for up to three motors, which is 160 chars,
// so we round up to the nearest binary round number, but it should NOT be less then 160!
#define MAXCOMMANDLENGTH 255  


// The buffer for incoming command text and the current length/number of received bytes.
char CommandBuffer[MAXCOMMANDLENGTH + 2]; // for safety reason, add two bytes spare...
byte CurrentCommandLength = 0;

// Initializing the command completed flag to true will trigger the initial prompt, 
// pretending to have complted a non-existing command.
bool CommandComplete = true;

// Command line overflow - whenever the incoming command was longer than the buffer;
bool CommandWasOverflow = false;
#define WDBG(x) Serial.println(x);

#if ARDUINO_AVR_UNO 
  // uno version...
  #define CHANNEL_0 9
  #define CHANNEL_1 6
  #define CHANNEL_2 10
  #define CHANNEL_3 7
  #define CHANNEL_4 5
  #define CHANNEL_5 8
  #define CHANNEL_6 2
  #define CHANNEL_7 3
#else

#if ARDUINO_AVR_MEGA2560
  // MEGA version (test/debugging, should support same as UNO, just to separate it out in code...)
  #define CHANNEL_0 9
  #define CHANNEL_1 6
  #define CHANNEL_2 10
  #define CHANNEL_3 7
  #define CHANNEL_4 5
  #define CHANNEL_5 8
  #define CHANNEL_6 2
  #define CHANNEL_7 3
#else
  // add any other pin mappings here...
  #error Undefined board, please add PIN mappings
#endif

#endif

// internal constants and variables...
volatile int counters[2]; // "stepper" counters for trigger input 1 and 2
//volatile int reportAt[2]; // "stepper counter" report multiplier; report back via serial every N-steps, zero if not used.

// motor direction constants
#define STP 0
#define FWD 1
#define REV 2

// mode numbers are bits for "is valid command in" check, but never set together as active!
#define MODE_INIT 1 // init mode is to configure the controller setup for a project and do calibration.
#define MODE_FREE 2 // free mode just listens to "on/off" commands on the configured channels and provides counter feedback on the requested intervals.
#define MODE_AUTO 4 // auto mode listens to "turn motor X until counter Y reaches N" type of commands.
byte CurrentMode = MODE_INIT;

// Calibration steps for motor speed calibration; index is "motor index", not "channel index".
byte CalibrationSteps[] = {1,25,77,127,179,230,255};
#define CALIBRATIONSTEPCOUNT ((byte)(sizeof(CalibrationSteps) / sizeof(CalibrationSteps[0])))
byte MotorCalibrationData[3][(CALIBRATIONSTEPCOUNT+1)*2]; // per motor channel, forward/reverse.
byte MotorMinimum[3][2];

// motor and ditigal channel mappings
byte InputToMotorMap[2];
byte MotorToInputMap[3];
byte MotorToChannelMap[3];
bool IsDigital[6];
byte DigitalToChannelMap[6];
byte MotorDirection[3];
byte MotorSpeed[3];

int MotorTrap[2];
volatile byte MotorTrapped = 0;

#define UNMAPPED 0xFF

// arrays for easier addressing of Lego Channels, defined above.
int inputs[] = { CHANNEL_6, CHANNEL_7 };
int outputs[] = { CHANNEL_0, CHANNEL_1, CHANNEL_2, CHANNEL_3, CHANNEL_4, CHANNEL_5 };

// timer for reset/timed commands
ISR_Timer ISR_timer;
#define TIMERINTERVAL 50L
bool TimerAvailable = false;
volatile long Ticker = 0L;

void TimerHandler()
{
  Ticker++;
  // TODO: implement this for the "AUTO" mode, to support timed motor controls.
}

void ResetOutputMapping()
{
  InputToMotorMap[0] = UNMAPPED;
  InputToMotorMap[1] = UNMAPPED;
  MotorTrap[0] = 0;
  MotorTrap[1] = 0;
  MotorToInputMap[0] = UNMAPPED;
  MotorToInputMap[1] = UNMAPPED;
  MotorToInputMap[2] = UNMAPPED;
  for (byte i = 0; i < 3; i++)
    MotorToChannelMap[i] = UNMAPPED;
  for (byte i = 0; i < 6; i++)
  {
    IsDigital[i] = true;
    DigitalToChannelMap[i] = i;
  }
}

void FullStop()
{
  // FullStop is a "hard stop" - it will reset all counters and thus mess up servo functionallity.
  // It is prepared to be used in a "bail out" scenario only.
  for(int i= 0;i < 3; i++)
  {
    analogWrite(outputs[i*2],0);
    digitalWrite(outputs[i*2+1],false);
    MotorSpeed[i] = 0;
    MotorDirection[i] = STP;
  }
  for(int i=0;i<2;i++)
    MotorTrap[i] = 0;
  MotorTrapped = 0;
}

void ResetCalibrationData()
{
  for (byte m = 0; m< 3; m++)
  {
    for (byte i = 0; i< CALIBRATIONSTEPCOUNT; i++)
    {
      MotorCalibrationData[m][i] = CalibrationSteps[i];
    }
  }
}

void PrintBannerInfo()
{
  Serial.println(); // make sure the banner ends up on a new line
  Serial.println("LEGO Control-II Driver, Copyright 2021 by Atkelar");
  Serial.print("Version: ");
  Serial.println(DRIVERVERSIONSTRING);
  Serial.print("Timer: ");
  Serial.println(TimerAvailable ? "yes" : "no");
}

// the setup routine runs once when you press reset:
void setup() {
  // initialize serial communication at 9600 bits per second:
  Serial.begin(9600);
  for (int i = 0; i < 6; i++)
    pinMode(outputs[i], OUTPUT);

  // interrupt reference: https://www.arduino.cc/reference/de/language/functions/external-interrupts/attachinterrupt/
  counters[0]=0;
  counters[1]=0;
  MotorTrap[0] = 0;
  MotorTrap[1] = 0;
  pinMode(inputs[0], INPUT_PULLUP); // tried without pullup, but got wonky results. Better keep that, or we need to add a resistor in hardware...
  pinMode(inputs[1], INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(inputs[0]), Count0, CHANGE);
  attachInterrupt(digitalPinToInterrupt(inputs[1]), Count1, CHANGE);

  
  ResetCalibrationData();
  ResetOutputMapping();
  FullStop();

  // ITimer1 might take away PWM on pin 11 which forced us to use PIN5 - more debugging needed, 
  // but it works like this and should be compatible between UNO/MEGA...
  // On the UNO, ITimer1 seems to block PWM on some pins and crash...
  MYITIMER.init();
  if (!MYITIMER.attachInterruptInterval(TIMERINTERVAL, TimerHandler))
  {
    TimerAvailable = false;
    Serial.println("err-timer");
  }
  else
  {
    TimerAvailable = true;
  }
  PrintBannerInfo();
}

void TrapMotor(byte mIndex, int limit)
{
  // sets up a servo operation on motor "mIndex" with the number of ticks.
  byte counterIndex = MotorToInputMap[mIndex];
  if (counterIndex != UNMAPPED && limit > 0)
  {
    counters[counterIndex] = 0;
    MotorTrap[counterIndex] = limit;
  }
}

void RunMotor(byte mIndex, byte dir, byte pct = 100);

inline void HandleCounter(byte n)
{
  // count channel 0 or 1 (6 or 7 on Lego) and react to "trapped" motor in servo mode.
  counters[n]++;
  if (InputToMotorMap[n] != UNMAPPED && MotorTrap[n] > 0 && counters[n] >= MotorTrap[n])
  {
    MotorTrap[n] = 0;
    RunMotor(InputToMotorMap[n], STP);
    MotorTrapped |= (byte)(1 << InputToMotorMap[n]);
  }
}

void Count0()
{
  // interrupt handler for input channel 6
  HandleCounter(0);
}
void Count1()
{
  // interrupt handler for input channel 7
  HandleCounter(1);
}

void RunMotorRAW(byte n, byte dir, byte fSpeed = 255)
{
  // Run motor output N (NOT mapped motor output, but Lego channel A-B-C as number!)
  // in the specified direction and raw PWM speed.
  // This is used as the backend of the calibrated motor code 
  // and for PWM calibration.
  if (n > 2)
    return;
  int m1 = outputs[n*2];
  int m2 = outputs[n*2+1];
  byte b;
  switch(dir)
  {
    case FWD:
      b=false;
      break;
    case REV:
      b = true;
      fSpeed = 255-fSpeed; // invert speed for reverse...
      break;
    default:
      b=false;
      fSpeed = 0;
  }
  /*
  // DIAGNOSTIC output for motor control
  Serial.print("Running motor ");
  Serial.print(n);
  Serial.print(" on ");
  Serial.print(m1);
  Serial.print(" and ");
  Serial.print(m2);
  Serial.print(" at ");
  Serial.print(fSpeed);
  Serial.print(" (");
  Serial.print(b ? "on" : "off");
  Serial.println(")");
  */
  // we have PWM on the first of the motor pins only!
  analogWrite(m1, fSpeed);
  digitalWrite(m2, b);
  //digitalWrite(m1, a);
  //digitalWrite(m1+1, b);
}

byte CorrectPWMBaseValue(byte value, byte minWorkingValue)
{
  // Compute Base (min) value for PWM duty cycle for a specific "known" minimum.

  // shift (proportionally) so that the result is "n" between minWorkingValue and 255 instead of 0 and 255.
  int newN = value;
  newN *= (256-minWorkingValue);
  newN /= 256;
  newN += minWorkingValue;
  if (newN >= 254)  // make sure rounding errors yield 100% and no overflow!
    newN = 255;
  return (byte)newN;
}

byte CorrectPWMValueForCalibration(byte mIndex, byte dir, byte value)
{
  if (value == 0)
    return 0;

  byte baseIndex = CALIBRATIONSTEPCOUNT * (dir == REV ? 1 : 0);
  for (byte i = 0; i < CALIBRATIONSTEPCOUNT; i++)
  {
    if (value == CalibrationSteps[i]) // exact match, don't calculate (should cover 1/255 case)
    {
      return MotorCalibrationData[mIndex][i + baseIndex];
    }
    if (value < CalibrationSteps[i])  // found it between i-1 and i...
    {
      int n = (int)((value - CalibrationSteps[i - 1]) * 256 / (int)(CalibrationSteps[i] - CalibrationSteps[i-1]));
      int h = (int)((MotorCalibrationData[mIndex][i + baseIndex] - MotorCalibrationData[mIndex][i + baseIndex - 1]) * n);
      value = (byte)(MotorCalibrationData[mIndex][i + baseIndex - 1] + (h / 256));
      return value;
    }
  }
  return 0; // catch "not found in list" case...
}

void RunMotor(byte mIndex, byte dir, byte pct)
{
  // pct: 1-100
  // dir: FWD/REV/STP
  // n: 0-2

  if (mIndex > 2 || MotorToChannelMap[mIndex] == UNMAPPED)
    return;
  if (dir==STP)
    RunMotorRAW(MotorToChannelMap[mIndex], STP);
  else
  {
    int xVal = pct;
    xVal *= 255;
    xVal /= 100;
    
    RunMotorRAW(MotorToChannelMap[mIndex], dir, CorrectPWMBaseValue(CorrectPWMValueForCalibration(mIndex, dir, xVal), MotorMinimum[mIndex][dir == FWD ? 0 : 1]));
  }
  MotorSpeed[mIndex] = pct;
  MotorDirection[mIndex] = dir;
}

void PortSet(byte n, byte on)
{
  if (n > 5)
    return;
  digitalWrite(outputs[n], on);
}

int ReadCounter(byte n)
{
  if (n>1)
    return 0;
  int r = 0;
  noInterrupts();
  r = counters[n];
  counters[n] = 0;
  interrupts();
  return r;
}

/*
 * RAW test code for motor pins, not used in production version...
 * 
void TestRunMotor()
{
  RunMotorRAW(0,FWD);
  PortSet(2,true);
  delay(500);
  RunMotorRAW(0,STP);
  PortSet(2,false);
  delay(500);
  RunMotorRAW(0,REV,127);
  PortSet(2,true);
  delay(500);
  RunMotorRAW(0,STP);
  PortSet(2,false);
  delay(500);
  RunMotorRAW(0,FWD,200);
  PortSet(2,true);
  delay(500);
  RunMotorRAW(0,STP);
  PortSet(2,false);
  delay(500);
  RunMotorRAW(0,REV,90);
  PortSet(2,true);
  delay(500);
  RunMotorRAW(0,STP);
  PortSet(2,false);
  delay(500);
  Serial.println("loop:");
  Serial.println(ReadCounter(0));
  Serial.println(ReadCounter(1));
}
*/

byte MotorChannelToIndex(char channel, bool handleError = true)
{
  // convert command motor index "A,B,C" to motor channel number
  switch (channel)
  {
    case 'A':
    case 'B':
    case 'C':
      return (byte)(channel) - (byte)'A';
    case 'b':
    case 'c':
    case 'a':
      return (byte)(channel) - (byte)'a';
    default:
      if (handleError)
        Serial.println("err-unkchn");
      return 0xFF;
  }
}

char IndexToMotorChannel(byte index)
{
  // convert command motor channel 0,1,2 to motor channel letter for result printing
  switch(index)
  {
    case 0:
    case 1:
    case 2:
      return 'A' + index;
  }
  return '?';
}

bool RunCalibrationStep(byte n, byte speed, byte index, byte minFwd, byte minRev)
{
  RunMotorRAW(n, STP); // force motor off.
  delay(100);
  ReadCounter(0); // force counter reset
  RunMotorRAW(n, FWD, CorrectPWMBaseValue(speed, minFwd));
  delay(3000);
  RunMotorRAW(n, STP);
  delay(100);
  int count1 = ReadCounter(0);
  if (count1 < 3)
  {
    Serial.print("cal:");
    Serial.print(IndexToMotorChannel(n));
    Serial.print(":fial:f:");
    Serial.println(index);
    return false;
  }
  RunMotorRAW(n, REV, CorrectPWMBaseValue(speed, minRev));
  delay(3000);
  RunMotorRAW(n, STP);
  delay(100);
  int count2 = ReadCounter(0);
  if (count2 < 3)
  {
    Serial.print("cal:");
    Serial.print(IndexToMotorChannel(n));
    Serial.print(":fial:r:");
    Serial.println(index);
    return false;
  }
  Serial.print("cal:");
  Serial.print(IndexToMotorChannel(n));
  Serial.print(":dat:");
  Serial.print(speed);
  Serial.print(":");
  Serial.print(count1);
  Serial.print(":");
  Serial.println(count2);
  return true;
}

bool CheckSpeed(byte n, bool fwd, byte fSpeed)
{
  RunMotorRAW(n, STP); // force motor off.
  delay(75);
  ReadCounter(0); // force counter reset
  RunMotorRAW(n, fwd ? FWD : REV, fSpeed);
  delay(750); // we might need quite a bit of detection time for low speeds...
  RunMotorRAW(n, STP);
  delay(75);
  int count1 = ReadCounter(0);
  return (count1 >= 2); // avoid checking for 1 becaue of "noise".
}

bool FindMinPWM(byte n, byte& fwdMin, byte& revMin)
{
  // try turning the motor at speeds, fwd and rev, using a "homing in" algorithm to find the lowest speed that WILL turn the motor.
  byte fNowHigh = 230;  // we start "high" as the initial test is done at full speed to validate the setup. If THIS doesn't work, it should fail by design.
  byte fNowLow = 1;   // will never expect that, but we have to start somewhere, zero WILL not move, so better start at one.
  if (!CheckSpeed(n, true, 255))
  {
    Serial.print("cal:");
    Serial.print(IndexToMotorChannel(n));
    Serial.println(":fail:s");
    return false;
  }

  /*
   * Algorithm: try half way between the hig/low speed. If it works, set high to current, if it doesn't, set low. Repeat until distance is close enough...
   */
  while (fNowHigh - fNowLow >= 5)
  {
    byte fNow = (fNowHigh - fNowLow) / 2 + fNowLow;
    //Serial.println(fNowHigh);
    //Serial.println(fNowLow);
    //Serial.println(fNow);
    if (CheckSpeed(n, true, fNow))
    {
      //Serial.println("Yay");
      fNowHigh = fNow;
    }
    else
    {
      //Serial.println("Nay");
      fNowLow = fNow;
    }
  }
  byte fFwd = fNowHigh; // make sure to use the LAST GOOD one, as we might exit the loop when fNow was NOT working...
  fNowHigh = 230;
  fNowLow = 1;
  while (fNowHigh - fNowLow >= 5)
  {
    byte fNow = (fNowHigh - fNowLow) / 2 + fNowLow;
    if (CheckSpeed(n, false, fNow))
      fNowHigh = fNow;
    else
      fNowLow = fNow;
  }
  byte fRev = fNowHigh; // make sure to use the LAST GOOD one, as we might exit the loop when fNow was NOT working...

  if (fFwd < 127) 
    fFwd += 10; // just to make sure we can move the motor...
  if (fRev < 127) 
    fRev += 10; // just to make sure we can move the motor...
  
  Serial.print("cal:");
  Serial.print(IndexToMotorChannel(n));
  Serial.print(":min:");
  Serial.print(fFwd);
  Serial.print(":");
  Serial.println(fRev);

  fwdMin = fFwd;
  revMin = fRev;
        
  return true; // gotcha!
}

void MotorPWMCalibration(byte n)
{
  // runs a calibration sequence for motor speed alignment in forward/reverse on channel "n"
  // build up the calibration gizmo with the counting disc feedint into channel 6

  if (n > 2)
    return;
    
  Serial.print("cal:");
  Serial.print(IndexToMotorChannel(n));
  Serial.println(":run");

  byte fFwd, fRev;

  if (!FindMinPWM(n, fFwd, fRev)) return;

  for(byte i = 0; i < CALIBRATIONSTEPCOUNT; i++)
  {
    if (!RunCalibrationStep(n, CalibrationSteps[i], i, fFwd, fRev)) 
    {
      return;
    }
  }

  Serial.print("cal:");
  Serial.print(IndexToMotorChannel(n));
  Serial.println(":end");
}

void SpeedComparison()
{
  // calibration routine for forward/backward speed comparison.

  // Run motor 1 forward for 2 seconds, at speed 127, then 
  // pause and backwards for 2 seconds. Compare counters between.

  // if the motor runs equally fast in both directions, the counters
  // should be equal or at most one off in edge cases.

  ReadCounter(0);  // reset counter
  RunMotorRAW(0,FWD,127);
  delay(2000);
  int count1 = ReadCounter(0);
  delay(100);
  RunMotorRAW(0,REV,150);
  delay(2000);
  RunMotorRAW(0,STP);
  int count2 = ReadCounter(0);
  Serial.print("Counter A=");
  Serial.print(count1);
  Serial.print(" Counter B=");
  Serial.println(count2);
  
  delay(100);
}


void Prompt()
{
  Serial.println("rdy");
}

void HandleSerialInput()
{
  // This is the main input loop.
  // It also will be used to "report" any pending status updates.
  
  noInterrupts();
  byte mt = MotorTrapped;   // set by the interrupt handler in case of "motor has stopped".
  MotorTrapped = 0;
  interrupts();
  if (mt != 0)
  {
    // we have trapped at least one motor since the last loop...
    for (int i=0;i<3;i++)
    {
      if (mt & 1)
      {
        Serial.print("trap:");    // report that motor channel "i" has reached the requested ticks.
        Serial.println(i);
      }
      mt >>= 1;
    }
  }
  if (CommandComplete) 
  {
    // this indicates that the last command was passed back and we are looping
    // around again... start with prompt and reset command buffer.
    Prompt();
    CurrentCommandLength = 0;
    CommandComplete = false;
  }
  while (Serial.available()>0)
  {
    // loop and read char by char...
    char c = Serial.read();
    if (c == 13)  // CR - ignore.
      continue;
    if (c == 10)  // NL
    {
      if (CommandWasOverflow)
      {
        CommandWasOverflow = false;
        CurrentCommandLength = 0;
        Prompt();
      }
      else
      {
        CommandComplete = true;
        CommandBuffer[CurrentCommandLength]=0;
      }
      break;
    }
    if (CurrentCommandLength >= MAXCOMMANDLENGTH || CommandWasOverflow)
    {
      if (!CommandWasOverflow)    // first overflow char triggers error message, all other chars are ignored.
        Serial.println("err-cmdovrflw");
      CommandWasOverflow = true;
    }
    else
    {
      CommandBuffer[CurrentCommandLength] = c;
      CurrentCommandLength++;
    }
  }
}

void SyntaxError2(const char* msg, int n)
{
  Serial.print("err-syn-");
  Serial.print(msg);
  Serial.print("-");
  Serial.println(n);
}

void SyntaxError(const char* msg)
{
  Serial.print("err-syn-");
  Serial.println(msg);
}

void HandlePing()
{
  // Returns a message as a communications check.

  // syntax: ping
  //    or:  ping:message

  // if "message" is specified, it will be returned as the "pong".
  // i.e. the result is either "pong" or "pong:message".
  
  if (CommandBuffer[4] == 0)
    Serial.println("pong");
  else
    if (CommandBuffer[4] != ':')
    {
      SyntaxError("mc");
    }
    else
    {
      Serial.print("pong:");
      Serial.println(CommandBuffer+5);
    }
}

bool IsValidMode(byte mask, bool handleError = true)
{
  if ((CurrentMode & mask) == 0)
  {
    if (handleError)
      Serial.println("err-notavl");
    return false;
  }
  return true;
}

void HandleTime()
{
  // Returns the current timer tick counter for validation.

  // Syntax: time
  
  if (!TimerAvailable)
  {
    Serial.println("err-notavl");
  }
  else
  {
    Serial.print("tck:");
    Serial.println(Ticker);
  }
}

void HandleCal()
{
  // Runs a motor PWM calibration cycle.

  // expected: "cal:#" - where # is A,B,C for motor channel A B C

  // The calibration requires that a sensor brick is connected
  // to input #6 and the detection disk be connected to the motor
  // with the apropriate reduction. Ideally, this is done "in situ"
  // i.e. with the motor in the actual model. Though it is not
  // 100% sure how far the motor will move the gears in either
  // direction, so be careful and have a hand on the stop button!
  
  if (!IsValidMode(MODE_INIT))
    return;
  if (CommandBuffer[3] != ':' || CurrentCommandLength != 5)
    SyntaxError("mcln");
  else
  {
    byte motorchannel = MotorChannelToIndex(CommandBuffer[4]);
    if (motorchannel <= 2)
    {
      MotorPWMCalibration(motorchannel);
    }
  }
}

void HandleCheckPolarity()
{
  // "Polarity Check" for a specific motor channel.
  
  // syntax: "cpol:#" where # is A,B,C for motor channel.
  
  // pulse the motor # for half a second in FORWARD mode to find out
  // what direction it is turning and confirm/flip the leads...
  // FORWARD should turn the motor CLOCKWISE when viewed from head on.
  if (!IsValidMode(MODE_INIT))
    return;

  if (CommandBuffer[4] != ':' || CurrentCommandLength != 6)
    SyntaxError("mc");
  else
  {
    byte motorchannel = MotorChannelToIndex(CommandBuffer[5]);
    if (motorchannel <= 2)
    {
      RunMotorRAW(motorchannel, FWD);
      delay(500);
      RunMotorRAW(motorchannel, STP);
      delay(10);
    }
    else
      Serial.println("err-int");
  }
}

void HandleReset()
{
  // Resets the driver. Stops all outputs and resets all calibration and mapping info.

  // syntax: reset
  
  CurrentMode = MODE_INIT;
  FullStop();
  ResetOutputMapping();
  ResetCalibrationData();
  PrintBannerInfo();
}

int ParseCommandByte(byte cIndex)
{
  if (CommandBuffer[cIndex] == 0 || CommandBuffer[cIndex+1] == 0)
    return -1;
  char temp[3];
  temp[2] = 0;
  temp[0] = CommandBuffer[cIndex];
  temp[1] = CommandBuffer[cIndex+1];
  return strtol(temp, 0, 16);
}

bool ParseMotorSpeedData(byte mIndex, byte &cIndex, byte dir)
{
  int temp = ParseCommandByte(cIndex);
  if(temp<=0) // shold never ever be zero.
  {
    SyntaxError2("mb", cIndex);
    return false;
  }
  cIndex+=2;

  MotorMinimum[mIndex][dir] = (byte)temp;
  for (byte i = 0; i < CALIBRATIONSTEPCOUNT; i++)
  {
    if (CommandBuffer[cIndex] != '~')
    {
      SyntaxError2("td", cIndex);
      return false;
    }
    cIndex++;
    temp = ParseCommandByte(cIndex);
    if(temp<=0) // shold never ever be zero.
    {
      SyntaxError2("btc", cIndex);
      return false;
    }
    cIndex+=2;

    MotorCalibrationData[mIndex][dir * CALIBRATIONSTEPCOUNT + i] = (byte)temp;
  }
  return true;
}

bool ParseMotorMapping(byte mIndex, byte &cIndex)
{
  if (!ParseMotorSpeedData(mIndex, cIndex, 0))
    return false;
  if (CommandBuffer[cIndex] != '~')
  {
    SyntaxError2("td", cIndex);
    return false;
  }
  cIndex++;
  if (!ParseMotorSpeedData(mIndex, cIndex, 1))
    return false;
  return true;
}

bool MapInputToMotor(byte mIndex, byte iIndex)
{
  if (InputToMotorMap[iIndex] != UNMAPPED)
  {
    Serial.println("err-dupmap");
    return false;
  }
  InputToMotorMap[iIndex] = mIndex;
  MotorToInputMap[mIndex] = iIndex;
  return true;
}

void HandleMotorSetup()
{
  // initializes the motor setup: defines channels to be used for motors, 
  // sets calibration data and leaves any "non-used" channels for digital output.

  // syntax: mot{1-3*MOTORDATA}

  // The motor channels (0,1,2) are assigned in order of this command to the
  // specified LEGO output channels.

  // MOTORDATA has the following format:
  //  :C:CALIBRATIONDATA:M

  //   C => Lego Channel number, A,B,C
  //   M => The monitor channel to attach, either 6 or 7 for the two inputs,
  //         or - for "not mapped".
  //   CALIBRATIONDATA is a sequence of two-digit-hex numbers, 
  //     separated by ~ characters. The format is two sequences for forward
  //     and reverse, starting with the minimum PWM value and followed by the
  //     speed ramp. There need to be exactly as many PWM entries as the
  //     cal command returned.
  if (!IsValidMode(MODE_INIT))
    return;

  ResetOutputMapping();
  ResetCalibrationData();
  
  byte cIndex = 3;
  byte mIndex = 0;
  // handle one motor channel after the other...
  while (CommandBuffer[cIndex] != 0)
  {
    if (mIndex>2)
    {
      SyntaxError2("mi", cIndex);
      return;
    }
    if (CommandBuffer[cIndex] != ':')
    {
      SyntaxError2("cn1", cIndex);
      return;
    }
    cIndex++;
    byte motorChannel = MotorChannelToIndex(CommandBuffer[cIndex]);
    if (motorChannel > 2)
    {
      SyntaxError2("mcn", cIndex);
      return;
    }
    cIndex++;
    if (CommandBuffer[cIndex] != ':')
    {
      SyntaxError2("cn2", cIndex);
      return;
    }
    cIndex++;
    // now there should be "min-A-B-C...-X" with min-[CALIBRATIONSTEPCOUNT*value] inputs for fwd/rev...
    byte m1,m2;
    m1 = motorChannel*2;
    m2 = motorChannel*2+1;
    IsDigital[m1] = IsDigital[m2] = false;

    if (!ParseMotorMapping(mIndex, cIndex))
    {
      return;
    }

    if (CommandBuffer[cIndex] != ':')
    {
      SyntaxError2("cn3", cIndex);
      return;
    }
    cIndex++;

    // next we have:
    // counter mapping, 6,7 as in the channel number...
    switch (CommandBuffer[cIndex])
    {
      case '6':
        if (!MapInputToMotor(mIndex, 0))
          return;
        break;
      case '7':
        if (!MapInputToMotor(mIndex, 1))
          return;
        break;
      case '-': // not mapped to any channel.
        break;
      default:
      {
        SyntaxError2("fbc", cIndex);
        return;
      }
    }
    MotorToChannelMap[mIndex] = motorChannel;

    cIndex++;
    mIndex++; // next motor...
  }
}

void HandleModeFree()
{
  // Switch to "free run" mode.
  
  // Syntax: free
  
  if (!IsValidMode(MODE_INIT))
    return;
  CurrentMode = MODE_FREE;
}

void HandleMotorCommand()
{
  // Direct drive, or motor on/off command.
  
  // syntax: drv:#:D[:%%]
  // or:     drv:#:S

  // # => channel number, as registered with the "mot" command.
  // D => direction, either F or R for forward/reverse.
  // S => stop, doesn't allow %%.
  // %% => two digit hex integer for speed percentage, 1-100 decimal.

  if (!IsValidMode(MODE_FREE))
    return;
  if (CommandBuffer[3] != ':' || CommandBuffer[5] != ':')
  {
    SyntaxError("cn");
    return;
  }
  byte mIndex, dir;
  int pct = 100;
  if (CommandBuffer[7] == ':')
  {
    pct = ParseCommandByte(8);
    if (pct > 100 || pct < 0)
    {
      SyntaxError2("pct", pct);
      return;
    }
  }
  switch(CommandBuffer[4])
  {
    case '0':
    case '1':
    case '2':
      mIndex = ((byte)CommandBuffer[4]) - (byte)'0';
      break;
    default:
      SyntaxError("mcn");
      return;
  }
  if (MotorToChannelMap[mIndex] == UNMAPPED)
  {
    Serial.println("err-chn");
    return;
  }
  switch(CommandBuffer[6])
  {
    case 'f':
    case 'F':
      dir = FWD;
      break;
    case 'r':
    case 'R':
      dir = REV;
      break;
    case 's':
    case 'S':
      dir = STP;
      break;
    default:
      SyntaxError("mmod");
      return;
  }
  if(MotorToInputMap[mIndex] != UNMAPPED) // reset the trap if it is set...
    MotorTrap[MotorToInputMap[mIndex]] = 0;
  RunMotor(mIndex, dir, (byte)pct);
}

void HandleMotorStatus()
{
  // Query the status of a motor channel.
  
  // syntax: mstat:#

  // The # is the motor channel number, as registered with the "mot" command.
  // The returned status info is:
  // mot:#:D:SS:COUNTERINFO
  //   # => motor number, should match the requested number.
  //   D => Direction: F, R or S for forward, reverse or stop. If unknown, ? is returned.
  //   SS => Speed: Hex code for the running motor speed, 1-100 decimal.
  // COUTNERINFO depends on wether there is a servo operation running.
  // If so, the format is:
  //   *:TICK:COUNT:CHANNEL
  //   * is fixed, and indicates a running operation.
  //   TICK is the target tick count
  //   COUNT is the current tick count
  //   CHANNEL is the raw number of the input channel that is monitored
  // If there is no servo operation, the format is:
  //  COUNT:CHANNEL
  //  with these two the same info as above.

  if (!IsValidMode(MODE_FREE))
    return;
  if (CommandBuffer[5] !=':' || CommandBuffer[7] != 0)
  {
    Serial.println("err-syn");
    return;
  }
  byte num;
  switch(CommandBuffer[6])
  {
    case '0':
    case '1':
    case '2':
      num = ((byte)CommandBuffer[6]) - (byte)'0';
      break;
    default:
      Serial.println("err-syn");
      return;
  }

  if (MotorToChannelMap[num] == UNMAPPED)
  {
    Serial.println("err-chn");
    return;
  }

  Serial.print("mot:");
  Serial.print(num);
  Serial.print(":");
  switch(MotorDirection[num])
  {
    case STP:
      Serial.print('S');
      break;
    case FWD:
      Serial.print('F');
      break;
    case REV:
      Serial.print('R');
      break;
    default:
      Serial.print('?');
      break;
  }
  Serial.print(':');
  Serial.print(MotorSpeed[num],HEX);
  if(MotorToInputMap[num] != UNMAPPED)
  {
    if (MotorTrap[MotorToInputMap[num]] > 0)
    {
      Serial.print(":*:");
      Serial.print(MotorTrap[MotorToInputMap[num]]);
      Serial.print(":");
      Serial.print(counters[MotorToInputMap[num]]);
      Serial.print(":");
      Serial.print(MotorToInputMap[num]);
    }
    else
    {
      Serial.print(':');
      Serial.print(counters[MotorToInputMap[num]]);  // don't reset counter if there's a trap running!
      Serial.print(":");
      Serial.print(MotorToInputMap[num]);
    }
  }
  Serial.println();
}

void HandleQueryCount()
{
  // "Query Counter" command. Reads back (and resets!) the requested raw counters.
  
  // syntax: qcnt:#
  // or:     qcnt

  // If there is a "trap" (i.e. a servo operation) pending for the requested counter,
  // an error "err-trap" is returned.
  // The # can be the channel number (6,7) to query, returning "cnt:###"
  // If the number is omitted, both channels are returned, "cnt:###:###"

  if (!IsValidMode(MODE_FREE))
    return;

  if (CommandBuffer[4]== 0)
  {
    if (MotorTrap[0] != 0 || MotorTrap[1] != 0)
    {
      Serial.println("err-trap");
      return;
    }
    Serial.print("cnt:");
    Serial.print(ReadCounter(0));
    Serial.print(":");
    Serial.println(ReadCounter(1));
  }
  else
  {
    if (CommandBuffer[4] !=':' || CommandBuffer[6] != 0)
    {
      Serial.println("err-syn");
      return;
    }
    byte num;
    switch(CommandBuffer[5])
    {
      case '6':
      case '7':
        num = ((byte)CommandBuffer[5]) - (byte)'0';
        break;
      default:
        Serial.println("err-syn");
        return;
    }
    if (MotorTrap[num])
    {
      Serial.println("err-trap");
      return;
    }
    Serial.print("cnt:");
    Serial.println(ReadCounter(num));
  }
}

void SetDigitalChannelPWM(byte n, char option)
{
  if (!IsDigital[n])
  {
    Serial.print('M');
    return;
  }
  byte value = 0;
  switch (option)
  {
    case '*':
      Serial.print('*');
      return;
    case '+':
    case '4':
      value = 255;
      break;
    case '-':
    case '0':
      value = 0;
      break;
    case '2':
      value = 128;
      break;
    case '1':
      value = 64;
      break;
    case '3':
      value = 192;
      break;
    default:
      Serial.print('E');
      return;
  }
  analogWrite(outputs[n], value);
  Serial.print(option);
}

void SetDigitalChannelStrict(byte n, char option)
{
  if (!IsDigital[n])
  {
    Serial.print('M');
    return;
  }
  bool value = false;
  switch (option)
  {
    case '*':
      Serial.print('*');
      return;
    case '+':
    case '1':
    case '2':
    case '3':
    case '4':
      value = true;
      option = '+';
      break;
    case '-':
    case '0':
      value = false;
      option = '-';
      break;
    default:
      Serial.print('E');
      return;
  }
  digitalWrite(outputs[n], value);
  Serial.print(option);
}

void SetDigitalChannel(byte n, char option)
{
  if (n == 0 || n == 2 || n == 4)
    SetDigitalChannelPWM(n, option);
  else
    SetDigitalChannelStrict(n, option);
}

void HandleSetDigitalPorts()
{
  // Sets digital output to requested status;
  // Any channel that is assigned a motor via the "mot" command is
  // ignored in this command.
  
  // syntax: set:######
  
  // the six-# marks are mapped to channel 0-5 in order.
  // # can be one of the following:
  //  + : turn on
  //  - : turn off
  //  * : leave as is (ignore)
  //  0-4: set "brightness" in channels 0, 2, 4 to 0, 25, 50, 75, 100% 
  //       (4 is equal to +, 0 equal to -) 
  //       For the other channels (1,3,5): 0 is off, anything else is on.

  if (!IsValidMode(MODE_FREE))
    return;

  if (CurrentCommandLength != 10 || CommandBuffer[3] != ':')
  {
    Serial.println("err-syn");
    return;
  }

  Serial.print("set:");
  for(byte i = 0; i< 6; i++)
  {
    SetDigitalChannel(i, CommandBuffer[i+4]);
  }
  Serial.println();
}

void HandleInit()
{
  // The "init" command will reset stop any output port 
  // and put it into the init mode without resetting
  // any channel assignments or calibration data.
  if (!IsValidMode(MODE_FREE | MODE_AUTO))
    return;
  FullStop();
  CurrentMode = MODE_INIT;
}

void HandleServoCommand()
{
  // The servo command will start a motor channel into the desired
  // direction and speed, setting a trap on the associated counter
  // channel to stop the motor.
  
  // syntax: srv:#:D:TICK[:%%]
  
  // # => motor channel number, 0,1,2 as assigned with the "mot" command.
  // D => direction: F or R, forward or reverse. STOP is not supported, 
  //                 use DRV command for that.
  // TICK => four digit hex integer for number of ticks to count.
  // %% => two digit hex integer for speed percentage, 1 to 100 decimal.
  
  if (!IsValidMode(MODE_FREE))
    return;

  
  if (CommandBuffer[3] != ':' || CommandBuffer[5] != ':' || CommandBuffer[7] != ':')
  {
    Serial.println("err-syn");
    return;
  }
  int limit = 0;
  byte temp = ParseCommandByte(8);
  limit = temp;
  limit <<=8;
  temp = ParseCommandByte(10);
  limit |= temp;
  if (limit < 0 || limit > 1000)
  {
    Serial.println("err-ovrflw");
    return;
  }
  byte mIndex, dir;
  int pct = 100;
  if (CommandBuffer[12] == ':')
  {
    pct = ParseCommandByte(13);
    if (pct > 100 || pct < 0)
    {
      Serial.println("err-syn");
      return;
    }
  }
  switch(CommandBuffer[4])
  {
    case '0':
    case '1':
    case '2':
      mIndex = ((byte)CommandBuffer[4]) - (byte)'0';
      break;
    default:
      Serial.println("err-syn");
      return;
  }
  if (MotorToChannelMap[mIndex] == UNMAPPED)
  {
    Serial.println("err-chn");
    return;
  }
  if (MotorToInputMap[mIndex] == UNMAPPED)
  {
    Serial.println("err-chn");
    return;
  }
  switch(CommandBuffer[6])
  {
    case 'f':
    case 'F':
      dir = FWD;
      break;
    case 'r':
    case 'R':
      dir = REV;
      break;
    // servo doesn't support "stopping" into position...
    default:
      Serial.println("err-syn");
      return;
  }
  TrapMotor(mIndex, limit);
  RunMotor(mIndex, dir, pct);
}

char* CommandNames[] = {  "ping", "cal", "reset", "init", "time", 
                          "cpol", "mot", "free", "drv", "qcnt", 
                          "mstat", "set", "srv" };
void (*CommandHandlers[])() = {  HandlePing, HandleCal, HandleReset, HandleInit, HandleTime, 
                              HandleCheckPolarity, HandleMotorSetup, HandleModeFree, HandleMotorCommand, HandleQueryCount, 
                              HandleMotorStatus, HandleSetDigitalPorts, HandleServoCommand };
#define COMMANDCOUNT ((byte)(sizeof(CommandNames) / sizeof(CommandNames[0])))

// Parse the command string and call the apropriate handler function
// which will take care of the details
void HandleCommand()
{
  int cmd = -1;
  for (int i = 0; i < COMMANDCOUNT; i++)
  {
    if (strncmp(CommandBuffer, CommandNames[i], strlen(CommandNames[i]))==0)
    {
      cmd = i;
      break;
    }
  }
  if (cmd >= 0 && CommandHandlers[cmd] != NULL)
    CommandHandlers[cmd]();
  else
    Serial.println("err-unkcmd");
}

void loop() {
  // check for input and append to buffer.
  HandleSerialInput();
  if (CommandComplete)
  {
    // in case of completed command line, interpret and run:
    HandleCommand();
  }

  // Some test methods to run hard coded sample patterns.
  // use these instead of the command loop above for
  // bare metal testing.
  //TestRunMotor();  
  //SpeedComparison();
  //MotorPWMCalibration(0);
}
