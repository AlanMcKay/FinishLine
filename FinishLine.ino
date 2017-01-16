
/*
 * 2017.01.15 - alan.mckay@gmail.com
 * 
 * Program to run a Cub Car / Scout Truck track finish line
 * Hardware required :
 *  - Arduino Uno or Sparkfun Redboard
 *  - 16x2 LCD connected in parallel connected as per SIK circuit 15
 *  - potentiometer also as per SIK circuit 15
 *  - simple button from Sparkfun Inventor's Kit (SIK circuit 6)
 *  - 3 x SF QRD1114 Optical Detector https://www.sparkfun.com/products/246 
 *    - connected as per details in that link
 * Full circuit diagram to be provided at a later date. 
 * 
 */

#include <stdio.h>
#include <LiquidCrystal.h>
LiquidCrystal lcd(12,11,5,4,3,2);

// the button is moved from SIK circuit 6 to pin 10
const int BUTTONPIN  = 10;  // pushbutton pin to reset
const int STARTPIN    = 7;  // the door sensor to start the race
const int ledPin      = 9;  // LED pin
const int LANE1       = A0;
const int LANE2       = A1;
const int LANE3       = A2;
const int NUMLANES    = 3;    // Not used - maybe next release
const int NUMSAMPLES  = 3000; // used for calibrating the light sensors

unsigned long int 
  milliStart  = 0;    // timestamp of when the race starts

char          myOutput[2][32];

// Important Note : Arduino uses 0-based arrays like C, but
// I am being lazy and not accounting for that - I simply use
// arrays of 4 and do not use the 0 index.  That means this 
// really would not be suitable for a 4 lane track

// An earlier version of the program had a reason for tracking how
// many resets there were.  This version really has no reason.
unsigned int  
  myResets = 0,
  // myResults stores lane numbers.
  // [0] is the winner
  // [1] is 2nd place
  // [2] is 3rd place
  // yes, I use the 0 index on this one
  myResults[4], 
  // when myLanesDone == NUMLANES all cars are in
  myLanesDone = 0;

unsigned long int 
  // stores the calibrated baseline for the lane sensors
  myCalibrated[4],
  // as each car finishes, store its time in this guy
  myLaneTime[4];
boolean
  myRaceOn    = 0,
  myRaceOver  = 0,
  // as a lane finishes set this to true
  myLaneDone[4];

// We need a few global variables specifically for 
// rotating the display when the race is over

const int 
  FINISHFANCY   = 1,      // Set to 0 if you don't want rotating results after finish
  FINISHSTATES  = 4,      // how many states are there at the finish?
  FINISHROTDIS  = 3000;   // rotate display this many milliseconds

boolean
  myFinishRotate  = 0;
  
int
  myFinishState = 1;      // keep track of which state we are in

unsigned long int
  myFinishRotTime = 0;    // last time we rotated the finish message
  
void milli2human( char * myOutput, unsigned long int mSec )
/*
 * converts a number of milliseconds to a human readable string
 * H:MM:SS.MMM
 * Hours:Minutes:Seconds.Milliseconds
 * 
 * Input :  myOutput - buffer to put it into
 *          milliSec - number of milliseconds
 */
{
  int myHour  = 0,
      myMin   = 0,
      mySec   = 0,
      myMilli = 0;

  // Just keep using modulo to strip off the remainder
  
  myMilli = mSec % 1000;
  mSec    = mSec / 1000;
  mySec   = mSec % 60;
  mSec    = mSec / 60;
  myMin   = mSec % 60;
  mSec    = mSec / 60;
  myHour  = mSec;
  
  sprintf( myOutput, "%d:%02d:%02d.%03d", myHour, myMin, mySec, myMilli );
}

void UpdateLED( unsigned int myOnOff )
/*
 * Just turn the LED on or off according to the state
 * Sloppy use of a function, yeah
 */
{
  switch( myOnOff ) {
      case 0:
        digitalWrite( ledPin, LOW );
        break;
      case 1:
        digitalWrite( ledPin, HIGH );
        break;
      default:
        digitalWrite( ledPin, LOW );
        break;
  } 
}

void UpdateDisplay( char myDisplay[2][32], boolean myClear = 0 )
/*
 * This will eventually be rewritten to account for different types of displays
 * At which point we'll set a constant for display type or some such thing
 * Right now it assumes a 16x2 LCD in parallel mode
 */
{
  Serial.println( myDisplay[0] );
  Serial.println( myDisplay[1] );
  if ( myClear )
    lcd.clear();
  lcd.setCursor(0,0);
  lcd.print( myDisplay[0] );
  lcd.setCursor(0,1);
  lcd.print( myDisplay[1] );
}

void CalibrateLightSensors()
// We just loop NUMSAMPLES times and take a sample for each iteration
// Compute the average of the samples and use that for our baseline
{
  unsigned long int
    myLoop = 0;
  
  // Set up the light sensors for the lanes
  pinMode(LANE1, INPUT);
  pinMode(LANE2, INPUT);
  pinMode(LANE3, INPUT);

  // establishing lighting level baselines  
  sprintf( myOutput[0], "Calibrating" );
  sprintf( myOutput[1], "Sensors" );
  UpdateDisplay( myOutput, 1 );

  // Be careful not to set NUMSAMPLE too high or
  // you could end up outside the range of 
  // unsigned long int (4 bytes IIRC)
  
  for ( myLoop=0; myLoop < NUMSAMPLES; myLoop++ )
  {
    myCalibrated[1] += analogRead(LANE1);
    myCalibrated[2] += analogRead(LANE2);
    myCalibrated[3] += analogRead(LANE3); 
  }
  myCalibrated[1] = myCalibrated[1] / myLoop;
  myCalibrated[2] = myCalibrated[2] / myLoop;
  myCalibrated[3] = myCalibrated[3] / myLoop;
}

void RaceResultsString( char * myString )
// returns the race results up to this point
// e.g. "L2" if only lane 2 has finished
// or "L2 L1" if lane 2 was in first, and lane 1 next
// or "L2 L1 L3" if all 3 cars are in
{
  char myTempStr[32];
  
  switch( myLanesDone ) 
  {
  case 1:
    sprintf( myTempStr, "L%d", myResults[0] );
    break;
  case 2:
    sprintf( myTempStr, "L%d L%d", myResults[0], myResults[1] );
    break;
  case 3:
    sprintf( myTempStr, "L%d L%d L%d", myResults[0], myResults[1], myResults[2] );
    break;
  }
  strcpy( myString, myTempStr );
}
  
int CrossFinishLine( int myLane )
// call me when a car reaches the finish line
// return 0 if this car already crossed the finish line
// or if the race has not yet started
// otherwise return 1
{
  char myTempStr[32];

  // check to make sure the race is running
  if ( myRaceOn == 0 )
    return 0;

  // return if this lane already crossed finish line
  
  if ( myLaneDone[ myLane ] )
    return 0;

  // record this lane's time
  myLaneTime[myLane]        = millis() - milliStart;

  // set this lane as done
  myLaneDone[myLane]        = 1;

  // record the result and increment done lanes
  myResults[myLanesDone++]  = myLane;
  
  // put this car's time in the output buffer
  
  milli2human( myOutput[0], myLaneTime[myLane] );
  RaceResultsString( myOutput[1] );
  UpdateDisplay( myOutput, 1 );

  return 1;
}

void StartRace()
{
  myRaceOn    = 1;
  myRaceOver  = 0;
  milliStart  = millis();
  strcpy( myOutput[1], "Go!" );
  UpdateDisplay( myOutput, 1 );
}

void FinishRace()
{
  char
    myTempStr[32];
    
  myRaceOn    = 0;
  myRaceOver  = 1;

  if (FINISHFANCY == 0)
  {
    // just display the results - no rotating
    
    strcpy( myOutput[0], "Race Results" );
    RaceResultsString( myOutput[1] );
    UpdateDisplay( myOutput, 1 );
    
  } else {
    
    // rotate the display  
  
    if ( myFinishRotate == 0 ) 
    {
      myFinishRotTime = millis();
      myFinishRotate  = 1;
    }

    if ( millis() > myFinishRotTime + FINISHROTDIS )
    {
      myFinishRotate  = 0;
    
      switch (myFinishState)
      {
          case 1:
            strcpy( myOutput[0], "Race Results" );
            RaceResultsString( myOutput[1] );
            UpdateDisplay( myOutput, 1 );
            break;
            
         case 2:
            strcpy( myOutput[0], "First Place" );
            milli2human( myTempStr, myLaneTime[ myResults[0] ] );
            sprintf( myOutput[1], "L%d %s", myResults[0], myTempStr );
            UpdateDisplay( myOutput, 1 );
            break;
            
          case 3:
            strcpy( myOutput[0], "Second Place" );
            milli2human( myTempStr, myLaneTime[ myResults[1] ] );
            sprintf( myOutput[1], "L%d %s", myResults[1], myTempStr );
            UpdateDisplay( myOutput, 1 );
            break;
            
          case 4:
            // I am not sure why but this one stays twice as long as the others
            strcpy( myOutput[0], "Third Place" );
            milli2human( myTempStr, myLaneTime[ myResults[2] ] );
            sprintf( myOutput[1], "L%d %s", myResults[2], myTempStr );
            UpdateDisplay( myOutput, 1 );
            break;
      }

      // change states for next time
      
      if ( myFinishState++ > FINISHSTATES )
        myFinishState = 1;

    }
      
  }
   
  UpdateDisplay( myOutput, 1 );
 
}

void OnYourMarks()
{
  strcpy( myOutput[0], "On your marks ..." );
  strcpy( myOutput[1], "Get set ..." );
  UpdateDisplay( myOutput, 1 );
}

void setup()
{
  int myLoop  = 0;

  myLanesDone     = 0;
  myRaceOn        = 0;
  myRaceOver      = 0;
  myFinishState   = 1;
  myFinishRotate  = 0;
  
  // Serial port setup for debug
  Serial.begin(9600);

  // Initialize the LCD
  lcd.begin(16, 2);
  lcd.clear();

  // Set up the pushbutton pins to be an input:
  pinMode(BUTTONPIN, INPUT);
  pinMode(STARTPIN, INPUT);
  
  // Set up the LED pin to be an output:
  pinMode(ledPin, OUTPUT);      

  // Initialize some vars
  for ( myLoop=0; myLoop < 4; myLoop++ )
  {
    myResults[ myLoop ]     = 0;
    myLaneDone[ myLoop ]    = 0;
    myCalibrated[ myLoop ]  = 0;
    myLaneTime[ myLoop ]    = 0;    
  }

  CalibrateLightSensors();

  // Flash the LED and say "Reset"
  // Kind of redundant but whatever ...
  
  UpdateLED(1);
  sprintf( myOutput[0], "Reset" );
  sprintf( myOutput[1], " " );
  UpdateDisplay( myOutput, 1 );
  delay(1000);
  UpdateLED(0);

  // clear the output buffers
  sprintf( myOutput[0], "                               " );
  sprintf( myOutput[1], "                               " );
  UpdateDisplay( myOutput, 1 ); 
  myResets++;

}

void loop()
{
  int myButton  = 0,
      myStart   = 0,
      oneLane   = 0,
      myLanes[4];

  // read all the input sensors
  myLanes[1]  = analogRead(LANE1);
  myLanes[2]  = analogRead(LANE2);
  myLanes[3]  = analogRead(LANE3);
  myButton    = digitalRead(BUTTONPIN);
  myStart     = digitalRead(STARTPIN);

  // First check for reset button
  
  if (myButton == LOW)
  {
    setup();
    UpdateLED(1);
    delay(100);
    UpdateLED(0);
  }

  // if the race is over the reset button is required
  if ( myRaceOver == 1 )
  {
    FinishRace();
  } else
  {
    // check to see if the race has started
    if ( myRaceOn == 0 )
    {
      OnYourMarks();
      if (myStart == LOW)
        StartRace();
    } else 
    {
      // if the light entering the lane is 2/3 of the calibrated baseline
      // then we assume it is a car crossing the finish line
  
      for ( oneLane=1; oneLane < 4; oneLane++ )
        if (myLanes[oneLane] < ( myCalibrated[oneLane] ) / 3 * 2 )
          if ( CrossFinishLine( oneLane ) )
            UpdateDisplay( myOutput, 1 ); 

      // update display line 0 with running time

      milli2human( myOutput[0], millis() - milliStart );
      UpdateDisplay( myOutput, 1 );

      // check to see if race is over
    
      if ( myLanesDone == NUMLANES )
        FinishRace();
    }
  }

}
