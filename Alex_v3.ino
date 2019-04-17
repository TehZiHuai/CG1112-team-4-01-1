#include <serialize.h>
#include "packet.h"
#include "constants.h"
#define COLOUR_CHECK true
#define IR_CHECK true

typedef enum
{
  STOP=0,
  FORWARD=1,
  BACKWARD=2,
  LEFT=3,
  RIGHT=4
} TDirection;
volatile TDirection dir = STOP;

/*
 * Alex's configuration constants
 */

// Number of ticks per revolution from the 
// wheel encoder.

#define COUNTS_PER_REV      192

// Wheel circumference in cm.
// We will use this to calculate forward/backward distance traveled 
// by taking revs * WHEEL_CIRC

#define WHEEL_CIRC          21 //MAYBE

// Motor control pins. You need to adjust these till
// Alex moves in the correct direction
#define LF                  6   // Left forward pin
#define LR                  5   // Left reverse pin
#define RF                  10  // Right forward pin
#define RR                  11  // Right reverse pin

// PI, for calculating turn circumference
#define PI                  3.141592654

// Alex's length and breath in cm
#define ALEX_LENGTH         18
#define ALEX_BREADTH        13

// TCS230 or TCS3200 pins wiring to Arduino
//#define S0 0
//#define S1 1
//#define S3 7

#define S2 4
#define sensorOut 8

// IR
#define FRONT 7
#define BACK 9
#define LEFT 12
#define RIGHT 13

/*
 *    Alex's State Variables
 */

// Alex's diagonal. We compute and store this once
// since it is expensive to compute and really doesn't change.
float AlexDiagonal = 0.0;

// Alex's turnign circumference, calculated once
float AlexCirc = 0.0; 

// Store the ticks from Alex's left and
// right encoders for moving forwards and backwards
volatile unsigned long leftForwardTicks; 
volatile unsigned long rightForwardTicks;
volatile unsigned long leftReverseTicks; 
volatile unsigned long rightReverseTicks;

//Left and right encoder ticks for turning
volatile unsigned long leftForwardTicksTurns; 
volatile unsigned long rightForwardTicksTurns;
volatile unsigned long leftReverseTicksTurns; 
volatile unsigned long rightReverseTicksTurns;


// Forward and backward distance traveled
volatile unsigned long forwardDist;
volatile unsigned long reverseDist;

// Variable to keep track of whether we have moved a commanded distance
unsigned long deltaDist;
unsigned long newDist;

// Variables to keep track of our turing angle
unsigned long deltaTicks;
unsigned long targetTicks;

// Stores frequency read by the photodiodes
int redFrequency = 0;
int greenFrequency = 0;
int count = 0;
int red_ave = 100;
int green_ave = 0;
int color;
int red;
int green;


// Store values for IR sensor
int IR;
int front_out;
int back_out;
int left_out;
int right_out;

/*
 * 
 * Alex Communication Routines.
 * 
 */
 
TResult readPacket(TPacket *packet)
{
    // Reads in data from the serial port and
    // deserializes it.Returns deserialized
    // data in "packet".
    
    char buffer[PACKET_SIZE];
    int len;

    len = readSerial(buffer);

    if(len == 0)
      return PACKET_INCOMPLETE;
    else
      return deserialize(buffer, len, packet);
    
}


void sendStatus()
{
  // Implement code to send back a packet containing key
  // information like leftTicks, rightTicks, leftRevs, rightRevs
  // forwardDist and reverseDist
  // Use the params array to store this information, and set the
  // packetType and command files accordingly, then use sendResponse
  // to send out the packet. See sendMessage on how to use sendResponse.
  //
  TPacket statusPacket;
  statusPacket.packetType = PACKET_TYPE_RESPONSE;
  statusPacket.command = RESP_STATUS;
  statusPacket.params[0] = leftForwardTicks;
  statusPacket.params[1] = rightForwardTicks;
  statusPacket.params[2] = leftReverseTicks;
  statusPacket.params[3] = rightReverseTicks;
  statusPacket.params[4] = leftForwardTicksTurns;
  statusPacket.params[5] = rightForwardTicksTurns;
  statusPacket.params[6] = leftReverseTicksTurns;
  statusPacket.params[7] = rightReverseTicksTurns;
  statusPacket.params[8] = forwardDist;
  statusPacket.params[9] = reverseDist;
  if (COLOUR_CHECK) {
    statusPacket.params[10] = color;
//    statusPacket.params[10] = red;
//    statusPacket.params[10] = green;
  }
  if (IR_CHECK) {
    statusPacket.params[11] = IR;
  }
  sendResponse(&statusPacket);
}

void sendMessage(const char *message)
{
  // Sends text messages back to the Pi. Useful
  // for debugging.
  
  TPacket messagePacket;
  messagePacket.packetType=PACKET_TYPE_MESSAGE;
  strncpy(messagePacket.data, message, MAX_STR_LEN);
  sendResponse(&messagePacket);
}

void sendBadPacket()
{
  // Tell the Pi that it sent us a packet with a bad
  // magic number.
  
  TPacket badPacket;
  badPacket.packetType = PACKET_TYPE_ERROR;
  badPacket.command = RESP_BAD_PACKET;
  sendResponse(&badPacket);
  
}

void sendBadChecksum()
{
  // Tell the Pi that it sent us a packet with a bad
  // checksum.
  
  TPacket badChecksum;
  badChecksum.packetType = PACKET_TYPE_ERROR;
  badChecksum.command = RESP_BAD_CHECKSUM;
  sendResponse(&badChecksum);  
}

void sendBadCommand()
{
  // Tell the Pi that we don't understand its
  // command sent to us.
  
  TPacket badCommand;
  badCommand.packetType=PACKET_TYPE_ERROR;
  badCommand.command=RESP_BAD_COMMAND;
  sendResponse(&badCommand);

}

void sendBadResponse()
{
  TPacket badResponse;
  badResponse.packetType = PACKET_TYPE_ERROR;
  badResponse.command = RESP_BAD_RESPONSE;
  sendResponse(&badResponse);
}

void sendOK()
{
  TPacket okPacket;
  okPacket.packetType = PACKET_TYPE_RESPONSE;
  okPacket.command = RESP_OK;
  sendResponse(&okPacket);  
}

void sendResponse(TPacket *packet)
{
  // Takes a packet, serializes it then sends it out
  // over the serial port.
  char buffer[PACKET_SIZE];
  int len;

  len = serialize(buffer, packet, sizeof(TPacket));
  writeSerial(buffer, len);
}


/*
 * Setup and start codes for external interrupts and 
 * pullup resistors.
 * 
 */
// Enable pull up resistors on pins 2 and 3
void enablePullups()
{
  // Use bare-metal to enable the pull-up resistors on pins
  // 2 and 3. These are pins PD2 and PD3 respectively.
  // We set bits 2 and 3 in DDRD to 0 to make them inputs. 

  DDRD  &= 0b11110011;
  PORTD |= 0b00001100;
}

// Functions to be called by INT0 and INT1 ISRs.
void leftISR()
{
  if (dir == FORWARD){
    leftForwardTicks++;
    forwardDist = (unsigned long) ((float) leftForwardTicks / COUNTS_PER_REV * WHEEL_CIRC);
  }
  else if (dir == BACKWARD){
    leftReverseTicks++;
    reverseDist = (unsigned long) ((float) leftReverseTicks / COUNTS_PER_REV * WHEEL_CIRC);
  }
  else if (dir == LEFT){
    leftReverseTicksTurns++;
  }
  else if (dir == RIGHT){
    leftForwardTicksTurns++;
  }


  
}

void rightISR()
{
  if (dir == FORWARD){
    rightForwardTicks++;
  }
  else if (dir == BACKWARD){
    rightReverseTicks++;
  }
  else if (dir == LEFT){
    rightForwardTicksTurns++;
  }
  else if (dir == RIGHT){
    rightReverseTicksTurns++;
  }

}

// Set up the external interrupt pins INT0 and INT1
// for falling edge triggered. Use bare-metal.
void setupEINT()
{
  // Use bare-metal to configure pins 2 and 3 to be
  // falling edge triggered. Remember to enable
  // the INT0 and INT1 interrupts.

  EICRA = 0b00001010;
  EIMSK = 0b00000011;
}

// Implement the external interrupt ISRs below.
// INT0 ISR should call leftISR while INT1 ISR
// should call rightISR.

ISR(INT0_vect)
{
  leftISR();
}

ISR(INT1_vect)
{
  rightISR();
}


// Implement INT0 and INT1 ISRs above.

/*
 * Setup and start codes for serial communications
 * 
 */
// Set up the serial connection. For now we are using 
// Arduino Wiring, you will replace this later
// with bare-metal code.
void setupSerial()
{
  // To replace later with bare-metal.
  Serial.begin(57600);
}

// Start the serial connection. For now we are using
// Arduino wiring and this function is empty. We will
// replace this later with bare-metal code.
void startSerial()
{
  // Empty for now. To be replaced with bare-metal code
  // later on.
  
}

// Read the serial port. Returns the read character in
// ch if available. Also returns TRUE if ch is valid. 
// This will be replaced later with bare-metal code.

int readSerial(char *buffer)
{

  int count=0;

  while(Serial.available())
    buffer[count++] = Serial.read();

  return count;
}

// Write to the serial port. Replaced later with
// bare-metal code

void writeSerial(const char *buffer, int len)
{
  Serial.write(buffer, len);
}

/*
 * Alex's motor drivers.
 * 
 */

// Set up Alex's motors. Right now this is empty, but
// later you will replace it with code to set up the PWMs
// to drive the motors.
void setupMotors()
{
  /* Our motor set up is:  
   *    A1IN - Pin 5, PD5, OC0B
   *    A2IN - Pin 6, PD6, OC0A
   *    B1IN - Pin 10, PB2, OC1B
   *    B2In - pIN 11, PB3, OC2A
   */
}

// Start the PWM for Alex's motors.
// We will implement this later. For now it is
// blank.
void startMotors()
{
  
}

// Convert percentages to PWM values
int pwmVal(float speed)
{
  if(speed < 0.0)
    speed = 0;

  if(speed > 100.0)
    speed = 100.0;

  return (int) ((speed / 100.0) * 255.0);
}

// Move Alex forward "dist" cm at speed "speed".
// "speed" is expressed as a percentage. E.g. 50 is
// move forward at half speed.
// Specifying a distance of 0 means Alex will
// continue moving forward indefinitely.
//void forward(float dist, float speed)
void forward(float dist)
{
  if (dist > 0)
    deltaDist  = dist*0.92;
  else
    deltaDist = 5;
  newDist = forwardDist + deltaDist;

  
  dir = FORWARD;
  
  //int val = pwmVal(speed);

  
  // For now we will ignore dist and move
  // forward indefinitely. We will fix this
  // in Week 9.


  // LF = Left forward pin, LR = Left reverse pin
  // RF = Right forward pin, RR = Right reverse pin
  // This will be replaced later with bare-metal code.
  
  analogWrite(LF, 204);
  analogWrite(RF, 160);
//  analogWrite(LF, val);
//  analogWrite(RF, 0.7*val);
  analogWrite(LR,0);
  analogWrite(RR, 0);
}

// Reverse Alex "dist" cm at speed "speed".
// "speed" is expressed as a percentage. E.g. 50 is
// reverse at half speed.
// Specifying a distance of 0 means Alex will
// continue reversing indefinitely.
//void reverse(float dist, float speed)
void reverse(float dist)
{
  if(dist>0)
    deltaDist = dist*0.92;
  else
    deltaDist = 5;

  newDist = reverseDist + deltaDist;
  
  dir = BACKWARD;
  
  //int val = pwmVal(speed);

  // For now we will ignore dist and 
  // reverse indefinitely. We will fix this
  // in Week 9.

  // LF = Left forward pin, LR = Left reverse pin
  // RF = Right forward pin, RR = Right reverse pin
  // This will be replaced later with bare-metal code.
  analogWrite(LR, 204);
  analogWrite(RR,160);
//  analogWrite(LR, val);
//  analogWrite(RR, 0.7*val);
  analogWrite(LF, 0);
  analogWrite(RF, 0);
}

// New function to estimate number of wheel ticks
// needed to turn an angle
unsigned long computeDeltaTicks(float ang)
{
  // We will assume that angular distance moved = linear distance moved in one wheel
  // revolution. This is (probably) incorrect but simplifies calculation.
  // # of wheel revs to make one full 360 turn is AlexCirc / WHEEL_CIRC
  // This is for 360 degrees. For ang degrees it will be (ang * AlexCirc) / (360 * WHEEL_CIRC)
  // To convert to ticks, we multiply by COUNTS_PER_REV.

  unsigned long ticks = (unsigned long) ((ang * AlexCirc * COUNTS_PER_REV) / (360.0 * WHEEL_CIRC));

  return ticks;
}


// Turn Alex left "ang" degrees at speed "speed".
// "speed" is expressed as a percentage. E.g. 50 is
// turn left at half speed.
// Specifying an angle of 0 degrees will cause Alex to
// turn left indefinitely.
void left(float ang)//, float speed)
{
  if (ang > 0)
    deltaTicks = computeDeltaTicks(ang*10)* 0.35;
  else
    deltaTicks = 10;

  dir = LEFT;
//  int val = pwmVal(speed);
//
//  if(val<255*0.8){
//    val = 255*0.8;
//  }
  
  targetTicks = leftReverseTicksTurns + deltaTicks;
// For now we will ignore ang. We will fix this in Week 9.
  // We will also replace this code with bare-metal later.
  // To turn left we reverse the left wheel and move
  // the right wheel forward.
//  analogWrite(LR, 220);
//  analogWrite(RF, 220);
  analogWrite(LR, 255*0.8);
  analogWrite(RF, 255*0.8*0.7);
  analogWrite(LF, 0);
  analogWrite(RR, 0);
}

// Turn Alex right "ang" degrees at speed "speed".
// "speed" is expressed as a percentage. E.g. 50 is
// turn left at half speed.
// Specifying an angle of 0 degrees will cause Alex to
// turn right indefinitely.
void right(float ang)//, float speed)
{

  if (ang > 0)
    deltaTicks = computeDeltaTicks(ang*10) * 0.25;
  else
    deltaTicks = 10;
  
  dir = RIGHT;
//  int val = pwmVal(speed);
//
//  if (val < 255*0.8) {
//    val = 255*0.8;
//  }

  targetTicks = rightReverseTicksTurns + deltaTicks;


  // For now we will ignore ang. We will fix this in Week 9.
  // We will also replace this code with bare-metal later.
  // To turn right we reverse the right wheel and move
  // the left wheel forward.
  analogWrite(RR, 255*0.8*0.7);
  analogWrite(LF, 255*0.8);
//  analogWrite(RR, 220);
//  analogWrite(LF, 220);
  analogWrite(LR, 0);
  analogWrite(RF, 0);
}

// Stop Alex. To replace with bare-metal code later.
void stop()
{
  dir = STOP;
  
  analogWrite(LF, 0);
  analogWrite(LR, 0);
  analogWrite(RF, 0);
  analogWrite(RR, 0);
}

/*
 * Alex's setup and run codes
 * 
 */

// Clears all our counters
void clearCounters()
{
  leftForwardTicks=0;
  rightForwardTicks=0;
  leftReverseTicks=0;
  rightReverseTicks=0;
  leftForwardTicksTurns=0;
  rightForwardTicksTurns=0;
  leftReverseTicksTurns=0;
  rightReverseTicksTurns=0;
  forwardDist=0;
  reverseDist=0; 
}

// Clears one particular counter
void clearOneCounter(int which)
{
  clearCounters();
}
// Intialize Vincet's internal states

void initializeState()
{
  clearCounters();
}

void handleCommand(TPacket *command)
{
  switch(command->command)
  {
    // For movement commands, param[0] = distance, param[1] = speed.
    case COMMAND_FORWARD:
        sendOK();
//        forward((float) command->params[0], (float) command->params[1]);
//      forward(5.0);
        forward((float) command->params[0]);
      break;
    case COMMAND_REVERSE:
        sendOK();
//        reverse(5.0);//(float) command->params[0], (float) command->params[1]);
        reverse((float) command->params[0]);
      break;
    case COMMAND_TURN_LEFT:
        sendOK();
//        left();//(float) command->params[0], (float) command->params[1]);
        left((float) command->params[0]);
      break;
    case COMMAND_TURN_RIGHT:
        sendOK();
//        right();//(float) command->params[0], (float) command->params[1]);
        right((float) command->params[0]);
      break;
    case COMMAND_STOP:
        sendOK();
        stop();
      break;
    case COMMAND_GET_STATS:
        sendOK();
        sendStatus();
      break;
    

    /*
     * Implement code for other commands here.
     * 
     */
        
    default:
      sendBadCommand();
  }
}

void waitForHello()
{
  int exit=0;

  while(!exit)
  {
    TPacket hello;
    TResult result;
    
    do
    {
      result = readPacket(&hello);
    } while (result == PACKET_INCOMPLETE);

    if(result == PACKET_OK)
    {
      if(hello.packetType == PACKET_TYPE_HELLO)
      {
     

        sendOK();
        exit=1;
      }
      else
        sendBadResponse();
    }
    else
      if(result == PACKET_BAD)
      {
        sendBadPacket();
      }
      else
        if(result == PACKET_CHECKSUM_BAD)
          sendBadChecksum();
  } // !exit
}

void setup() {
  
  // Compute the diagonal
  AlexDiagonal = sqrt((ALEX_LENGTH * ALEX_LENGTH) + (ALEX_BREADTH * ALEX_BREADTH));
  AlexCirc = PI * AlexDiagonal;

  // Setting the outputs for TCS3200 color sensor
  //pinMode(S0, OUTPUT);
  //pinMode(S1, OUTPUT);
  pinMode(S2, OUTPUT);
  //pinMode(S3, OUTPUT);
  pinMode(sensorOut, INPUT);

  // Setting for IR sensor
  pinMode(FRONT, INPUT);
  pinMode(LEFT, INPUT);
  pinMode(RIGHT, INPUT);
  pinMode(BACK, INPUT);
  
  // Setting frequency scaling to 20%
  //digitalWrite(S0,HIGH);
  //digitalWrite(S1,LOW);
  
   // Begins serial communication 
  Serial.begin(9600);

  cli();
  setupEINT();
  setupSerial();
  startSerial();
  setupMotors();
  startMotors();
  enablePullups();
  initializeState();
  sei();
}

void handlePacket(TPacket *packet)
{
  switch(packet->packetType)
  {
    case PACKET_TYPE_COMMAND:
      handleCommand(packet);
      break;
case PACKET_TYPE_RESPONSE:
      break;

    case PACKET_TYPE_ERROR:
      break;

    case PACKET_TYPE_MESSAGE:
      break;

    case PACKET_TYPE_HELLO:
      break;
  }
}


void loop() {

  TPacket recvPacket; // This holds commands from he Pi

  TResult result = readPacket(&recvPacket);
  
  if(result == PACKET_OK)
    handlePacket(&recvPacket);
  else if(result == PACKET_BAD)
  {
    sendBadPacket();
  }
  else if(result == PACKET_CHECKSUM_BAD)
  {
    sendBadChecksum();
  } 
  

  if(deltaDist > 0)
  {
    if (dir == FORWARD)
    {
      if (forwardDist > newDist)
      {
        deltaDist = 0;
        newDist = 0;
        stop();
      }
    }
    else if(dir==BACKWARD)
    {
      if(reverseDist > newDist)
      {
        deltaDist = 0;
        newDist = 0;
        stop();
      }
    }

    else if (dir == STOP)
    {
      deltaTicks = 0;
      targetTicks = 0;
      stop();
    }
  }

  if (deltaTicks > 0)
  {
    if (dir == LEFT)
    {
      if (leftReverseTicksTurns >= targetTicks)
      {
        deltaTicks = 0;
        targetTicks = 0;
        stop();
      }
    }
    else if (dir == RIGHT)
      {
      if (rightReverseTicksTurns >= targetTicks)
      {
        deltaTicks = 0;
        targetTicks = 0;
        stop();
      }
    }
    else if (dir == STOP)
    {
      deltaTicks = 0;
      targetTicks = 0;
      stop();
    }
  }

 if (COLOUR_CHECK) { 
  // Setting RED (R) filtered photodiodes to be read
  digitalWrite(S2,LOW);
  //digitalWrite(S3,LOW);
  
  // Reading the output frequency
  redFrequency = pulseIn(sensorOut, LOW);
  
  // Setting GREEN (G) filtered photodiodes to be read
  digitalWrite(S2,HIGH);
  //digitalWrite(S3,HIGH);
  
  // Reading the output frequency
  greenFrequency = pulseIn(sensorOut, LOW);

  count += 1;
  
  if (count == 4) {
    red_ave /= 5; 
    green_ave /= 5;

    red = red_ave;
    green = green_ave;

//  if (red_ave < green_ave) {
  if (red_ave > 100) {
    color = 0;
  }
  else if ((red_ave <= 100) & (green_ave <akk 100)){
    color = 1;
  }
  else color = 9;
    
    count = 0; // reset
    red_ave = 0;
    green_ave = 0;
    
  } else {
    red_ave += redFrequency;
    green_ave += greenFrequency;
  }
 }

if (IR_CHECK){
//reading IR sensors
  front_out = digitalRead(FRONT);
  back_out = digitalRead(BACK);
  left_out = digitalRead(LEFT);
  right_out = digitalRead(RIGHT);
  
  if(front_out == 0){
    stop();
    delay(50);
    reverse(2);
    delay(50);
    IR = 0;
    //sendStatus();
  }  
  else if(back_out == 0){
    stop();
    delay(50);
    forward(2);
    delay(50);
    IR = 1;
    //sendStatus();
  }

}
}
