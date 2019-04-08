/*********
  Rui Santos
  Complete project details at http://randomnerdtutorials.com  
*********/

// base code taken from Rui Santos, modified to only detect red or green
// takes averages of colour data, prints out color detected

// **************************** IMPORTANT **********************************
//
// only tested up to 4cm, past this range green seems to be very unstable
// well lit condition when testing data, to simulate Alex's condition in lab
//
// **************************************************************************

// TCS230 or TCS3200 pins wiring to Arduino
#define S0 4
#define S1 5
#define S2 6
#define S3 7
#define sensorOut 8

// Stores frequency read by the photodiodes
int redFrequency = 0;
int greenFrequency = 0;
int blueFrequency = 0;
int count = 0;
int red_ave = 0;
int green_ave = 0;

void setup() {
  // Setting the outputs
  pinMode(S0, OUTPUT);
  pinMode(S1, OUTPUT);
  pinMode(S2, OUTPUT);
  pinMode(S3, OUTPUT);
  pinMode(9, OUTPUT);
  digitalWrite(9, HIGH);
  
  // Setting the sensorOut as an input
  pinMode(sensorOut, INPUT);
  
  // Setting frequency scaling to 20%
  digitalWrite(S0,HIGH);
  digitalWrite(S1,LOW);
  
   // Begins serial communication 
  Serial.begin(9600);
}
void loop() {
  // Setting RED (R) filtered photodiodes to be read
  digitalWrite(S2,LOW);
  digitalWrite(S3,LOW);
  
  // Reading the output frequency
  redFrequency = pulseIn(sensorOut, LOW);
  
  
  // Setting GREEN (G) filtered photodiodes to be read
  digitalWrite(S2,HIGH);
  digitalWrite(S3,HIGH);
  
  // Reading the output frequency
  greenFrequency = pulseIn(sensorOut, LOW);

  count += 1;
  
  if (count == 4) {
    red_ave /= 5; green_ave /= 5;
    if (red_ave < green_ave) Serial.print("red");
    else Serial.print("green");
    Serial.print("\n");
    count = 0; // reset
    red_ave = 0;
    green_ave = 0;
  } else {
    red_ave += redFrequency;
    green_ave += greenFrequency;
  }
  
}
