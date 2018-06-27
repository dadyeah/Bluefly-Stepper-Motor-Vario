



#include <SoftwareSerial.h>
//To drive the extra serial port for RS232 to Bluefly


#define rxPin 9
#define txPin 8
SoftwareSerial BlueflySerial = SoftwareSerial(rxPin, txPin);
//String buffer_test = "";

static const byte GPS_BUFFER_SIZE = 100;
                  // The maximum size allowed for an NMEA sentence.
char sentence[GPS_BUFFER_SIZE];
                  // The character array where the NMEA sentence
                  // is stored.
byte sentenceIndex;
                  // The position in the sentence where the next
                  // character will be appended.
boolean checking; // Determines if characters arriving from GPS
                  // are X-OR'd for comparison to check digits.
char checkHex1, checkHex2;
                  // The last two characters of the NMEA sentence.
int checksum, checkDigit;
                  // checksum: the running total of X-OR operations
                  // on all the characters arriving from the GPS
                  // checkDigit: which of the check digits is
                  // is arriving next

//-------------------------------------------------------------------------------------

// Arduino Wire library is required if I2Cdev I2CDEV_ARDUINO_WIRE implementation
// is used in I2Cdev.h
#include "Wire.h"

// I2Cdev and ADXL345 must be installed as libraries, or else the .cpp/.h files
// for both classes must be in the include path of your project
#include "I2Cdev.h"
#include "ADXL345.h"

// class default I2C address is 0x53
// specific I2C addresses may be passed as a parameter here
// ALT low = 0x53 (default for SparkFun 6DOF board)
// ALT high = 0x1D

ADXL345 accel;

int16_t ax, ay, az;
float Last_reading = 0;
float Damping = 15;
float New_reading = 0;
float Difference = 0;
float G_factor;

//---------------------------------------------------------------------------------------------

String msg_field[30] = { "", "", "", "", "", "" };
int position_1 = 0;
int mysteps = 0;
int smDirectionPin = 3; //Direction pin
int smStepPin = 2; //Stepper pin
int vario;

// standard X25.168 range 315 degrees at 1/3 degree steps
//#define STEPS (315*3)


void setup()
{
  Serial.begin(57600);
  BlueflySerial.begin(57600);
  Serial.println("Start!");
  sentenceIndex = 0;  // resetting all variables
  checking = false;
  checksum = 0;
  checkHex1 = 0;
  checkHex2 = 0;
  checkDigit = 0;
  pinMode(smDirectionPin, OUTPUT);
  pinMode(smStepPin, OUTPUT);
    Wire.begin();
    accel.initialize();
    accel.setRange(0x1);  // +/- 4g
    zero();
}

//------------------------------------------------------------------------------------

void getAccelData() {
    // read raw accel measurements from device
    accel.getAcceleration(&ax, &ay, &az);
    Difference = ((Last_reading + ((az) - 127))/Damping);
    New_reading = Last_reading - Difference;
    Last_reading = New_reading;
    G_factor = -((Last_reading)/-127 + 0.900);
//    Serial.println(G_factor);
//    delay(17);
}


//---------------------------------------------------------------------------------------------
void loop()
{
  getGpsData();
//  Serial.println(millis());
  // This gets called repeatedly. As soon as it's finished
  // it's called again.
}

//---------------------------------------------------------------------------------------------
void getGpsData()
{
  while (BlueflySerial.available())
  // When there's no data available this function quits.
  {
    char character = BlueflySerial.read(); // get a single character
    switch (character) // decide based on the character received
    {
    case 10: // ASCII line feed 'LF'
      continue;  // just ignore it, restart at while()

    case 13: // ASCII carriage return 'CR'
      // This is the end of the sentence. Process the entire
      // sentence and restart for the next sentence.
//      if (isValid()) {    // see function below
//        Serial.print("Valid:   ");
//      }else{
//        Serial.print("INVALID: ");
//      }
     // printSentence(sentence, sentenceIndex);
      sentence[sentenceIndex++] = 0; 
      getCSVfields();
   //    Serial.println(sentence);
      if ((msg_field[0]) == "$LK8EX1"){
        Serial.println("found $LK8EX1");
      vario = (msg_field[3].toInt());
    //  Serial.println(millis());
      //Serial.println(vario);
        if (((((vario * 25)/10) > -2500) && ((vario * 25/10) < 2500)) && isValid())
      {
      getAccelData();
      mysteps = (position_1 - ((vario * 25)/10 + ((int) ((G_factor - 1.00) * 500))));
     // mysteps = (position_1 - (vario * 3));
      rotate(-mysteps, 0.5);
      position_1 = ((vario * 25/10) + ((int) ((G_factor - 1.00) * 500)));
      }
      }
      else {
        Serial.println("didn't find LX8EX1");
      
      }
      sentenceIndex = 0;  // restart the sentence position
      checksum = 0;  // reset the checksum
      continue; // go back to while()
        
    case 42: // ASCII asterisk '*'
      // Stop adding characters to the sentence and instead
      // collect the check digit information.
      checking = false; // don't add remaining characters to 'checksum'
      checkDigit = 1; // next character will be first check digit
      continue;
    default:
      break;

      // END OF SWITCH
    }
    if (checkDigit == 1)
    {
      checkHex1 = character;
      checkDigit++; // next character will be checkDigit 2
    }
    else if (checkDigit == 2)
    {
      checkHex2 = character;
      checkDigit = 0; // no more check digits
    }
    else sentence[sentenceIndex++] = character;
      // Append the current character to the sentence
    if (sentenceIndex == GPS_BUFFER_SIZE)
      // Check we haven't overrun the sentence array, if so, restart
    {
      sentenceIndex = 0;
      checksum = 0;
      continue;
    }
    if (checking) checksum ^= character;
      // X-OR character with previous X-OR sum
    if (character == 36) checking = true; // ASCII Dollar '$'
      // Leading $ is not included in checksum calculation, but
      // does indicate when to start calculating the checksum.
  }
}

//---------------------------------------------------------------------------------------------
boolean isValid()
// Compares the calculated [chechsum] to the check digits [checkHex1, checkHex2]
// supplied by the GPS. If the calculation matches the sentence is valid.
{
  return (charToHex(checkHex1) == (checksum >> 4) && charToHex(checkHex2) == (checksum & 0x0F));
}

//---------------------------------------------------------------------------------------------
byte charToHex(char in)
// Converts a hexadecimal character 
// {'0', '1', '2', '3', '4', '5', '6', '7', '8', '9', 'A', 'B', 'C', 'D', 'E', 'F'}
// into its numeric equivalent
// { 0,   1,   2,   3,   4,   5,   6,   7,   8,   9,  10,  11,  12,  13,  14,  15}
{
  if (in >= '0' && in <= '9') return in - '0';
  if (in >= 'A' && in <= 'F') return in - 'A' + 10;
  return 0;
}

//---------------------------------------------------------------------------------------------
void printSentence(char *sentence, byte length)
// Prints a specified number of characters from a character array
// and shows any non-printing characters by their ASCII number.
{
  for(int i = 0; i < length; i++)
  {
    if (sentence[i] < 32)
    {
      Serial.print('<');
      Serial.print(sentence[i], DEC);
      Serial.print('>');
    }
    else
    {
      Serial.print(sentence[i]);
      
    }
  }
}

byte getCSVfields()
{
    byte _sentencePos = 0;
    byte _comma_count = 0;
    msg_field[_comma_count] = "";
    while (1)
    {
        if (sentence[_sentencePos] == 0) break;
        if (sentence[_sentencePos] == 44) //comma
        {
            _comma_count++;
            msg_field[_comma_count] = "";
            _sentencePos++;
        }
        else
        {
            msg_field[_comma_count] += sentence[_sentencePos];
            _sentencePos++;
        }
    }
    return _comma_count + 1;
}

/*The rotate function turns the stepper motor. Tt accepts two arguments: 'steps' and 'speed'*/
void rotate(int steps, float speed){
  /*This section looks at the 'steps' argument and stores 'HIGH' in the 'direction' variable if */
  /*'steps' contains a positive number and 'LOW' if it contains a negative.*/
  int direction;
 
  if (steps > 0){
    direction = HIGH;
  }else{
    direction = LOW;
  }
 
  speed = 1/speed * 70; //Calculating speed
  steps = abs(steps); //Stores the absolute value of the content in 'steps' back into the 'steps' variable
 
  digitalWrite(smDirectionPin, direction); //Writes the direction (from our if statement above), to the EasyDriver DIR pin
 
  /*Steppin'*/
  for (int i = 0; i < steps; i++){
    digitalWrite(smStepPin, HIGH);
    delayMicroseconds(speed);
    digitalWrite(smStepPin, LOW);
    delayMicroseconds(speed);
  }
}

void zero(){
  // run the motor against the stops
digitalWrite(smDirectionPin, HIGH); //Writes the direction to the EasyDriver DIR pin. (HIGH is clockwise).
unsigned long  set_time = 0; 
set_time = millis();
while((millis() - set_time) < 6000) {
//  Serial.println((millis() - set_time));
digitalWrite(smStepPin, HIGH);
    delayMicroseconds(600);
    digitalWrite(smStepPin, LOW);
    delayMicroseconds(400);  
}
  delay(2000); //Pauses for a second (the motor does not need to pause between switching direction, so you can safely remove this)
 
  digitalWrite(smDirectionPin, LOW); //Writes the direction to the EasyDriver DIR pin. (LOW is counter clockwise).
  /*Turns the motor fast 2500 steps*/
  for (int i2 = 0; i2 < 2500; i2++){
    digitalWrite(smStepPin, HIGH);
    delayMicroseconds(100);
    digitalWrite(smStepPin, LOW);
    delayMicroseconds(100);
    //delay(1000);
  }
  delay(1000);
}

