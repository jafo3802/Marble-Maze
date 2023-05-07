//ACCELEROMETER CODE
#include "Wire.h" // This library allows you to communicate with I2C devices.

const int MPU_ADDR = 0x68; // I2C address of the MPU-6050. If AD0 pin is set to HIGH, the I2C address will be 0x69.

int16_t accelerometer_x, accelerometer_y, accelerometer_z; // variables for accelerometer raw data
int16_t gyro_x, gyro_y, gyro_z; // variables for gyro raw data
int16_t temperature; // variables for temperature data

char tmp_str[7]; // temporary variable used in convert function

char* convert_int16_to_str(int16_t i) { // converts int16 to string. Moreover, resulting strings will have the same length in the debug monitor.
  sprintf(tmp_str, "%6d", i);
  return tmp_str;
}

//SERVO CODE
#include <Servo.h>

Servo servoX;  // create servo object to control a servo
Servo servoY;


const int xPin = 9;
const int yPin = 10;

const float alpha = 0.9;
double data_filteredX[] = {0, 0};
double data_filteredY[] = {0,0};
const int n = 1;
const int m = 1;

//LCD CODE
#include <LiquidCrystal.h>
//initialise the library with the numbers of the interface pins
LiquidCrystal lcd(12, 11, 5, 4, 3, 2);

//PHOTORESISTOR CODE
const int pResistor = A0; // Photoresistor at Arduino analog pin A0

//Variables
int value;          // Store value from photoresistor (0-1023)

//BUTTON

int button = 0;
int buttonPin = 6;
bool buttonState = 0;

//DEBOUNCING

int lastButtonState = LOW;  // the previous reading from the input pin

unsigned long lastDebounceTime = 0;  // the last time the output pin was toggled
unsigned long debounceDelay = 50;    // the debounce time; increase if the output flickers


void setup() {
  servoX.attach(9);  // attaches servo connected to Arduino pin 9 to the myservo object
  servoY.attach(10);
  pinMode(xPin,INPUT);
  pinMode(yPin,INPUT);
  pinMode(buttonPin, INPUT);

  Serial.begin(9600);
  Wire.begin();
  Wire.beginTransmission(MPU_ADDR); // Begins a transmission to the I2C slave (GY-521 board)
  Wire.write(0x6B); // PWR_MGMT_1 register
  Wire.write(0); // set to zero (wakes up the MPU-6050)
  Wire.endTransmission(true);

  pinMode(pResistor, INPUT);// Set pResistor - A0 pin as an input (optional)
  Serial.begin(9600);
  Serial.println("here");
}

  

void loop() {
  int loopStartOver = millis();
  Serial.println("start");

  lcd.autoscroll();
  lcd.begin(16, 2);


  lcd.print("Press the Red");
  lcd.setCursor(0, 1);
  lcd.print("Button to Play!");

  delay(100);

 
  
  buttonState = digitalRead(buttonPin);

  
  Serial.println(buttonState);

  if(buttonState == true){
    buttonState = true;
    int trueTime = millis();
    int gameStart = millis();

    lcd.autoscroll();
    // set up the LCD's number of columns and rows:
    lcd.clear();
    lcd.begin(16, 2);
    
    // Print a message to the LCD.
    lcd.setCursor(0, 1);


    lcd.print("ready?");
    delay(3000);
    lcd.clear();
    lcd.print("3");
    delay(1000);
    lcd.clear();
    lcd.print("2");
    delay(1000);
    lcd.clear();
    lcd.print("1");
    delay(1000);
    lcd.clear();

    lcd.print("GO!!!!!!!!");

    Serial.println("Button push");

    int timer = millis();

    while (buttonState == true){
      
      int startTime = millis();
      Serial.println(startTime);

      Serial.println("Playing");
    
      Wire.beginTransmission(MPU_ADDR);
      Wire.write(0x3B); // starting with register 0x3B (ACCEL_XOUT_H) [MPU-6000 and MPU-6050 Register Map and Descriptions Revision 4.2, p.40]
      Wire.endTransmission(false); // the parameter indicates that the Arduino will send a restart. As a result, the connection is kept active.
      Wire.requestFrom(MPU_ADDR, 7*2, true); // request a total of 7*2=14 registers
      
      // "Wire.read()<<8 | Wire.read();" means two registers are read and stored in the same variable
      accelerometer_x = Wire.read()<<8 | Wire.read(); // reading registers: 0x3B (ACCEL_XOUT_H) and 0x3C (ACCEL_XOUT_L)
      accelerometer_y = Wire.read()<<8 | Wire.read(); // reading registers: 0x3D (ACCEL_YOUT_H) and 0x3E (ACCEL_YOUT_L)
      accelerometer_z = Wire.read()<<8 | Wire.read(); // reading registers: 0x3F (ACCEL_ZOUT_H) and 0x40 (ACCEL_ZOUT_L)
      temperature = Wire.read()<<8 | Wire.read(); // reading registers: 0x41 (TEMP_OUT_H) and 0x42 (TEMP_OUT_L)
      gyro_x = Wire.read()<<8 | Wire.read(); // reading registers: 0x43 (GYRO_XOUT_H) and 0x44 (GYRO_XOUT_L)
      gyro_y = Wire.read()<<8 | Wire.read(); // reading registers: 0x45 (GYRO_YOUT_H) and 0x46 (GYRO_YOUT_L)
      gyro_z = Wire.read()<<8 | Wire.read(); // reading registers: 0x47 (GYRO_ZOUT_H) and 0x48 (GYRO_ZOUT_L)

      int xValue = map(accelerometer_x, -16000,16000, 20,120 );
      int yValue = map(accelerometer_y, -16000,16000, 0,120);
      //float data_filtered [n] = alpha * data + (1 - alpha) * data_filtered [n-1]; 

      data_filteredX[n] = alpha * xValue + (1 - alpha) * data_filteredX[n-1];
      data_filteredX[n-1] = data_filteredX[n];
      data_filteredY[m] = alpha * yValue + (1 - alpha) * data_filteredY[m-1];
      data_filteredY[m-1] = data_filteredY[m];
      
      Serial.print("x: ");
      Serial.print(xValue);
      Serial.print(" y: ");
      Serial.println(yValue);
      //int turn = analogRead(pot);
      servoX.write(data_filteredX[n]);              
      servoY.write(data_filteredY[m]);
      
      delay(150);
      
      int millisCount = millis();
      lcd.setCursor(0, 1);
      // print the number of seconds since reset:
      //int decimals = ((millis()-10000) % 1000);
      //lcd.print(((millis() - timer)/1000));
      //lcd.print(".");
      //lcd.print(decimals);

      //output statements at end of round
      Serial.print("Your time is: ");
      //Serial.print(((millis() - timer) / 1000));
      Serial.println("seconds");

      value = analogRead(pResistor);
      value = map(value, 0, 1023, 0, 100);
      
      Serial.println(value);
      
      if(value <= 13){
        servoX.write(75);              
        servoY.write(65);
        
        lcd.clear();
        lcd.print("Time: ");
        //int time = ((((millis() - timer) / 1000)));
        //lcd.print(time-65);
        //int lagTime = gameStart - loopStartOver;
        //lcd.print((millis()-lagTime)/1000); 
        int falseTime = millis(); 
        lcd.print(((falseTime - trueTime - 6000)/1000));      
        lcd.print(" seconds");
        delay(5000);
      
        buttonState = false;

        break;        
      } 
      
    }    
  }
}



