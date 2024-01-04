#include <Wire.h>
#include <math.h>

// Pin declares
int pwmPin = 5; // PWM output pin for motor 1
int dirPin = 8; // direction output pin for motor 1

// Kinematics variables
double xh = 0;           // position of the handle [m]
// Force output variables
double force = 0;           // force at the handle
double Tp = 0;              // torque of the motor pulley
double duty = 0;            // duty cylce (between 0 and 255)
unsigned int output = 0;    // output command to the motor
//---------------------------------------------------------------------------
// Encoder variables
int magnetStatus = 0;       //value of the status register (MD, ML, MH)
int lowbyte;                //raw angle 7:0
word highbyte;              //raw angle 7:0 and 11:8
int rawAngle;               //final raw angle 
float degAngle;             //raw angle in degrees (360/4096 * [value between 0-4095])
int quadrantNumber, previousquadrantNumber; //quadrant IDs
float numberofTurns = 0;                    //number of turns
float correctedAngle = 0;                   //tared angle - based on the startup value
float startAngle = 0;                       //starting angle
float totalAngle = 0;                       //total absolute angular displacement
float previoustotalAngle = 0;               //for the display printing
float filteredAngle = 0;

int counter = 0;



void setup()
{
  Serial.begin(112500); //start serial - tip: don't use serial if you don't need it (speed considerations)
 
  setPwmFrequency(pwmPin,1); 

  // Output pins`
  pinMode(pwmPin, OUTPUT);  // PWM pin for motor A
  pinMode(dirPin, OUTPUT);  // dir pin for motor A
  
  // Initialize motor 
  analogWrite(pwmPin, 0);     // set to not be spinning (0/255)
  digitalWrite(dirPin, LOW);  // set direction


  Wire.begin(); //start i2C  
	Wire.setClock(800000L); //fast clock

  checkMagnetPresence(); //check the magnet (blocks until magnet is found)

  ReadRawAngle(); //make a reading so the degAngle gets updated
  startAngle = degAngle; //update startAngle with degAngle - for taring  
  }

void loop()
{
  //*************************************************************
  //*** Section 1. Function calls to compute encoder readings
  //*************************************************************

  // Encoder code
  ReadRawAngle(); //ask the value from the sensor
  correctAngle(); //tare the value
  checkQuadrant(); //check quadrant, check rotations, check absolute angular position


  //*************************************************************
  //*** Section 2. Compute position in meters *******************
  //*************************************************************
  
  // Define kinematic parameters you may need
  // These were recently changed
  double rh = 0.095;   //[m]
  double rp = 0.005;   // Radius of motor pulley [m]
  double rs = 0.073152;       // Radius of sector puelly [m] 
  
  // Calibration equation
//  double ts = 0.0102*updatedPos - 2.4137;// Compute the angle of the sector pulley (ts) in degrees based on updatedPos (for MR Sensor) 
//  double ts = -0.0975*totalAngle + 1.7176; // Compute the angle of the sector pulley (ts) in degrees based on totalAngle (for encoder)
  double ts = -0.0975*filteredAngle + 1.7176; // Compute the angle of the sector pulley (ts) in degrees based on totalAngle (for encoder)
  // Handle position
  xh = rh*ts *(3.14 / 180);       // Compute the position of the handle (in meters) based on ts (in radians). 
                                   // NOTE: A negative sign was included to ensure that righward motions on the hapkit handle
                                   // correspons to positive xh values. 
                                     
  // Step B.8: print xh via serial monitor
  //other values are also printed to assist with debugging if needed
  // Comment serial print statements out before running motor 
//  Serial.print(" force: ");
//  Serial.print(force); 
//  Serial.print(" encoderReading: ");
//  Serial.print(totalAngle, 2);
//  Serial.print(" xh: ");
//  Serial.println(xh, 5);

  
  //*************************************************************
  //*** Section 3. Assign a motor output force in Newtons *******  
  //*************************************************************
 
  Tp = ((rh*rp) / (rs))*force;    // Compute the require motor pulley torque (Tp) to generate that force

  //*************************************************************
  //******************* Rendering Algorithms ********************
  //*************************************************************

  //-------------------------------------------------------------
  //--- This next block of code computes an estimate of the handle velocity 
  //--- and implements a digital low-pass filter on the signal
  //-------------------------------------------------------------
  
  // Discrete differentiation of position signal to obtain handle velocity
  static double hxPrev = 0;         // Variable to keep track of previous handle position. Static variable is used to preserve 
                                    // its value between each function (i.e. in between loops)
  static double timePrev = 0;       // Variable to keep track of previous time. 
  double t = millis();              // Capture time since the program has started
  double deltaTime = t - timePrev;  // Compute period of the control loop 
  
  double vh = (xh - hxPrev) / (0.0001);  // Compute estimate of handle velocity
  hxPrev = xh;     // Keep track of current handle position for next iteration                           
  timePrev = t;    // Keep track of current time since program has started for next iteration
  
  // Digital low pass filter on velocity
  static double vhPrev = 0;                                 // Variable to keep track of previous handle position
  const double alpha = 0.1;                                 // Constant for digital filter
  double vhFiltered = alpha * vh + (1 - alpha) * vhPrev;    // Digital filter
  vhPrev = vhFiltered;                                      // Keep track of filtered handle velocity
  

  //----------------------------------------------
  //--- DEFINES
  //----------------------------------------------
  // Uncomment the environment that you would like to feel
//  #define VIRTUAL_SPRING
  #define VIRTUAL_WALL
//#define LINEAR_DAMPING

//  Serial.println(force);
  //----------------------------------------------
  //--- Virtual Spring
  //----------------------------------------------
  #ifdef VIRTUAL_SPRING
  double xwall = 0;    // Wall position [m]
  double kwall = 200;      // Wall stiffness [N/m]
  force = kwall*(xwall - xh);
  #endif
  
  //----------------------------------------------
  //--- Virtual Wall
  //----------------------------------------------
  #ifdef VIRTUAL_WALL
  double xwall = -0.005;    // Wall position [m]
  double kwall = 200;      // Wall stiffness [N/m]
  if (xh < xwall)
  {
    force = kwall*(xwall - xh);       // Compute force from the virtual wall
  }
  else 
  {
    force = 0;
  }
  if (counter >= 10) {
    counter = 0;
    Serial.println(xh, 5); //print to processing
  } else {
    counter++;
  }
  #endif

  //----------------------------------------------
  //--- Linear Damping
  //----------------------------------------------
  #ifdef LINEAR_DAMPING
  const double b = 0.9;    // Viscous damping coefficient [Ns/m]
  force = -b*vhFiltered;   // Compute damping force
  #endif

  
  //*************************************************************
  //*** Section 4. Force output (do not change) *****************
  //*************************************************************
  if (force > 3)
  {
    force = 3;
  }
  if (force < -3)
  {
    force = -3;
  }
  // Determine correct direction for motor torque
  if(force > 0) { 
    digitalWrite(dirPin, HIGH);
  } else {
    digitalWrite(dirPin, LOW);
  }

  // Compute the duty cycle required to generate Tp (torque at the motor pulley)
  duty = sqrt(abs(Tp)/0.03);  //

  // Make sure the duty cycle is between 0 and 100%
  if (duty > 1) {            
    duty = 1;
  } else if (duty < 0) { 
    duty = 0;
  }    
  output = (int)(duty* 255);   // convert duty cycle to output signal
  analogWrite(pwmPin,output);  // output the signal
  
}

// --------------------------------------------------------------
// Function to set PWM Freq -- DO NOT EDIT
// --------------------------------------------------------------
void setPwmFrequency(int pin, int divisor) 
{
  byte mode;
  if(pin == 5 || pin == 6 || pin == 9 || pin == 10) {
    switch(divisor) {
      case 1: mode = 0x01; break;
      case 8: mode = 0x02; break;
      case 64: mode = 0x03; break;
      case 256: mode = 0x04; break;
      case 1024: mode = 0x05; break;
      default: return;
    }
    if(pin == 5 || pin == 6) {
      TCCR0B = TCCR0B & 0b11111000 | mode;
    } else {
      TCCR1B = TCCR1B & 0b11111000 | mode;
    }
  } else if(pin == 3 || pin == 11) {
    switch(divisor) {
      case 1: mode = 0x01; break;
      case 8: mode = 0x02; break;
      case 32: mode = 0x03; break;
      case 64: mode = 0x04; break;
      case 128: mode = 0x05; break;
      case 256: mode = 0x06; break;
      case 1024: mode = 0x7; break;
      default: return;
    }
    TCCR2B = TCCR2B & 0b11111000 | mode;
  }
}
