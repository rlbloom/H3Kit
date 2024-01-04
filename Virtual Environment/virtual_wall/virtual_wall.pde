// Pro_Graph2.pde
/*
 Based on the Arduining example which is based on the Tom Igoe example.
 Mofified by Cara Nunez 5/1/2019:
  -A wider line was used. strokeWeight(4);
  -Continuous line instead of vertical lines.
  -Bigger Window size 600x400.
 Modified by Tania Morimoto 2/10/2021:
  - more detailed comments/instructions
-------------------------------------------------------------------------------
This program takes ASCII-encoded strings
from the serial port at 9600 baud and graphs them. It expects values in the
range 0 to 1023, followed by a newline, or newline and carriage return


*/

import processing.serial.*;

String myString = null;

Serial myPort;        // The serial port

//initialize variables
float inByte = 0;
float lastByte = 0;


void setup () {
  // set the window size:
  size(1080, 720);        

  // List all the available serial ports
  printArray(Serial.list());
  
  // Check the listed serial ports in your machine
  // and use the correct index number in Serial.list()[] below.
  // Note that these are indexed from 0, and you are looking for the same port as your ardunio.
  myPort = new Serial(this, Serial.list()[0], 115200);  //make sure baud rate matches Arduino

  // A serialEvent() is generated when a newline character is received :
  myPort.bufferUntil('\n');
  background(0);      // set inital background:
}
void draw () {
  // everything happens in the serialEvent()
  background(0); //uncomment if you want to control a ball
  stroke(127,34,255);     //stroke color
  strokeWeight(10);        //stroke wider
  
  // START EDITING HERE
  
  // Virtual Wall
  // map the wall position from units of Arduino simulation to the screen width.
  // HINT: use map(myValue, minValueIn, maxValueIn, minValueOut, maxValueOut) to map from units of your Arduino simulation to pixels
  float x_wall = map(-0.005, -0.045, 0.055, 0, width);
  
  // draw the wall as a line
  line(x_wall, 0, x_wall, height);  
  
  // draw an ellipse to represent user position
  // if the handle position is less than the wall position, the ellipse follows the handle
  if (inByte > (x_wall + 25))
  {
    ellipse(inByte, height/2, 50, 50);
  }
  // Otherwise, the ellipse stops at the wall
  else 
  {
    ellipse(x_wall+25, height/2, 50, 50);  // An offset of 25/2 was added so that the user's elliple does not penetrate the wall
  }
}

void serialEvent (Serial myPort) {
  // get the ASCII string:

  // read the input string
  // HINT: use myPort.readStringUntil('\n')  (https://processing.org/reference/libraries/serial/Serial_readStringUntil_.html)
  myString = myPort.readStringUntil('\n');
  
  // trim using trim()
  myString.trim();
  
  // convert string to a number
  inByte = float(myString);
  
  // if: the number is NaN, set current value to previous value
  if (inByte == Float.NaN)
  {
    inByte = lastByte;        
  }
  // otherwise: map the new value to the screen width & update previous value variable
  else
  {
    inByte = map(inByte, -0.033, 0.041, 0, width);
    lastByte = inByte;
  }
   println(inByte);
  //STOP EDITING HERE
}
