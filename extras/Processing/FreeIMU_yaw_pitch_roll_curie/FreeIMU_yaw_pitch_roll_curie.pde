/**
Visualize orientation information

This program is free software: you can redistribute it and/or modify
it under the terms of the version 3 GNU General Public License as
published by the Free Software Foundation.

This program is distributed in the hope that it will be useful,
but WITHOUT ANY WARRANTY; without even the implied warranty of
MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
GNU General Public License for more details.

You should have received a copy of the GNU General Public License
along with this program.  If not, see <http://www.gnu.org/licenses/>.


Original code by Fabio Veresano, original developer of FreeIMU Library
Modified by Michael J Smorto 2/21/16 for use with CurieIMU.

*/

import processing.serial.*;

Serial myPort;  // Create object from Serial class

final String serialPort = "COM3"; // replace this with your serial port. On windows you will need something like "COM1".
int BaudRate=57600;

float [] Euler = new float [3]; // psi, theta, phi
float gx, gy, gz, ax, ay, az, q0, q1, q2 ,q3;
float temp;

int lf = 10; // 10 is '\n' in ASCII
byte[] inBuffer = new byte[22]; // this is the number of chars on each line from the Arduino (including /r/n)

PFont font;
final int VIEW_SIZE_X = 800, VIEW_SIZE_Y = 600;

String val;
boolean Contact = false;

void setup() 
{
  size(800, 600, P3D);
  
  // The font must be located in the sketch's "data" directory to load successfully
  font = loadFont("CourierNew36.vlw");
  
  myPort = new Serial(this, serialPort, BaudRate);  
  
}


void draw() {  
  
  background(#000000);
  fill(#ffffff);
  
  textFont(font, 20);
  //float temp_decoded = 35.0 + ((float) (temp + 13200)) / 280;
  //text("temp:\n" + temp_decoded + " C", 350, 250);
  textAlign(LEFT, TOP);
  //text("Q:\n" + q[0] + "\n" + q[1] + "\n" + q[2] + "\n" + q[3], 20, 10);
  text("Euler Angles:\nYaw (psi)  : " + nfp(degrees(Euler[0]),3,3) + 
                    "\nPitch (theta): " + nfp(degrees(Euler[1]),3,3) + 
                    "\nRoll (phi)  : " + nfp(degrees(Euler[2]),3,3), 200, 10); 

  text("Acceleration\nax : " + nfp(ax,2,5) +
                   "\nay : " + nfp(ay,2,5) +
                   "\naz : " + nfp(az,2,5), 320, 440);
    
  text("Gryo Rates:\ngx : " + nfp(gx,2,5) +
                   "\ngy : " + nfp(gy,2,5) +
                   "\ngz : " + nfp(gz,2,5), 560, 440);
  
  text("Q:\nq0 : " + nfp(q0,2,5) +
                   "\nq1 : " + nfp(q1,2,5) +
                   "\nq2 : " + nfp(q2,2,5) +
                   "\nq3 : " + nfp(q3,2,5) , 80, 440);
  
  
  text("Temp (deg F): " + nfp(temp,2,2), 320, 540);
  
  drawcompass(Euler[0], VIEW_SIZE_X/2 - 250, VIEW_SIZE_Y/2, 200);
  drawAngle(Euler[1], VIEW_SIZE_X/2, VIEW_SIZE_Y/2, 200, "Pitch:");
  drawAngle(Euler[2], VIEW_SIZE_X/2 + 250, VIEW_SIZE_Y/2, 200, "Roll:");


}

float decodeFloat(String inString) {
  byte [] inData = new byte[4];
  
  if(inString.length() == 8) {
    inData[0] = (byte) unhex(inString.substring(0, 2));
    inData[1] = (byte) unhex(inString.substring(2, 4));
    inData[2] = (byte) unhex(inString.substring(4, 6));
    inData[3] = (byte) unhex(inString.substring(6, 8));
  }
      
  int intbits = (inData[3] << 24) | ((inData[2] & 0xff) << 16) | ((inData[1] & 0xff) << 8) | (inData[0] & 0xff);
  return Float.intBitsToFloat(intbits);
}


void serialEvent(Serial p) {
  if(p.available() >= 1) {
    String inputString = p.readStringUntil('\n');
    //print(inputString);
    if (inputString != null && inputString.length() > 0) {      
      String [] inputStringArr = split(inputString, ",");
      if(inputStringArr.length >= 12) { // q1,q2,q3,q4,\r\n so we have 5 elements
        Euler[0] = decodeFloat(inputStringArr[0])*PI/180;
        Euler[1] = decodeFloat(inputStringArr[1])*PI/180;
        Euler[2] = decodeFloat(inputStringArr[2])*PI/180;
        
        ax = decodeFloat(inputStringArr[3]); 
        ay = decodeFloat(inputStringArr[4]);
        az = decodeFloat(inputStringArr[5]);
        
        gx = decodeFloat(inputStringArr[6])*PI/180; 
        gy = decodeFloat(inputStringArr[7])*PI/180;
        gz = decodeFloat(inputStringArr[8])*PI/180;
  
        temp = decodeFloat(inputStringArr[9]);
        
        q0 = decodeFloat(inputStringArr[10]);
        q1 = decodeFloat(inputStringArr[11]);
        q2 = decodeFloat(inputStringArr[12]);
        q3 = decodeFloat(inputStringArr[13]);
      }
    }
  }
}

void myDelay(int time) {
  try {
    Thread.sleep(time);
  } catch (InterruptedException e) { }
}

void buildBoxShape() {
  //box(60, 10, 40);
  noStroke();
  beginShape(QUADS);
  
  //Z+ (to the drawing area)
  fill(#00ff00);
  vertex(-30, -5, 20);
  vertex(30, -5, 20);
  vertex(30, 5, 20);
  vertex(-30, 5, 20);
  
  //Z-
  fill(#0000ff);
  vertex(-30, -5, -20);
  vertex(30, -5, -20);
  vertex(30, 5, -20);
  vertex(-30, 5, -20);
  
  //X-
  fill(#ff0000);
  vertex(-30, -5, -20);
  vertex(-30, -5, 20);
  vertex(-30, 5, 20);
  vertex(-30, 5, -20);
  
  //X+
  fill(#ffff00);
  vertex(30, -5, -20);
  vertex(30, -5, 20);
  vertex(30, 5, 20);
  vertex(30, 5, -20);
  
  //Y-
  fill(#ff00ff);
  vertex(-30, -5, -20);
  vertex(30, -5, -20);
  vertex(30, -5, 20);
  vertex(-30, -5, 20);
  
  //Y+
  fill(#00ffff);
  vertex(-30, 5, -20);
  vertex(30, 5, -20);
  vertex(30, 5, 20);
  vertex(-30, 5, 20);
  
  endShape();
}


void drawcompass(float heading, int circlex, int circley, int circlediameter) {
  noStroke();
  ellipse(circlex, circley, circlediameter, circlediameter);
  fill(#ff0000);
  ellipse(circlex, circley, circlediameter/20, circlediameter/20);
  stroke(#ff0000);
  strokeWeight(4);
  line(circlex, circley, circlex - circlediameter/2 * sin(-heading), circley - circlediameter/2 * cos(-heading));
  noStroke();
  fill(#ffffff);
  textAlign(CENTER, BOTTOM);
  text("N", circlex, circley - circlediameter/2 - 10);
  textAlign(CENTER, TOP);
  text("S", circlex, circley + circlediameter/2 + 10);
  textAlign(RIGHT, CENTER);
  text("W", circlex - circlediameter/2 - 10, circley);
  textAlign(LEFT, CENTER);
  text("E", circlex + circlediameter/2 + 10, circley);
}


void drawAngle(float angle, int circlex, int circley, int circlediameter, String title) {
  angle = angle + PI/2;
  
  noStroke();
  ellipse(circlex, circley, circlediameter, circlediameter);
  fill(#ff0000);
  strokeWeight(4);
  stroke(#ff0000);
  line(circlex - circlediameter/2 * sin(angle), circley - circlediameter/2 * cos(angle), circlex + circlediameter/2 * sin(angle), circley + circlediameter/2 * cos(angle));
  noStroke();
  fill(#ffffff);
  textAlign(CENTER, BOTTOM);
  text(title, circlex, circley - circlediameter/2 - 30);
}