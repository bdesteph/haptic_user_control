/**
 **********************************************************************************************************************
 * @file       sketch_2_Hello_Wall.pde
 * @author     Steve Ding, Colin Gallacher
 * @version    V3.0.0
 * @date       08-January-2021
 * @brief      Wall haptic example with programmed physics for a haptic wall 
 **********************************************************************************************************************
 * @attention
 *
 *
 **********************************************************************************************************************
 */
 
  /* library imports *****************************************************************************************************/ 
import processing.serial.*;
import static java.util.concurrent.TimeUnit.*;
import java.util.concurrent.*;
import oscP5.*;
import netP5.*;
/* end library imports *************************************************************************************************/  


/* scheduler definition ************************************************************************************************/ 
private final ScheduledExecutorService scheduler      = Executors.newScheduledThreadPool(1);
/* end scheduler definition ********************************************************************************************/ 



/* device block definitions ********************************************************************************************/
Board             haplyBoard;
Device            widgetOne;
Mechanisms        pantograph;

byte              widgetOneID                         = 5;
int               CW                                  = 0;
int               CCW                                 = 1;
boolean           renderingForce                     = false;
/* end device block definition *****************************************************************************************/



/* framerate definition ************************************************************************************************/
long              baseFrameRate                       = 120;
/* end framerate definition ********************************************************************************************/ 



/* elements definition *************************************************************************************************/

/* Screen and world setup parameters */
float             pixelsPerMeter                      = 4000.0;
float             radsPerDegree                       = 0.01745;

/* pantagraph link parameters in meters */
float             l                                   = 0.07;
float             L                                   = 0.09;


/* end effector radius in meters */
float             rEE                                 = 0.006;

/* joystick circle parameter */
int radius = 100;
PVector penX = new PVector(0, 0);
PVector penY = new PVector(0, 0);

/* virtual wall parameter  */
float             kWall                               = 450;
float             kLine1                               = 50;
float             kLine2                               = 100;
PVector           fWall                               = new PVector(0, 0);
PVector           fPos                           = new PVector(0, 0);
PVector           fLine                               = new PVector(0, 0);
PVector           penWall                             = new PVector(0, 0);
PVector           penLine                             = new PVector(0, 0);
PVector           posWall                             = new PVector(0.01, 0.10);

PVector           posLine                               = new PVector(0.01, 0.08);

/* slider parameters */
Slider s;
float xSlider; // x position
float mSlider = 0.15; // mass in kg
PVector accelerationSlider = new PVector(0, 0); // acceleration
PVector forceSlider = new PVector(0, 0); // force in N (kg m s−2)
PVector userForceSlider = new PVector(0, 0);
PVector sumUserForceSlider = new PVector(0, 0);

FloatList sliderPositions = new FloatList();
FloatList sliderSpeeds = new FloatList();
int positionCounter = 0;

float velocity = 0;
int countvelocity = 0;

PVector magneticForce = new PVector(0, 0);

float sinTheta = 0;
//boolean 
int sinSwitch = 1;
boolean firstPart = true;
boolean sinPositive = true;

/* joystick parameters */
float x_start = 0.01;
float x_max = 0.091;

/* OSC server elements */
OscP5 oscP5;
OscP5 oscP52;
NetAddress myRemoteLocation;

/* generic data for a 2DOF device */
/* joint space */
PVector           angles                              = new PVector(0, 0);
PVector           torques                             = new PVector(0, 0);

/* task space */
PVector           posEE                               = new PVector(0, 0);
PVector posY = new PVector(0, 0); 
PVector           fEE                                 = new PVector(0, 0); 

/* device graphical position */
PVector           deviceOrigin                        = new PVector(0, 0);

/* World boundaries reference */
final int         worldPixelWidth                     = 800;
final int         worldPixelHeight                    = 650;

boolean start = false;
boolean firstElement = true;
boolean secondElement = false;

int last_timer;
int max_time = 0;
int deltaTime;

/* graphical elements */
PShape pGraph, joint, endEffector, sliderCursor;
PShape wall, line2, circle;
/* end elements definition *********************************************************************************************/ 


/* setup section *******************************************************************************************************/
void setup(){
  /* put setup code here, run once: */
  
  /* screen size definition */
  size(800, 650);
  
  /* device setup */
  
  /**  
   * The board declaration needs to be changed depending on which USB serial port the Haply board is connected.
   * In the base example, a connection is setup to the first detected serial device, this parameter can be changed
   * to explicitly state the serial port will look like the following for different OS:
   *
   *      windows:      haplyBoard = new Board(this, "COM10", 0);
   *      linux:        haplyBoard = new Board(this, "/dev/ttyUSB0", 0);
   *      mac:          haplyBoard = new Board(this, "/dev/cu.usbmodem1411", 0);
   */ 
   // haplyBoard          = new Board(this, "COM7", 0);
   //haplyBoard          = new Board(this, Serial.list()[0], 0);
  haplyBoard          = new Board(this, "/dev/cu.usbmodem14201", 0);
  widgetOne           = new Device(widgetOneID, haplyBoard);
  pantograph          = new Pantograph();
  
  widgetOne.set_mechanism(pantograph);
  
  widgetOne.add_actuator(1, CCW, 2);
  widgetOne.add_actuator(2, CW, 1);
 
  widgetOne.add_encoder(1, CCW, 241, 10752, 2);
  widgetOne.add_encoder(2, CW, -61, 10752, 1);
  
  widgetOne.device_set_parameters();
  
  myRemoteLocation = new NetAddress("127.0.0.1", 5005);
  
  /* OSC server setup */
  // oscP5 = new OscP5(this,1234);
  oscP52 = new OscP5(this,1234);
  
  /* visual elements setup */
  background(0);
  deviceOrigin.add(worldPixelWidth/2, 0);
  /* create pantagraph graphics */
  create_pantagraph();
  
  /* create wall graphics */
  
  
  wall = create_wall(posWall.x-0.2, posWall.y+rEE, posWall.x+0.2, posWall.y+rEE);
  wall.setStroke(color(0));

  
  line2 = create_wall(posLine.x-0.2, posLine.y-rEE, posLine.x+0.2, posLine.y-rEE);
  line2.setStroke(color(128));
  
  /* setup framerate speed */
  frameRate(baseFrameRate);
  
  s = new Slider(-0.085, 0.13);
  
  /* setup simulation thread to run at 1kHz */ 
  SimulationThread st = new SimulationThread();
  scheduler.scheduleAtFixedRate(st, 1, 1, MILLISECONDS);
}
/* end setup section ***************************************************************************************************/

/* With a screen width of 800 x 650px/m, posEE.x goes from -0.096 to 0.096,posEE.y from 0.022 to 0.15

/* draw section ********************************************************************************************************/
void draw(){
  /* put graphical code here, runs repeatedly at defined framerate in setup, else default at 60fps: */
  if(renderingForce == false){
    background(255); 
    update_animation(angles.x*radsPerDegree, angles.y*radsPerDegree, posEE.x, posEE.y);
  }
}
/* end draw section ****************************************************************************************************/



/* simulation section **************************************************************************************************/
class SimulationThread implements Runnable{
  
  public void run(){
    /* put haptic simulation code here, runs repeatedly at 1kHz as defined in setup */
    
    renderingForce = true;
    
    /* sinusoid force simulation part */ 
    float aSinusoid;
    float posSinusoid;
    
    /* TO CHANGE this code segment is used to inverse the sinusod at each phase, so it goes up then down, then down and up etc... */
    
    // first part of the sine wave
    if (firstPart == true) {
      if (sinPositive == true) {
        if (sin(sinTheta) < 0) {
          firstPart = false;
          sinPositive = false;
        }
      }
      else {
        if (sin(sinTheta) > 0) {
          firstPart = false;
          sinPositive = true;
        }
      }
    } // second partof the sine wave
    else {
      if (sinPositive == false) {
        if (sin(sinTheta) > 0) {
          firstPart = true;
          sinPositive = true;
          sinSwitch = -1 * sinSwitch;
          // print("PHASE ");
        }
      }
      else {
        if (sin(sinTheta) < 0) {
          firstPart = true;
          sinPositive = true;
          sinSwitch = -1 * sinSwitch;
          // print("PHASE ");
        }
      }
    }

    forceSlider.mult(0);
    
    if(haplyBoard.data_available()){
      /* GET END-EFFECTOR STATE (TASK SPACE) */
      widgetOne.device_read_data();
    
      angles.set(widgetOne.get_device_angles()); 
      posEE.set(widgetOne.get_device_position(angles.array()));
      posEE.set(device_to_graphics(posEE)); 
      
      
      /* haptic wall force calculation */
      fWall.set(0, 0);
      posY.set(0, posEE.y);
      fPos.set(0, 0);
      fLine.set(0, 0);
      
      penWall.set(0, (posWall.y - (posEE.y + rEE)));
      
      if(penWall.y < 0){
        //fWall = fWall.add(penWall.mult(-kWall));  
        fWall = fWall.add(penWall.mult(-kWall));  
      }
      
      if (start == true) {
        // fPos = fPos.add(s.getVelocity());
        penLine.set(0, (posLine.y - (posEE.y - rEE)));
        if(penLine.y > 0){
          // fLine = fLine.add(penLine.mult(-kWall));  
        }
        
        // fPos = (grid_to_force(pos_to_grid(posEE))).mult(-4);
      
        // fPos can't be over 2 because it would be to much
        if (fPos.x > 2.0) {
          fPos.set(2, 0);
        }
        if (fPos.x < -2.0) {
          fPos.set(-2, 0);
        }
      }
      
      // is the cursor in the rest zone? if no, then the slider force will apply as well on the cursor
      if (posEE.x < -0.01 || posEE.x > 0.01) {
        fEE = (fWall.copy().add(fLine).add(fPos)).mult(-1);
      }
      else {
        fEE = (fWall.copy().add(fLine)).mult(-1);
      }
      
      float f = 1000;
      magneticForce.set(0.000001, 0); // 0.00001
      
      aSinusoid = (-pow(0.005, 2) * sin(sinTheta));
      posSinusoid = sin(sinTheta);

      // aSinusoid = sinSwitch * (sin(sinTheta) * 0.000000675); // 0.0000000135 parcourt bien tout en x avec sinTheta += 0.0005
      // 0.000000027 parcourt bien tout en x avec sinTheta += 0.001
      // 0.000000675 parcourt bien tout en x avec sinTheta += 0.005
      
      if (start == true) {
        if (firstElement) {
          OscMessage myMessage = new OscMessage("/getPosition");
          myMessage.add(aSinusoid);
          oscP52.send(myMessage, myRemoteLocation);

          sinTheta += 0.005;
          forceSlider.add(aSinusoid, 0);
          firstElement = false;
          secondElement = true; 
        }
        if (secondElement) {
          OscMessage myMessage = new OscMessage("/getPosition");
          myMessage.add(aSinusoid);
          oscP52.send(myMessage, myRemoteLocation);
          
          last_timer = millis();

          sinTheta += 0.005;
          forceSlider.add(aSinusoid, 0);
          secondElement = false;
        }
        if (sliderPositions.size() >= positionCounter+2) {
          // forceSlider.add(sliderPositions.get(positionCounter), 0);
          print(" In, size: ", sliderPositions.size(), " counter: ", positionCounter);
          // print(millis() - last_timer, " ");
          last_timer = millis();
          sinTheta += 0.005;

          OscMessage myMessage = new OscMessage("/getPosition");
          myMessage.add(aSinusoid);
          oscP52.send(myMessage, myRemoteLocation);
          
          
          forceSlider.add(sliderPositions.get(positionCounter), 0);
          // forceSlider.add(aSinusoid, 0);
          positionCounter += 1;
          // cursorPercentage is the user's position explained 
          float cursorPercentage = (posEE.x + rEE - x_start) / (x_max - x_start);
          
          if (cursorPercentage < -1) {
            cursorPercentage = -1;
          }
          if (cursorPercentage > 1) {
            cursorPercentage = 1;
          }
  
          if (cursorPercentage >= 0) {
              // attracted to the right
              userForceSlider = magneticForce.mult(pow(cursorPercentage, 2));
          } else {
              // attracted to the left
               userForceSlider = magneticForce.mult(-pow(cursorPercentage, 2));
          }
  
          // if the cursor is out the rest zone, then the user is expressing a force
          if (posEE.x < -0.01 || posEE.x > 0.01) {
            /*
            userForceSlider = forceSlider.copy();
            userForceSlider.add(forceSlider.copy().mult(cursorPercentage));
            forceSlider = userForceSlider;
            */
            forceSlider.add(userForceSlider);
            sumUserForceSlider.add(userForceSlider);
          } else {
            s.applyForce(sumUserForceSlider.mult(-1));
            sumUserForceSlider.mult(0);
          }
  
          s.applyForce(forceSlider);
          s.update();
        }
      }
            
      // mult(5000) is nice with a classic sinusoid but I can't base it on multiplication as velocity can be wayyyyyy bigger

      fEE.set(graphics_to_device(fEE));
      /* end haptic wall force calculation */
    }

    torques.set(widgetOne.set_device_torques(fEE.array()));
    widgetOne.device_write_torques();
  
    renderingForce = false;
  }
}
/* end simulation section **********************************************************************************************/


/* helper functions section, place helper functions here ***************************************************************/
void create_pantagraph(){
  float lAni = pixelsPerMeter * l;
  float LAni = pixelsPerMeter * L;
  float rEEAni = pixelsPerMeter * rEE;
  
  pGraph = createShape();
  pGraph.beginShape();
  pGraph.fill(255);
  pGraph.stroke(0);
  pGraph.strokeWeight(2);
  
  pGraph.vertex(deviceOrigin.x, deviceOrigin.y);
  pGraph.vertex(deviceOrigin.x, deviceOrigin.y);
  pGraph.vertex(deviceOrigin.x, deviceOrigin.y);
  pGraph.vertex(deviceOrigin.x, deviceOrigin.y);
  pGraph.endShape(CLOSE);
  
  joint = createShape(ELLIPSE, deviceOrigin.x, deviceOrigin.y, rEEAni, rEEAni);
  joint.setStroke(color(0));
  
  endEffector = createShape(ELLIPSE, deviceOrigin.x, deviceOrigin.y, 2*rEEAni, 2*rEEAni);
  endEffector.setStroke(color(0));
  strokeWeight(5);
  
}

class Slider {
  PVector location;
  PVector velocity;
  PVector acceleration;
  
  Slider(float x, float y) {
    location = new PVector(x, y);
    velocity = new PVector(0, 0);
    acceleration = new PVector(0, 0);  
  }
  
  void update() {
    // if the slider must go off the limits, the location stays the same and velocity needs to be stopped until it accelerates in a proper direction
    if (this.location.x + this.velocity.x < -0.085) {
      this.location.set(-0.085, 0.13);
      this.velocity.set(0, 0);
      sumUserForceSlider.mult(0);
    } else if (this.location.x + this.velocity.x > 0.085) {
      this.location.set(0.085, 0.13);
      this.velocity.set(0, 0);
      sumUserForceSlider.mult(0);
    } else {
      // the velocity must be bounded
      velocity.add(acceleration);
      location.add(velocity);
    } 
    acceleration.mult(0);
  }
  
  void applyForce(PVector force) {
    acceleration.add(force);
  }
  
  PShape display() {
    float x = this.location.x;
    float x1 = pixelsPerMeter * (x - 0.01);
    float y1 = pixelsPerMeter * (0.083 + rEE);
    float h = pixelsPerMeter * 0.014;
    float w = pixelsPerMeter * 0.02;
  
    return createShape(RECT, x1, deviceOrigin.y + y1, w, h);
  }
  
  float getX() {
    return this.location.x;
  }
  
  PVector getVelocity() {
    return this.velocity;
  }
}

PShape create_wall(float x1, float y1, float x2, float y2){
  x1 = pixelsPerMeter * x1;
  y1 = pixelsPerMeter * y1;
  x2 = pixelsPerMeter * x2;
  y2 = pixelsPerMeter * y2;
  
  return createShape(LINE, deviceOrigin.x + x1, deviceOrigin.y + y1, deviceOrigin.x + x2, deviceOrigin.y+y2);
}


void update_animation(float th1, float th2, float xE, float yE){
  background(255);
  
  float lAni = pixelsPerMeter * l;
  float LAni = pixelsPerMeter * L;
  
  xE = pixelsPerMeter * xE;
  yE = pixelsPerMeter * yE;
  
  th1 = 3.14 - th1;
  th2 = 3.14 - th2;
  
  pGraph.setVertex(1, deviceOrigin.x + lAni*cos(th1), deviceOrigin.y + lAni*sin(th1));
  pGraph.setVertex(3, deviceOrigin.x + lAni*cos(th2), deviceOrigin.y + lAni*sin(th2));
  pGraph.setVertex(2, deviceOrigin.x + xE, deviceOrigin.y + yE);
  
  PShape circle = createShape(ELLIPSE, worldPixelWidth/2, 0.09 * pixelsPerMeter, radius, radius);
  circle.noFill();
  circle.stroke(0);
  circle.strokeWeight(3);
  
  // sliderCursor = s.display();
  sliderCursor = create_slider(s.getX());
  sliderCursor.setStroke(color(0));
  sliderCursor.setFill(color(255));

  
  shape(pGraph);
  shape(joint);
  shape(wall);
  shape(line2);
  shape(circle);
  shape(sliderCursor);
  
  // ellipse(worldPixelWidth/2, worldPixelHeight/2, 250, 250);
  
  
  translate(xE, yE);
  shape(endEffector);
}

PShape create_slider(float x) {
  float x1 = pixelsPerMeter * (x - 0.01);
  float y1 = pixelsPerMeter * (0.125 + rEE);
  float h = pixelsPerMeter * 0.014;
  float w = pixelsPerMeter * 0.02;
  
  return createShape(RECT,deviceOrigin.x + x1, deviceOrigin.y + y1, w, h);
}

PVector pos_to_grid(PVector position){
   float posX = position.x/0.01;
   float posY = position.y/0.0075;
   
   PVector grid = new PVector(posX+10, posY);
   
   return grid;
}

PVector grid_to_force(PVector grid){
  //PVector force = new PVector(grid.x - 10, grid.y - 10);

  //float gauss_x = 1/(pow(2*PI, 1/2))*exp(-pow(grid.x - 10, 2)/4);
  //float gauss_y = 1/(pow(2*PI, 1/2))*exp(-pow(grid.y - 10, 2)/4);

  float gauss_x = (pow(2*PI, 1/2))*exp(-pow(grid.x - 10, 2)/4);
  float gauss_y = (pow(2*PI, 1/2))*exp(-pow(grid.y - 10, 2)/4);

  PVector force = new PVector(1 - gauss_x, 1 - gauss_y);
  
  if (grid.x < 10){
    force.x = - force.x;
  }
  
  if (grid.y < 10){
    force.y = - force.y;
  }
  
  return force;
}

PVector device_to_graphics(PVector deviceFrame){
  return deviceFrame.set(-deviceFrame.x, deviceFrame.y);
}


PVector graphics_to_device(PVector graphicsFrame){
  return graphicsFrame.set(-graphicsFrame.x, graphicsFrame.y);
}

void oscEvent(OscMessage theOscMessage) {
  if(theOscMessage.checkAddrPattern("/position")==true) {
    float pos = float(theOscMessage.get(0).stringValue());
    sliderPositions.append(pos);
    if (sliderPositions.size() > 2) {
      float speed = (sliderPositions.get(sliderPositions.size() - 1) - sliderPositions.get(sliderPositions.size() - 2)) / 0.001; // v = d(p0p1)/dt en m/s
      sliderSpeeds.append(speed);
      print(
    }
   }
}

void keyPressed() {
  if (start == false) {
    start = true;
    OscMessage myMessage = new OscMessage("/sliderFocus");
    myMessage.add(0);
    oscP52.send(myMessage, myRemoteLocation);
  }
  else {
    start = false;
  }
  
}



/* end helper functions section ****************************************************************************************/




 
