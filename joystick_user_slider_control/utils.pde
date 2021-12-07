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

PVector sliderDeplacementForce = new PVector(0, 0);

PVector vibratoryForce = new PVector(0, 0);
float vSinTheta = 0;

FloatList sliderPositions = new FloatList();
FloatList sliderSpeeds = new FloatList();
FloatList sliderAccelerations = new FloatList();

FloatList sinusoidAccelerations = new FloatList();

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
float lastPosX;
// true means a deplacement to the right, false to the left
boolean cursorDirection;

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


class Slider {
  PVector location;
  PVector velocity;
  PVector acceleration;
  // to be able to calculate the maximum velocity of the slider, we must look into values without any user force order
  PVector accelerationWithoutUser;
  PVector velocityWithoutUser;
  
  float max_velocity = 0;
  
  Slider(float x, float y) {
    location = new PVector(x, y);
    velocity = new PVector(0, 0);
    velocityWithoutUser = new PVector(0, 0);
    acceleration = new PVector(0, 0); 
    accelerationWithoutUser = new PVector(0, 0);
    
  }
  /* Update Slider's velocity and location */
  void update() {
    // if the slider must go off the limits, the location stays the same and velocity needs to be stopped until it accelerates in a proper direction
    if (this.location.x + this.velocity.x < -0.085) {
      this.location.set(-0.085, 0.13);
      this.velocity.set(0, 0);
      this.velocityWithoutUser.set(0, 0);
      /*
      velocity.add(acceleration);
      location.add(velocity);
      print("LIMIT REACHED ");
      */
      sumUserForceSlider.mult(0);
    } else if (this.location.x + this.velocity.x > 0.085) {
      this.location.set(0.085, 0.13);
      this.velocity.set(0, 0);
      this.velocityWithoutUser.set(0, 0);
      /*
      velocity.add(acceleration);
      location.add(velocity);
      print("LIMIT REACHED ");
      */
      sumUserForceSlider.mult(0);
    } else {
      velocityWithoutUser.add(accelerationWithoutUser); 
      velocity.add(acceleration);
      location.add(velocity);
      
      if (abs(this.velocityWithoutUser.x) > abs(max_velocity)) {
        max_velocity = abs(this.velocityWithoutUser.x);
      }
    } 
    acceleration.mult(0);
    accelerationWithoutUser.mult(0);
  }
  
  void applyForce(PVector force, PVector userForce) {
    accelerationWithoutUser.add(force);
    acceleration.add(force.add(userForce));
  }
  
  void applyForce(PVector force) {
    PVector uf = new PVector(0, 0);
    this.applyForce(force, uf);
  }
  
  void setLocation(float x) {
    this.location.x = x;
  }
  
  PShape display() {
    float x1 = pixelsPerMeter * (this.location.x - 0.01);
    float y1 = pixelsPerMeter * (0.125 + rEE);
    float h = pixelsPerMeter * 0.014;
    float w = pixelsPerMeter * 0.02;
  
    return createShape(RECT,deviceOrigin.x + x1, deviceOrigin.y + y1, w, h);
  }
  
  float getX() {
    return this.location.x;
  }
  
  float getMaxVelocity() {
    return this.max_velocity;
  }
  
  float getVelocityX() {
    return this.velocity.x;
  }
  
  float getVelocityWithoutUser() {
    return this.velocityWithoutUser.x;
  }
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
  
  sliderCursor = s.display();

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

/* helper functions section, place helper functions here ***************************************************************/

float sliderVelocityToForce(float velocity) {
  float force = velocity * 10000;
  
  return force;
}

// This function takes a value from a [-0.085, 0.085] interval and convert it to a [0, 1] value, if absolute is true
// the two sides intervals (0 to 0.085 and -0.085 to 0 corresponds to 2 0 to 1 intervals
float scaleValue0to1 (float value, boolean absolute) {
  if (absolute) {
    value = abs(value);
    
    value /= 0.085;
  } 
  else {
    value += 0.085;
    value /= 0.17;
    
    if (value > 1) {
      value = 1;
    }
    if (value < 0) {
      value = 0;
    }
  }
  
  return value;
}

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

PShape create_wall(float x1, float y1, float x2, float y2){
  x1 = pixelsPerMeter * x1;
  y1 = pixelsPerMeter * y1;
  x2 = pixelsPerMeter * x2;
  y2 = pixelsPerMeter * y2;
  
  return createShape(LINE, deviceOrigin.x + x1, deviceOrigin.y + y1, deviceOrigin.x + x2, deviceOrigin.y+y2);
}

PVector device_to_graphics(PVector deviceFrame){
  return deviceFrame.set(-deviceFrame.x, deviceFrame.y);
}

PVector graphics_to_device(PVector graphicsFrame){
  return graphicsFrame.set(-graphicsFrame.x, graphicsFrame.y);
}

/* end helper functions section ****************************************************************************************/
