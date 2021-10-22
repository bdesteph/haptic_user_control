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
int radius = 250;
PVector penX = new PVector(0, 0);
PVector penY = new PVector(0, 0);

/* virtual wall parameter  */
float             kWall                               = 450;
float             kLine1                               = 50;
float             kLine2                               = 100;
PVector           fWall                               = new PVector(0, 0);
PVector           fPos                           = new PVector(0, 0);
PVector           fLines                               = new PVector(0, 0);
PVector           penWall                             = new PVector(0, 0);
PVector           penLine1                             = new PVector(0, 0);
PVector           penLine2                            = new PVector(0, 0);
PVector           posWall                             = new PVector(0.01, 0.10);

PVector           posLine1                               = new PVector(0.01, 0.05);
PVector           posLine2                               = new PVector(0.01, 0.075);


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

/* graphical elements */
PShape pGraph, joint, endEffector;
PShape wall, line1, line2, circle;
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
  haplyBoard          = new Board(this, "/dev/cu.usbmodem14201", 0);
  widgetOne           = new Device(widgetOneID, haplyBoard);
  pantograph          = new Pantograph();
  
  widgetOne.set_mechanism(pantograph);
  
  widgetOne.add_actuator(1, CCW, 2);
  widgetOne.add_actuator(2, CW, 1);
 
  widgetOne.add_encoder(1, CCW, 241, 10752, 2);
  widgetOne.add_encoder(2, CW, -61, 10752, 1);
  
  widgetOne.device_set_parameters();
  
  
  /* visual elements setup */
  background(0);
  deviceOrigin.add(worldPixelWidth/2, 0);
  /* create pantagraph graphics */
  create_pantagraph();
  
  /* create wall graphics */
  
  
  /*wall = create_wall(posWall.x-0.2, posWall.y+rEE, posWall.x+0.2, posWall.y+rEE);
  wall.setStroke(color(0));
  
  line1 = create_wall(posLine1.x-0.2, posLine1.y+rEE, posLine1.x+0.2, posLine1.y+rEE);
  line1.setStroke(color(128));
  
  line2 = create_wall(posLine2.x-0.2, posLine2.y+rEE, posLine2.x+0.2, posLine2.y+rEE);
  line2.setStroke(color(128));*/
  
  /* setup framerate speed */
  frameRate(baseFrameRate);
  
  
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
      fLines.set(0, 0);
      
      penWall.set(0, (posWall.y - (posEE.y + rEE)));
      penLine1.set(0, (posLine1.y - posEE.y));
      
      penX.set(4000 * (posEE.x + rEE) - worldPixelWidth/2, 0);
      penY.set(0, 4000 * (posEE.y + rEE) - worldPixelHeight/2);
      
      //fY = fY.add(posY).mult(20);
      //fY = fY.add(0, 2);
      
      print("penX: ", penX.x, " penY: ", penY.y, " ");
      
      if(penLine1.y < 0){
        fLines = fLines.add(penLine1.mult(-50));
      }
      
      if(penWall.y < 0){
        //fWall = fWall.add(penWall.mult(-kWall));  
        fWall = fWall.add(penWall.mult(-kWall));  
      }
      
      if (start == true) {
        
      }
      
      fPos = (grid_to_force(pos_to_grid(posEE))).mult(-4);
 
      //fEE = (fWall.copy()).mult(-1);
      fEE = fPos.copy();
      fEE.set(graphics_to_device(fEE));
      /* end haptic wall force calculation */
    }
    
    //print(grid_to_force(pos_to_grid(posEE)));
    //print(pos_to_grid(posEE));
    
    //print(fEE.mag(), " ");
    //print((pow(2*PI, 1/2))*exp(-pow(10 - 10, 2)/4), " ");
    
    //fEE.set(0, 0);
    
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
  
  PShape circle = createShape(ELLIPSE, worldPixelWidth/2, worldPixelHeight/2, radius, radius);
  circle.noFill();
  circle.stroke(0);
  circle.strokeWeight(3);
  
  shape(pGraph);
  shape(joint);
  /*shape(wall);
  shape(line1);
  shape(line2);*/
  shape(circle);
  
  // ellipse(worldPixelWidth/2, worldPixelHeight/2, 250, 250);
  
  
  translate(xE, yE);
  shape(endEffector);
}

PVector pos_to_grid(PVector position){
   float posX = position.x/0.01;
   float posY = position.y/0.0075;
   
   PVector grid = new PVector(posX+10, posY);
   
   // print("X: ", posX, " Y: ", posY, " ");
   
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

void keyPressed() {
  if (start == false) {
    start = true;
  }
  else {
    start = false;
  }
  
}




/* end helper functions section ****************************************************************************************/




 
