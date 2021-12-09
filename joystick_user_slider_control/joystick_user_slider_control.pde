/* setup section *******************************************************************************************************/
void setup(){
  /* put setup code here, run once: */
  
  /* screen size definition */
  size(800, 650);
  
  /* GUI setup */

  // DEFINE THE SLIDER HERE
  
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
  
  /* create walls and lines graphics */
  wall = create_wall(posWall.x-0.2, posWall.y+rEE, posWall.x+0.2, posWall.y+rEE);
  wall.setStroke(color(0));

  line2 = create_wall(posLine.x-0.2, posLine.y-rEE, posLine.x+0.2, posLine.y-rEE);
  line2.setStroke(color(128));
  
  /* create the scrollbar */
  
  hs1 = new HScrollbar(0, 0.14 * pixelsPerMeter, width, 16, 16);
  
  /* setup framerate speed */
  frameRate(baseFrameRate);
  
  /* create the Slider object */
  s = new PhysicalSlider(-0.085, 0.115);
  
  // maxVelocityAttractionRatio = 1000;
  
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
      
      /* all forces must be reset to 0 */
      fWall.set(0, 0);
      posY.set(0, posEE.y);
      fPos.set(0, 0);
      fLine.set(0, 0);
      vibratoryForce.set(0, 0);
      forceSlider.set(0, 0);
      sliderDeplacementForce.set(0, 0);
      userForceSlider.set(0, 0);
      
      // TO CHANGE: the maximum magnetic force needs to be automatically decided depending on the slider's trajectory
      float sliderMaxVelocity = s.getMaxVelocity();
      // print(sliderMaxVelocity, " ");
      // print("Without: ", s.getVelocityWithoutUser(), " with: ", s.getVelocityX());
      // magneticForce.set(0.000000168, 0); // maxVelocity = 1.681e-4 this is set to 5e-7
      maxVelocityAttractionRatio = 1000 * hs1.getPos();
      magneticForce.set(sliderMaxVelocity / maxVelocityAttractionRatio, 0);
      
      /* sinusoid force simulation part */ 
      
      accSinusoid = (-pow(0.001, 2) * sin(sinTheta - PI/2) * 0.085);
      
      /* sinusoidal movement */
      posSinusoid = sin(sinTheta - PI/2) * 0.085;
      
      /* saw tooth movement with a 3 seconds period */
      // posSinusoid = - 0.085 + (sinTheta % 3) * (0.170/3);
      
      
      
      /* triangle movement */
      float period = 3; // this var is actually period/2
      // posSinusoid = - 0.085 + 2 * abs((sinTheta/period) - int((sinTheta/period) + 0.5)) * 0.170;
      
      // print(posSinusoid, " ");
      
      vibratoryForceSinusoid = sin(vSinTheta) * 1.25; // the vibratory force feedback

      // accSinusoid = sinSwitch * (sin(sinTheta) * 0.000000675); // 0.0000000135 parcourt bien tout en x avec sinTheta += 0.0005
      // 0.000000027 parcourt bien tout en x avec sinTheta += 0.001
      // 0.000000675 parcourt bien tout en x avec sinTheta += 0.005
      
      if (start == true) {
        /* These two first ifs ensures that we'll have the first 2 positions to calculate the first acceleration we need to start */
        if (firstElement) {
          OscMessage myMessage = new OscMessage("/getPosition");
          myMessage.add(posSinusoid);
          oscP52.send(myMessage, myRemoteLocation);
          
          if (sliderPositions.size() > 0) {
            sinTheta += 0.001;
            vSinTheta += 0.2;
            sinusoidAccelerations.append(accSinusoid);
            
            forceSlider.add(sinusoidAccelerations.get(positionCounter), 0);
            firstElement = false;
            secondElement = true;
          }
        }
        if (secondElement) {
          OscMessage myMessage = new OscMessage("/getPosition");
          myMessage.add(posSinusoid);
          oscP52.send(myMessage, myRemoteLocation);
          
          last_timer = millis();
          
          lastPosX = posEE.x;
          
          if (sliderPositions.size() > 1) {
            sinTheta += 0.001;
            vSinTheta += 0.2;
            sinusoidAccelerations.append(accSinusoid);
            
            forceSlider.add(sinusoidAccelerations.get(positionCounter), 0);
            secondElement = false;
          }
        }
        /* If we are 2 positions ahead, we can calculate the current acceleration so we can continue */
        if (sliderPositions.size() >= positionCounter+2) {
          
          sinusoidAccelerations.append(accSinusoid);
          
          if (sinusoidAccelerations.get(positionCounter) != sliderAccelerations.get(positionCounter)) {
            // print(sliderAccelerations.get(positionCounter) - sinusoidAccelerations.get(positionCounter), " ");
            print("N: ", sinusoidAccelerations.get(positionCounter), " M: ", sliderAccelerations.get(positionCounter), " ");
          } else {
            print("YESSS ");
          }
          
        // if (sliderPositions.size() >= 2) {
          if (millis() - last_timer > 1) {
            // print(millis() - last_timer, " ");
          } 
          
          last_timer = millis();
          
          /* detects the cursor's deplacement direction */
          float cursorMovement = posEE.x - lastPosX;
          lastPosX = posEE.x;
           
          if (cursorMovement > 0.000001) {
            cursorDirection = true;
          }
          if (cursorMovement < -0.000001) {
            cursorDirection = false;
          }

          OscMessage myMessage = new OscMessage("/getPosition");
          myMessage.add(posSinusoid);
          oscP52.send(myMessage, myRemoteLocation);
          
          // sinusoidAccelerations.append(accSinusoid);
          
          // forceSlider.add(sinusoidAccelerations.get(positionCounter), 0);
          forceSlider.add(sinusoidAccelerations.get(positionCounter), 0);
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
  
          /* if the cursor is out the rest zone, then the user is expressing a force */
          if (posEE.x < -0.01 || posEE.x > 0.01) {
            /*
            userForceSlider = forceSlider.copy();
            userForceSlider.add(forceSlider.copy().mult(cursorPercentage));
            forceSlider = userForceSlider;
            */
            // forceSlider.add(userForceSlider);
            sumUserForceSlider.add(userForceSlider);
          } 
          // TO CHANGE: it mustn't cancel the user force when the cursor is passing by the rest zone
          else {
            // s.applyForce(sumUserForceSlider.mult(-1));
            sumUserForceSlider.mult(0);
          }
          
          /* FORCE FEEDBACK CALCULATION */
          
          /* haptic wall force calculation */
          penWall.set(0, (posWall.y - (posEE.y + rEE)));
          
          if(penWall.y < 0){
            fWall = fWall.add(penWall.mult(-kWall));  
          }
          /* haptic line force calculation */
          penLine.set(0, (posLine.y - (posEE.y - rEE)));
          
          if(penLine.y > 0){
            // fLine = fLine.add(penLine.mult(-kWall));  
          }
          
          
          if (posEE.x < -0.01 || posEE.x > 0.01) {
            /* vibratory force return calculation */
            vibratoryForce = vibratoryForce.add(vibratoryForceSinusoid, 0);
            
            /* slider's movement force return */
            float sliderVelocity = s.getVelocityX();
            
            float preForce = sliderVelocityToForce(sliderVelocity);

            // then the direction is to the right
            if (cursorDirection) {
              if (preForce < 0) {
                // sliderDeplacementForce = sliderDeplacementForce.add(preForce, 0);
                // print("Force ");
              }
            } else {
              if (preForce > 0) {
                // sliderDeplacementForce = sliderDeplacementForce.add(preForce, 0);
                // print("Force ");
              }
            }
          }
          
          // is the cursor in the rest zone? if no, then the slider force will apply as well on the cursor
          if (posEE.x < -0.01 || posEE.x > 0.01) {
            fEE = (fWall.copy().add(fLine).add(vibratoryForce).add(sliderDeplacementForce)).mult(-1);
          }
          else {
            fEE = (fWall.copy().add(fLine)).mult(-1);
          }
          
          sinTheta += 0.001;
          
          vSinTheta += cursorPercentage * 0.8;

          s.applyForce(forceSlider, userForceSlider);
          s.update();
          
          // print(" acceleration: ", sliderAccelerations.get(positionCounter)); 
          
          // We send the "real" value to our OSC server so it can then be used in our parametric system
          s.getX();
          OscMessage sendEffectivePosition = new OscMessage("/effectivePosition");
          sendEffectivePosition.add(scaleValue0to1(s.getX(), false));
          oscP52.send(sendEffectivePosition, myRemoteLocation);
        }
      }

      fEE.set(graphics_to_device(fEE));
      /* end haptic wall force calculation */
    }

    torques.set(widgetOne.set_device_torques(fEE.array()));
    widgetOne.device_write_torques();
  
    renderingForce = false;
  }
}
/* end simulation section **********************************************************************************************/

/* managing events *****************************/

void oscEvent(OscMessage theOscMessage) {
  /* When we receive a position, we first stock it */
  if(theOscMessage.checkAddrPattern("/position")==true) {
    float pos = float(theOscMessage.get(0).stringValue());
    sliderPositions.append(pos);
    /* We then calculate the speed it took to get to this position if we have the precedent position */
    if (sliderPositions.size() == 1) {
      sliderSpeeds.append(0);
    }
    if (sliderPositions.size() > 1) {
      float speed = (sliderPositions.get(sliderPositions.size() - 1) - sliderPositions.get(sliderPositions.size() - 2)) / 1; // v = d(p0p1)/dt en m/s
      sliderSpeeds.append(speed);
      /* And finally we calculate the acceleration it took to get there */
      if (sliderPositions.size() == 1) {
        sliderAccelerations.append(0);
      }
      if (sliderPositions.size() == 2) {
        sliderAccelerations.append(speed);
      }
      if (sliderPositions.size() > 2) {
        float acceleration = (sliderSpeeds.get(sliderSpeeds.size() - 1) - sliderSpeeds.get(sliderSpeeds.size() - 2)) / 1;
        sliderAccelerations.append(acceleration);
      }
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
