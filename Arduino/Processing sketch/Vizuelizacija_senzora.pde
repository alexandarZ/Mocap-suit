import processing.serial.*;
import processing.opengl.*;
import toxi.geom.*;
import toxi.processing.*;

ToxiclibsSupport gfx;

Serial port;                         
char[] sensorPacket = new char[23];  
int serialCount = 0;                 
int synced = 0;
int interval = 0;

float[] q1 = new float[4];
float[] q2 = new float[4];

Quaternion quat_a = new Quaternion(1, 0, 0, 0);
Quaternion quat_b = new Quaternion(1, 0, 0, 0);

float[] gravity = new float[3];
float[] euler   = new float[3];
float[] ypr     = new float[3];

int show_counter=0;

void setup() {
    
    size(500, 500, OPENGL);
    gfx = new ToxiclibsSupport(this);

    // setup lights and antialiasing
    lights();
    smooth();
  
    // display serial port list for debugging/clarity
    println(Serial.list());

    // get the first available port
    String portName = Serial.list()[0];

    // open the serial port
    port = new Serial(this, portName, 115200);
}

void draw() 
{  
    if (millis() - interval > 1000) {
        // resend single character to trigger DMP init/start
        // in case the MPU is halted/reset while applet is running
        port.write('r');
        interval = millis();
    }
    
    // black background
    background(0);
    
    //Camera 
    if(mousePressed) 
    {
      float fov = PI/3.0; 
      float cameraZ = (height/2.0) / tan(fov/2.0); 
      perspective(fov, float(width)/float(height), cameraZ/2.0, cameraZ*2.0); 
    } 
      else 
    {
      ortho(-width/2, width/2, -height/2, height/2);
    }
    
    /* DRAW FIRST BOX */
    
    pushMatrix();
    
    // translate everything to the middle of the viewport
    translate(width / 2 - 100, height / 2);

    float[] axis_1 = quat_a.toAxisAngle();
    rotate(axis_1[0], -axis_1[1], axis_1[3], axis_1[2]);
    
    // Draw box
    box(100);

    popMatrix();
    
    /* DRAW SECOND BOX */
    
    pushMatrix();
    
    // translate everything to the middle of the viewport
    translate(width / 2 + 100, height / 2);

    float[] axis_2 = quat_b.toAxisAngle();
    rotate(axis_2[0], -axis_2[1], axis_2[3], axis_2[2]);
    
    // Draw box
    box(100);
      
    popMatrix();
}

void serialEvent(Serial port) 
{
    interval = millis();
    
    while (port.available() > 0) {
        
        int ch = port.read();

        if (synced == 0 && ch != '$') {return;}   // initial synchronization - also used to resync/realign if needed
        
        synced = 1;
  
        if ((serialCount == 1 && ch != 1)
            || (serialCount == 21 && ch != '\r')
            || (serialCount == 22 && ch != '\n'))  {
            serialCount = 0;
            synced = 0;
            return;
        }

        if (serialCount > 0 || ch == '$') {
            sensorPacket[serialCount++] = (char)ch;
           
            if (serialCount == 23) {
                serialCount = 0; // restart packet byte position
                
                //print("Serial count: 23 Synced: 1");

                // get quaternion from data packet
                q1[0] = ((sensorPacket[2] << 8) | sensorPacket[3]) / 16384.0f;
                q1[1] = ((sensorPacket[4] << 8) | sensorPacket[5]) / 16384.0f;
                q1[2] = ((sensorPacket[6] << 8) | sensorPacket[7]) / 16384.0f;
                q1[3] = ((sensorPacket[8] << 8) | sensorPacket[9]) / 16384.0f;
                for (int i = 0; i < 4; i++) if (q1[i] >= 2) q1[i] = -4 + q1[i];
                
                q2[0] = ((sensorPacket[11] << 8) | sensorPacket[12]) / 16384.0f;
                q2[1] = ((sensorPacket[13] << 8) | sensorPacket[14]) / 16384.0f;
                q2[2] = ((sensorPacket[15] << 8) | sensorPacket[16]) / 16384.0f;
                q2[3] = ((sensorPacket[17] << 8) | sensorPacket[18]) / 16384.0f;
                for (int i = 0; i < 4; i++) if (q2[i] >= 2) q2[i] = -4 + q2[i];
                
                // set our toxilibs quaternion to new data
                quat_a.set(q1[0], q1[1], q1[2], q1[3]);
                quat_b.set(q2[0], q2[1], q2[2], q2[3]);
                
                // below calculations unnecessary for orientation only using toxilibs
                /*
                // calculate gravity vector
                gravity[0] = 2 * (q1[1]*q1[3] - q1[0]*q1[2]);
                gravity[1] = 2 * (q1[0]*q1[1] + q1[2]*q1[3]);
                gravity[2] = q1[0]*q1[0] - q1[1]*q1[1] - q1[2]*q1[2] + q1[3]*q1[3];
    
                // calculate Euler angles
                euler[0] = atan2(2*q1[1]*q1[2] - 2*q1[0]*q1[3], 2*q1[0]*q1[0] + 2*q1[1]*q1[1] - 1);
                euler[1] = -asin(2*q1[1]*q1[3] + 2*q1[0]*q1[2]);
                euler[2] = atan2(2*q1[2]*q1[3] - 2*q1[0]*q1[1], 2*q1[0]*q1[0] + 2*q1[3]*q1[3] - 1);
    
                // calculate yaw/pitch/roll angles
                ypr[0] = atan2(2*q1[1]*q1[2] - 2*q1[0]*q1[3], 2*q1[0]*q1[0] + 2*q1[1]*q1[1] - 1);
                ypr[1] = atan(gravity[0] / sqrt(gravity[1]*gravity[1] + gravity[2]*gravity[2]));
                ypr[2] = atan(gravity[1] / sqrt(gravity[0]*gravity[0] + gravity[2]*gravity[2]));
    
                // output various components for debugging
                //println("q:\t" + round(q1[0]*100.0f)/100.0f + "\t" + round(q1[1]*100.0f)/100.0f + "\t" + round(q1[2]*100.0f)/100.0f + "\t" + round(q1[3]*100.0f)/100.0f);
                //println("euler:\t" + euler[0]*180.0f/PI + "\t" + euler[1]*180.0f/PI + "\t" + euler[2]*180.0f/PI);
                //println("ypr:\t" + ypr[0]*180.0f/PI + "\t" + ypr[1]*180.0f/PI + "\t" + ypr[2]*180.0f/PI);
                */
            }
        }
    }
}