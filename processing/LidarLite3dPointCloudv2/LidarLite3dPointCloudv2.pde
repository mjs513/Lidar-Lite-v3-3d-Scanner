// LidarScanner.pde Processing sketch
// http://www.qcontinuum.org/lidar

// Load sketch into Processing, available from:
// https://processing.org/

// This sketch accepts XYZ coordinates from Arduino LIDAR scanner
// and displays them graphically as a 3D point cloud that you can
// pan, zoom, and rotate using keyboard.

import processing.serial.*;

int PrintOutput = 1;       // Output raw data to standard file name
int scanPointCount = 10000;
int readFile = 0;

class Point {
  PVector p;
  PVector v;
  float radius;

  Point( PVector _p , float _radius){
    p = _p;
    v = new PVector( random(-.01,.01), random(-.01,.01), random(-.01,.01) );
    radius = _radius;
  }
  void process(){
    //simulate();
    render();
  }
  void simulate(){
    p.x+=v.x;
    p.y+=v.y;
    p.z+=v.z;
    p.x%=1;
    p.y%=1;
    p.z%=1;
  } 
  void render(){
    if(radius <= 5){
      stroke(255, 0, 0);
    } else if(radius > 5 && radius <= 10) {
      stroke(255, 255,255);
    } else if(radius > 10 && radius <= 20) {
      stroke(192, 192,192);
    } else if(radius > 20 && radius <= 30) {
      stroke(204, 204,255);
    } else if(radius > 30 && radius <= 60) {
      stroke(255, 255,153);//
    } else if(radius > 60 && radius <= 90) {
      stroke(255, 204,153);
    } else if(radius > 90 && radius <= 150) {
      stroke(204, 255,255);
    } else if(radius > 150 && radius <= 310){
      stroke(255, 255, 204);
    } else if(radius > 310 && radius <= 500){
      stroke(153, 153, 255);
    } else {
      stroke(153, 51, 102);
      //stroke(0, 0, 0);
    }
    point(p.x * scale + xOffset, -p.z * scale + yOffset, -p.y * scale);
  }
}

Point[] points = new Point[scanPointCount];

PrintWriter output;

Serial serial;
int serialPortNumber = 0;
float angle = 6.5f;
float angleIncrement = 0;
float xOffset = 3.0;
float xOffsetIncrement = 0;
float yOffset = 152.0f;
float yOffsetIncrement = 0;
float scale = 2.6f;
float scaleIncrement = 0;
ArrayList<PVector> vectors;
int lastPointIndex = 0;
int lastPointCount = 0;
float dist;
int pointCount = 0;
int flagFini = 0;

void setup() {
  size(800, 600, P3D);
  colorMode(RGB, 255, 255, 255);
  noSmooth();
  vectors = new ArrayList<PVector>();

  // Create a new file in the sketch directory
  if(PrintOutput == 1)
    output = createWriter("scan.txt"); 
  
  if(readFile == 0){
    String[] serialPorts = Serial.list();
    String serialPort = serialPorts[serialPortNumber];
    println("Using serial port \"" + serialPort + "\"");
    println("To use a different serial port, change serialPortNumber:");
    printArray(serialPorts);
    serial = new Serial(this, serialPort, 115200);
  }  else {
     readPoints();
     for( int t=0; t < pointCount;t++){
      points[t].process();
      if(PrintOutput == 1){
         output.println(points[t].p.x+","+points[t].p.y+","+points[t].p.z+","+points[t].radius);
      }
    }
  }
}

void draw() {
  background(0);
  translate(width/2, height/2, -50);
  rotateY(angle);
    
  if(readFile == 0){
    if(flagFini ==0)
      getCloud();
  }
  
  if(flagFini == 1 || readFile == 1){
    for( int t=0; t < pointCount;t++){
      points[t].process();
      if(PrintOutput == 1){
          output.println(points[t].p.x+","+points[t].p.y+","+points[t].p.z+","+points[t].radius);
      }
    }
    if(PrintOutput == 1){
      PrintOutput = 0;
      output.flush();  // Writes the remaining data to the file
      output.close();  // Finishes the file
    }
    
  }
  angle += angleIncrement;
  xOffset += xOffsetIncrement;
  yOffset += yOffsetIncrement;
  scale += scaleIncrement;
}

void keyPressed() {
  if (key == 'q') {
    // zoom in
    scaleIncrement = 0.02f;
  } else if (key == 'z') {
    // zoom out
    scaleIncrement = -0.02f;
  } else if (key == 'a') {
    // move left
    xOffsetIncrement = -1f;
  } else if (key == 'd') {
    // move right
    xOffsetIncrement = 1f;
  } else if (key == 'w') {
    // move up
    yOffsetIncrement = -1f;
  } else if (key == 's') {
    // move down
    yOffsetIncrement = 1f;
  } else if (key =='x') {
    // erase all points
    vectors.clear();
  } else if (key == CODED) {
    if (keyCode == LEFT) {
      // rotate left
      angleIncrement = -0.015f;
    } else if (keyCode == RIGHT) {
      // rotate right
      angleIncrement = 0.015f;
    } 
  }
}

void keyReleased() {
  if (key == 'q') {
    scaleIncrement = 0f;
  } else if (key == 'z') {
    scaleIncrement = 0f;
  } else if (key == 'a') {
    xOffsetIncrement = 0f;
  } else if (key == 'd') {
    xOffsetIncrement = 0f;
  } else if (key == 'w') {
    yOffsetIncrement = 0f;
  } else if (key == 's') {
    yOffsetIncrement = 0f;
  } else if (key == CODED) {
    if (keyCode == LEFT) {
      angleIncrement = 0f;
    } else if (keyCode == RIGHT) {
      angleIncrement = 0f;
    }
  }
}

void getCloud(){
    String input = serial.readStringUntil(10);
    if (input != null) {
      String[] components = split(input, ' ');
      if (components.length == 4) {
        dist = float(components[3]);
        points[pointCount] = new Point( new PVector(float(components[0]), float(components[1]), float(components[2])), dist);
        vectors.add(new PVector(float(components[0]), float(components[1]), float(components[2])));
        pointCount = pointCount + 1;
      }
    }
    
    int size = vectors.size();
    for (int index = 0; index < size; index++) {
      PVector v = vectors.get(index);
      if (index == size - 1) {
        // draw red line to show recently added LIDAR scan point
        if (index == lastPointIndex) {
          lastPointCount++;
        } else {
          lastPointIndex = index;
          lastPointCount = 0;
        }
        if (lastPointCount < 10) {
          stroke(255, 0, 0);
          line(xOffset, yOffset, 0, v.x * scale + xOffset, -v.z * scale + yOffset, -v.y * scale);
        }
      }
      stroke(255,255,255);
      point(v.x * scale + xOffset, -v.z * scale + yOffset, -v.y * scale);
    }
    
    if(pointCount > scanPointCount-1){
      flagFini = 1;
      if(readFile == 0){
         serial.clear();
         serial.write("t");
      }
    }
  //}
}

void readPoints(){
  String[] lines = loadStrings("scan.txt"); 
  for (int i=0; i < scanPointCount-1; i++) {
      String[] components = split(lines[i], ',');
      if (components.length == 4) {
        dist = float(components[3]);
        points[pointCount] = new Point( new PVector(float(components[0]), float(components[1]), float(components[2])), dist);
        vectors.add(new PVector(float(components[0]), float(components[1]), float(components[2])));
        pointCount = pointCount + 1;
      }
  } //for
}
