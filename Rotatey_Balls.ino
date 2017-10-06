#if CONFIG_FREERTOS_UNICORE
#define ARDUINO_RUNNING_CORE 0
#else
#define ARDUINO_RUNNING_CORE 1
#endif

#include "Wire.h"
#include "mpu6050rotatey.h" /* Source http://tocknlab.hatenablog.com/entry/2017/03/11/182703 */

#include "SSD1306.h" // alias for `#include "SSD1306Wire.h"`
#include "OLEDDisplayUi.h"

#define DEBUG false

// fps counter
unsigned int fpsall = 30;

int sdaPin = 21;
int sclPin = 22;

const double halfC = M_PI / 180;

// Overall scale and perspective distance
uint8_t sZ = 4, scale = 16, scaleMax = 16;
// screen center coordinates (calculated from screen dimensions)
uint8_t centerX = 64;
uint8_t centerY = 32;

typedef struct {
    double x;
    double y;
    double z;
} Coord3DSet;

typedef struct {
    double x;
    double y;
} Coord2DSet;

typedef struct {
    uint16_t id1;
    uint16_t id2;
} Lines;  


/* https://codepen.io/ge1doot/pen/grWrLe */

static Coord3DSet CubePoints3DArray[21] = {
  {  1,  1,  1 },
  {  1,  1, -1 },
  {  1, -1,  1 },
  {  1, -1, -1 },
  { -1,  1,  1 },
  { -1,  1, -1 },
  { -1, -1,  1 },
  { -1, -1, -1 },

  {  1,  1,  0 },
  {  1,  0,  1 },
  {  0,  1,  1 },

  {  -1,  1,  0 },
  {  -1,  0,  1 },
  {  0,  -1,  1 },

  {  1,  -1,  0 },
  {  1,  0,  -1 },
  {  0,  1,  -1 },

  {  -1,  -1,  0 },
  {  -1,  0,  -1 },
  {  0,  -1,  -1 },

  {0, 0, 0}
  
};

static Coord3DSet CubePoints2DArray[21] = {
  { 0,0 },
  { 0,0 },
  { 0,0 },
  { 0,0 },
  { 0,0 },
  { 0,0 },
  { 0,0 },
  { 0,0 },

  { 0,0 },
  { 0,0 },
  { 0,0 },
  { 0,0 },
  { 0,0 },
  { 0,0 },
  { 0,0 },
  { 0,0 },
  { 0,0 },
  
  { 0,0 },
  { 0,0 },
  { 0,0 },

  { 0,0 }
};

static Lines LinesArray[12] = {
  { 0, 1 },
  { 0, 2 },
  { 0, 4 },
  { 1, 3 },
  { 1, 5 },
  { 2, 3 },
  { 2, 6 },
  { 3, 7 },
  { 4, 5 },
  { 4, 6 },
  { 5, 7 },
  { 6, 7 }
/*
  { 1, 4 },
  { 2, 3 },
  { 1, 6 },
  { 2, 5 },
  { 2, 8 },
  { 6, 4 },
  { 4, 7 },
  { 3, 8 },
  { 1, 7 },
  { 3, 5 },
  { 5, 8 },
  { 7, 6 }
 */
  
};

// used for sorting points by depth
uint16_t zsortedpoints[21] = {0,1,2,3,4,5,6,7,8,9,10,11,12,13,14,15,16,17,18,19,20};

uint16_t totalpoints = sizeof(CubePoints3DArray) / sizeof(CubePoints3DArray[0]);
uint16_t totallines = sizeof(LinesArray) / sizeof(LinesArray[0]);


/*
const int maxScale = 64;
const int redZone = 5;

const int sampleSize = 1024; // initially 1024
const int sampleWindow = 20; // Sample window width in mS (50 mS = 20Hz)
unsigned int sample;
uint16_t a0value;
*/





SSD1306 display(0x3c, sdaPin, sclPin);
OLEDDisplayUi ui ( &display );


void setup() {

  Serial.begin(115200);

  mpu_init(sdaPin, sclPin);// sda, scl
  mpu_calibrate();

  // store initial position
  calcRotation(); // read from MPU
  lastAngleX = angleX;
  lastAngleY = angleY;
  lastAngleZ = angleZ;

  Serial.println("Starting screen");
  display.init();
  //display.flipScreenVertically(); // do this if you need to mess things up
  display.clear();   // clears the screen and buffer
  display.display();
  display.setColor(WHITE);

  //xTaskCreatePinnedToCore(loop1, "loop1", 4096, NULL, 1, NULL, ARDUINO_RUNNING_CORE);
  
}

void loop() {

  calcRotation(); // read from MPU

#if DEBUG == true
  Serial.print("angle{X,Y,Z}= ");
  Serial.print(angleX);
  Serial.print("\t,");
  Serial.print(angleY);
  Serial.print("\t,");
  Serial.print(angleZ);
  Serial.print("\t || acc_{x,y,z}= ");
  Serial.print(acc_x);
  Serial.print("\t,");
  Serial.print(acc_y);
  Serial.print("\t,");
  Serial.println(acc_z);
#endif  

  display.clear();
  cubeloop();
  //loop1();
  fps(1);
  msOverlay();
  display.display();
  
  // store last position
  lastAngleX = angleX;
  lastAngleY = angleY;
  lastAngleZ = angleZ;
 
}

/*
void loop1() {
  unsigned long startMillis= millis();  // Start of sample window
  unsigned int peakToPeak = 0;   // peak-to-peak level
  unsigned int signalMax = 0;
  unsigned int signalMin = sampleSize;

  while (millis() - startMillis < sampleWindow) {
      sample = analogRead(35); 
      if (sample < sampleSize){   // toss out spurious readings
         if (sample > signalMax) {
            signalMax = sample;  // save just the max levels
         } else if (sample < signalMin) {
            signalMin = sample;  // save just the min levels
         }
      }
      yield();
   }
   peakToPeak = signalMax - signalMin;

   // map 1v p-p level to the max scale of the display
   int displayPeak = map(peakToPeak, 0, sampleSize-1, 0, maxScale);

   display.drawLine(1, 1, 1, displayPeak);

   scale = 16 + (displayPeak/20);
}*/


void cubeloop() {

  float diffAngleX, diffAngleY, diffAngleZ;

  diffAngleX = lastAngleX - angleX;
  diffAngleY = lastAngleY - angleY;
  diffAngleZ = lastAngleZ - angleZ;

  vectorRotateXYZ((double)(diffAngleY+0.1)*halfC, 1); // X
  vectorRotateXYZ((double)(diffAngleX+0.1)*halfC, 2); // Y
  vectorRotateXYZ((double)diffAngleZ*halfC, 3); // Z

  zSortPoints();
  meshPlot();
  spherePlot();

}


void vectorRotateXYZ(double angle, int axe) {
  int8_t m1; // coords polarity
  uint8_t i1, i2; // coords index
  double t1, t2;
  uint16_t i;
  for( i=0; i<totalpoints; i++ ) {
    switch(axe) {
      case 1: // X
        m1 = -1;
        t1 = CubePoints3DArray[i].y;
        t2 = CubePoints3DArray[i].z;
        CubePoints3DArray[i].y = t1*cos(angle)+(m1*t2)*sin(angle);
        CubePoints3DArray[i].z = (-m1*t1)*sin(angle)+t2*cos(angle);
      break;
      case 2: // Y
        m1 = 1;
        t1 = CubePoints3DArray[i].x;
        t2 = CubePoints3DArray[i].z;
        CubePoints3DArray[i].x = t1*cos(angle)+(m1*t2)*sin(angle);
        CubePoints3DArray[i].z = (-m1*t1)*sin(angle)+t2*cos(angle);
      break;
      case 3: // Z
        m1 = 1;
        t1 = CubePoints3DArray[i].x;
        t2 = CubePoints3DArray[i].y;
        CubePoints3DArray[i].x = t1*cos(angle)+(m1*t2)*sin(angle);
        CubePoints3DArray[i].y = (-m1*t1)*sin(angle)+t2*cos(angle);
      break;
    }
  }
}

/* sort xyz by z depth */
void zSortPoints() {
  bool swapped;
  uint16_t temp;
  float radius, nextradius;
  do {
    swapped = false;
    for(uint16_t i=0; i!=totalpoints-1; i++ ) {
      radius     = (-CubePoints3DArray[zsortedpoints[i]].z+3)*2;
      nextradius = (-CubePoints3DArray[zsortedpoints[i+1]].z+3)*2;
      if (radius > nextradius) {
        temp = zsortedpoints[i];
        zsortedpoints[i] = zsortedpoints[i + 1];
        zsortedpoints[i + 1] = temp;
        swapped = true;
      }
    }
  } while (swapped);
}


/* draw scaled spheres from background to foreground */
void spherePlot() {
  uint16_t i;
  int radius, halfradius;
  int transid;
  for( i=0; i<totalpoints; i++ ) {
    transid = zsortedpoints[i];
    CubePoints2DArray[transid].x = centerX + scale/(1+CubePoints3DArray[transid].z/sZ)*CubePoints3DArray[transid].x; 
    CubePoints2DArray[transid].y = centerY + scale/(1+CubePoints3DArray[transid].z/sZ)*CubePoints3DArray[transid].y;
    radius = (-CubePoints3DArray[transid].z+3)*2.5;
    halfradius = radius / 2;
    display.setColor(BLACK);  
    //display.fillCircle(CubePoints2DArray[transid].x, CubePoints2DArray[transid].y, radius-1);
    display.fillCircle(CubePoints2DArray[transid].x, CubePoints2DArray[transid].y, radius+1);
    display.setColor(WHITE);
    display.drawCircle(CubePoints2DArray[transid].x, CubePoints2DArray[transid].y, radius);
    display.fillCircle(CubePoints2DArray[transid].x+halfradius-1, CubePoints2DArray[transid].y+halfradius-1, halfradius);
  }
}

/* draw lines between given pairs of points */
void meshPlot() {
  uint16_t i;
  uint16_t id1, id2;
  for( i=0; i<totallines; i++ ) {
    id1 = LinesArray[i].id1;
    id2 = LinesArray[i].id2;
    display.drawLine(CubePoints2DArray[id1].x, CubePoints2DArray[id1].y, CubePoints2DArray[id2].x, CubePoints2DArray[id2].y);
  }
}



static inline void fps(const int seconds){
  // Create static variables so that the code and variables can
  // all be declared inside a function
  static unsigned long lastMillis;
  static unsigned long frameCount;
  static unsigned int framesPerSecond;
  
  // It is best if we declare millis() only once
  unsigned long now = millis();
  frameCount ++;
  if (now - lastMillis >= seconds * 1000) {
    framesPerSecond = frameCount / seconds;
    //Serial.println(framesPerSecond);
    fpsall = framesPerSecond;
    frameCount = 0;
    lastMillis = now;
  }
}

void msOverlay() {
  //display.setTextAlignment(TEXT_ALIGN_RIGHT);
  display.setFont(ArialMT_Plain_10);
  display.drawString(0, 10, String(fpsall)+"fps");
}
