//--------------------------------------------------------------------------
// Code to test basic Hapkit functionality (sensing and force output)
// 04.11.14
// Updated by Tania Morimoto 11.01.18
//--------------------------------------------------------------------------

// Includes
#include <math.h>
#define Mass_spring_damping

// Pin declares
int pwmPin = 5; // PWM output pin for motor 1
int dirPin = 8; // direction output pin for motor 1
int sensorPosPin = A2; // input pin for MR sensor
int fsrPin = A3; // input pin for FSR sensor

// Position tracking variables
int updatedPos = 0;     // keeps track of the latest updated value of the MR sensor reading
int rawPos = 0;         // current raw reading from MR sensor
int lastRawPos = 0;     // last raw reading from MR sensor
int lastLastRawPos = 0; // last last raw reading from MR sensor
int flipNumber = 0;     // keeps track of the number of flips over the 180deg mark
int tempOffset = 0;
int rawDiff = 0;
int lastRawDiff = 0;
int rawOffset = 0;
int lastRawOffset = 0;
const int flipThresh = 700;  // threshold to determine whether or not a flip over the 180 degree mark occurred
boolean flipped = false;
double OFFSET = 980;
double OFFSET_NEG = 15;

// Kinematics variables
double xh = 0;           // position of the handle [m]

// Force output variables
double force = 0;           // force at the handle
double Tp = 0;              // torque of the motor pulley
double duty = 0;            // duty cylce (between 0 and 255)
unsigned int output = 0;    // output command to the motor
//int flag=1; //compute velocity but the first time cannot count
double stiffness=90;
double damping=2;
double velocity=0;
//double newvelocity=0;
double oldvelocity=0;
//double oldtime=0;
//double newtime=0;
double oldposition=0;
//double newposition=0;
double  filteredVelocity = 0;
double  oldfilteredVelocity=0;
double acceleration=0;
double mass=1;

// --------------------------------------------------------------
// Setup function -- NO NEED TO EDIT
// --------------------------------------------------------------
void setup() 
{
  // Set up serial communication
  Serial.begin(9600);
  
  // Set PWM frequency 
  setPwmFrequency(pwmPin,1); 
  
  // Input pins
  pinMode(sensorPosPin, INPUT); // set MR sensor pin to be an input
  pinMode(fsrPin, INPUT);       // set FSR sensor pin to be an input

  // Output pins
  pinMode(pwmPin, OUTPUT);  // PWM pin for motor A
  pinMode(dirPin, OUTPUT);  // dir pin for motor A
  
  // Initialize motor 
  analogWrite(pwmPin, 0);     // set to not be spinning (0/255)
  digitalWrite(dirPin, LOW);  // set direction
  
  // Initialize position valiables
  lastLastRawPos = analogRead(sensorPosPin);
  lastRawPos = analogRead(sensorPosPin);
  flipNumber = 0;
}


// --------------------------------------------------------------
// Main Loop
// --------------------------------------------------------------
void loop()
{
  
  //*************************************************************
  //*** Section 1. Compute position in counts (do not change) ***  
  //*************************************************************
//newtime=millis();
//newposition=xh;
//newvelocity=velocity;
  // Get voltage output by MR sensor
  rawPos = analogRead(sensorPosPin);  //current raw position from MR sensor

  // Calculate differences between subsequent MR sensor readings
  rawDiff = rawPos - lastRawPos;          //difference btwn current raw position and last raw position
  lastRawDiff = rawPos - lastLastRawPos;  //difference btwn current raw position and last last raw position
  rawOffset = abs(rawDiff);
  lastRawOffset = abs(lastRawDiff);
  
  // Update position record-keeping vairables
  lastLastRawPos = lastRawPos;
  lastRawPos = rawPos;
  
  // Keep track of flips over 180 degrees
  if((lastRawOffset > flipThresh) && (!flipped)) { // enter this anytime the last offset is greater than the flip threshold AND it has not just flipped
    if(lastRawDiff > 0) {        // check to see which direction the drive wheel was turning
      flipNumber--;              // cw rotation 
    } else {                     // if(rawDiff < 0)
      flipNumber++;              // ccw rotation
    }
    flipped = true;            // set boolean so that the next time through the loop won't trigger a flip
  } else {                        // anytime no flip has occurred
    flipped = false;
  }
   updatedPos = rawPos + flipNumber*OFFSET; // need to update pos based on what most recent offset is 

 
  //*************************************************************
  //*** Section 2. Compute position in meters *******************
  //*************************************************************

  // ADD YOUR CODE HERE
  // Define kinematic parameters you may need
     //double rh = ?;   //[m]
  // Step B.1: print updatedPos via serial monitor
 //Serial.println(updatedPos);       // print as an ASCII-encoded decimal
//  Serial.print("\t");
 //Step B.6: 
 double pi=3.14;
 double ts = (0.0167*updatedPos-14.24)/180*pi; // Compute the angle of the sector pulley (ts) in degrees based on updatedPos
  // Step B.7: 
 double rh = 0.073;
 double xh = rh*ts;       // Compute the position of the handle (in meters) based on ts (in radians)

 velocity=(xh-oldposition)/(0.0001);
 filteredVelocity = 0.7*velocity+0.3*oldvelocity;
 oldposition=xh ;
 oldvelocity=velocity;
 acceleration=(filteredVelocity-oldfilteredVelocity);
 oldfilteredVelocity=filteredVelocity;
 
  // Step B.8: print xh via serial monitor
// Serial.println(xh, 5);
//  Serial.print("\t");
//double time = millis();
//Serial.print(time);
//Serial.print("\n");
//newtime=time;
//newposition=xh;
//if(flag==1){
//  velocity=(newposition-oldposition)/(newtime-oldtime);
//  oldposition=newposition;
//  oldtime=newtime;
//  }
//  else{
//    time = millis();
//    
//  }
//flag=1;


//Serial.print(newtime-oldtime);
//Serial.print("\t");
//Serial.print(velocity);
//Serial.print("\t");

  
  //*************************************************************
  //*** Section 3. Assign a motor output force in Newtons *******  
  //*************************************************************
 
  // ADD YOUR CODE HERE
//_________A___________________________________________________________  
  #ifdef virtual_wall
  {
//    Serial.print("environment is wall");
//    Serial.print("\t");
    if(xh>0.005) {
      force=-stiffness*(xh-0.005);
    }
    else{
      force=0;
    }     
  }

  #endif
//_________B___________________________________________________________  
#ifdef linear_damping
{
//  Serial.print("environment is hard surface");
//  Serial.print("\t");
force=-damping*filteredVelocity;
  

}

#endif

//_________c___________________________________________________________  


#ifdef Nonlinear_friction
{
//  Serial.print("environment is hard surface");
//  Serial.print("\t");
if  (filteredVelocity>0){
  force = 0.1*(-1+0.5*cos(xh));
}

else
force = 0.1*(1+0.5*cos(xh));

}
#endif

//_________D___________________________________________________________  


#ifdef Magnetic_interaction
{
//  Serial.print("environment is hard surface");
//  Serial.print("\t");
force=0.001/(xh-0.045)/(xh-0.045);

}
#endif

//_________E___________________________________________________________  

#ifdef hard_surface
{
//  Serial.print("environment is hard surface");
//  Serial.print("\t");
double Signoid=1/(1+exp(filteredVelocity));
double amp1=Signoid;

   if(xh>0.005) {
      force=amp1*(-stiffness*(xh-0.005));
   }
   else{
      force=0;
   }     
  }
#endif
//_________F___________________________________________________________  


#ifdef Bump_and_valley
{
//  Serial.print("environment is hard surface");
//  Serial.print("\t");
force=0.2;

}
#endif

//_________G___________________________________________________________  


#ifdef Texture
{
//  Serial.print("environment is hard surface");
//  Serial.print("\t");
double forcedirection=0;
if(filteredVelocity>0){
  forcedirection=-1;
}
else {
  forcedirection=1;
}
int positioninterval=0;
positioninterval=xh/0.01;
//Serial.println(positioninterval);

if((positioninterval/2)%2) {
      force=forcedirection*0.5;
   }
   else{
      force=0;
   }

}
#endif

//_________H___________________________________________________________  

#ifdef Mass_spring_damping
{
//  Serial.print("environment is hard surface");
//  Serial.print("\t");
//Serial.println(acceleration);
force=-(acceleration*mass+damping*filteredVelocity+stiffness*(xh));
}
#endif

//#ifdef ​M
//{
////  Serial.print("environment is hard surface");
////  Serial.print("\t");
//force=mass*acceleration;
//
//}
//#endif


  // Define kinematic parameters you may need
 double rp = 0.0048;   //[m]
 double rs = 0.067;   //[m] 
  // Step C.1: 
 //double force = 0.2; // You will generate a force by simply assigning this to a constant number (in Newtons)
  // Step C.2: 
  Tp = force*rh/rs*rp;    // Compute the require motor pulley torque (Tp) to generate that force
 
  //*************************************************************
  //******************* Rendering Algorithms ********************
  //*************************************************************
  // ADD YOUR CODE HERE
  
  // NOTE: good to use #ifdef statements for the various environments
//oldtime=newtime;
//oldposition= newposition;
//oldvelocity=newvelocity;
//Serial.println(velocity);
//Serial.println("\t");
//Serial.println(filteredVelocity);


  //*************************************************************
  //*** Section 4. Force output (do not change) *****************
  //*************************************************************
  
  // Determine correct direction for motor torque
  if(force > 0) { 
    digitalWrite(dirPin, HIGH);
  } else {
    digitalWrite(dirPin, LOW);
  }

  // Compute the duty cycle required to generate Tp (torque at the motor pulley)
  duty = sqrt(abs(Tp)/0.0183);

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
void setPwmFrequency(int pin, int divisor) {
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
