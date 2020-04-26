int motorright = 9;
int motorrightdir = 7;
int motorleft = 10;
int motorleftdir = 8;
//defines pins numbers
const int echoPinFront = 5;
const int trigPinFront = A4;

const int echoPinLeft = 2;
const int trigPinLeft = A2;

const int echoPinRight = 4;
const int trigPinRight = A3;


#include <ros.h>
#include <geometry_msgs/Twist.h>
#include <std_msgs/Int16.h>
#include <SoftwareSerial.h>

class BlueROS : public ArduinoHardware
{
  protected:

  private:

  SoftwareSerial *mySerial;

  public:
  BlueRos(){}

  void init()
  {
      mySerial = new SoftwareSerial(A0,11);
      mySerial->begin(57600);  
  }

  int read()
  {
      return mySerial->read();
  };

  void write(uint8_t* data, int length)
  {
      for(int i=0; i<length; i++)
      {
        mySerial->write(data[i]);
      }
  }

    
};

ros::NodeHandle_<BlueROS, 3, 3, 100, 100> nh;

//ros::NodeHandle  nh;

std_msgs::Int16 msg_front;
std_msgs::Int16 msg_left;
std_msgs::Int16 msg_right;

ros::Publisher pubFront("Front", &msg_front);
ros::Publisher pubLeft("Left", &msg_left);
ros::Publisher pubRight("Right", &msg_right);

void ros_handler( const geometry_msgs::Twist& cmd_msg) {
float x = cmd_msg.linear.x;
float z = cmd_msg.angular.z;
if(x ==-2.0) backward(500);
if(x == 2.0) forward(500);
if(z == 2.0) left(500);
if(z ==-2.0) right(500);
stop();
}
ros::Subscriber<geometry_msgs::Twist> sub("/turtle1/cmd_vel", ros_handler);


// defines variables
long duration;
int distance;
//#include <SoftwareSerial.h>
//SoftwareSerial mySerial(0, 1); // RX, TX 0,1
void setup() {
pinMode(trigPinFront, OUTPUT); // Sets the trigPin as an Output
pinMode(echoPinFront, INPUT); // Sets the echoPin as an Input
nh.initNode();
nh.advertise(pubFront);

pinMode(trigPinLeft, OUTPUT); // Sets the trigPin as an Output
pinMode(echoPinLeft, INPUT); // Sets the echoPin as an Input
nh.initNode();
nh.advertise(pubLeft);

pinMode(trigPinRight, OUTPUT); // Sets the trigPin as an Output
pinMode(echoPinRight, INPUT); // Sets the echoPin as an Input
nh.initNode();
nh.advertise(pubRight);
pinMode(motorright, OUTPUT);

pinMode(motorleft, OUTPUT);
pinMode(motorrightdir, OUTPUT);
pinMode(motorleftdir, OUTPUT);
nh.initNode();
nh.subscribe(sub);
///nh.advertise(pub);

//mySerial.begin(9600); // Starts the serial communication
}
void loop() {


int distancecenter =  ultrasonic(echoPinFront,trigPinFront);
  msg_front.data = distancecenter;
  pubFront.publish( &msg_front);
  
  int distanceleft =  ultrasonic(echoPinLeft,trigPinLeft);
  msg_left.data = distancecenter;
  pubLeft.publish( &msg_left);
  

  int distanceright =  ultrasonic(echoPinRight,trigPinRight);
  msg_right.data = distancecenter;
  pubRight.publish( &msg_right);
  

 if(distanceleft > 30 && distanceright > 30)
  {
    forward(200);
  }
  if(distanceleft < 30)
  {
    right(200);
  }

  if(distanceright < 30)
  {
    left(200);
  }

  nh.spinOnce();
  delay(100);
}

void forward(int time)
{
digitalWrite(motorrightdir, LOW);
analogWrite(motorright,180);
digitalWrite(motorleftdir, LOW);
analogWrite(motorleft, 180);
delay(time);
stop();
}

void left(int time)
{
digitalWrite(motorrightdir, LOW);
analogWrite(motorright,180);

delay(time);
stop();
}
void right(int time)
{

digitalWrite(motorleftdir, LOW);
analogWrite(motorleft, 180);
delay(time);
stop();
}
void backward(int time)
{
digitalWrite(motorrightdir, HIGH);
analogWrite(motorright,180);
digitalWrite(motorleftdir, HIGH);
analogWrite(motorleft, 180);
delay(time);
stop();
}

void stop()
{
analogWrite(motorright, 0);
analogWrite(motorleft, 0);
}

int ultrasonic(int echoPin, int trigPin) {
// Clears the trigPin
digitalWrite(trigPin, LOW);
delayMicroseconds(2);
// Sets the trigPin on HIGH state for 10 micro seconds
digitalWrite(trigPin, HIGH);
delayMicroseconds(10);
digitalWrite(trigPin, LOW);
// Reads the echoPin, returns the sound wave travel time in microseconds
duration = pulseIn(echoPin, HIGH);
// Calculating the distance
distance= duration*0.034/2;
return distance;
}
