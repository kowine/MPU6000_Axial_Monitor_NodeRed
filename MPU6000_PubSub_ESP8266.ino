#include <PubSubClient.h>
#include <ESP8266WiFi.h>
#include <Wire.h>

//Define I2C address
#define Addr 0x68

//Wifi Credentials
#define wifi_ssid "DcubeAirtel"
#define wifi_password "D@Airtel190"

//Define MQTT server and topics
#define mqtt_server "iot.eclipse.org"
#define X_topic "AccelX"
#define Y_topic "AccelY"
#define Z_topic "AccelZ"
#define XR_topic "RotateX"
#define YR_topic "RotateY"
#define ZR_topic "RotateZ"

WiFiClient espClient;
PubSubClient client;

//Global Variable
volatile float Xtopic, Ytopic, Ztopic, XRtopic, YRtopic, ZRtopic; // using volatile with variable - It tells the compiler that the value of the variable may change at any time--without any action being taken by the code the compiler finds nearby.
uint32_t timer;
double  yaw1,yaw2,yaw3; //These are the angles in the complementary filter(roll, pitch)
float rollangle,pitchangle;

// Setup
void setup() {
  Wire.begin(2,14);
  Serial.begin(115200);
  setup_wifi();
  client.setServer(mqtt_server, 1883);
  client.setClient(espClient);
//  //start a timer
//  timer = micros();
  }

//Wifi Setup
void setup_wifi() {
  delay(10);
  // We start by connecting to a WiFi network
  Serial.println();
  Serial.print("Connecting to ");
  Serial.println(wifi_ssid);

  WiFi.begin(wifi_ssid, wifi_password);

  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }

  Serial.println("");
  Serial.println("WiFi connected");
  Serial.println("IP address: ");
  Serial.println(WiFi.localIP());
}

//Reconnect
void reconnect() {
  // Loop until we're reconnected
  while (!client.connected()) {
    Serial.print("Attempting MQTT connection...");
      if (client.connect("ESP8266Client")) {
      Serial.println("connected");
    } 
    else {
      Serial.print("failed, rc=");
      Serial.print(client.state());
      Serial.println(" try again in 5 seconds");
      // Wait 5 seconds before retrying
      delay(500);
    }
  }
}


void loop()
{
//  delay(100);
//  Timer = millis();
//  while(millis()- Timer<=Interval)// use intervels as per mentioned earlier
//  {
  tempTask();
  delay(100);
//  }
  
  if (!client.connected()) {
    reconnect();
  }
 
  //Mentioned below directly executed in String url
  
  Serial.print("X Axis: ");
  Serial.println(String(Xtopic).c_str());
  client.publish(X_topic, String(Xtopic).c_str(), true);

  Serial.print("Y Axis: ");
  Serial.println(String(Ytopic).c_str());
  client.publish(Y_topic, String(Ytopic).c_str(), true);
  
  Serial.print("Z Axis: ");
  Serial.println(String(Ztopic).c_str());
  client.publish(Z_topic, String(Ztopic).c_str(), true);

  Serial.print("X Rotate: ");
  Serial.println(String(XRtopic).c_str());
  client.publish(XR_topic, String(XRtopic).c_str(), true);

  Serial.print("Y Rotate: ");
  Serial.println(String(YRtopic).c_str());
  client.publish(YR_topic, String(YRtopic).c_str(), true);

  Serial.print("Z Rotate: ");
  Serial.println(String(ZRtopic).c_str());
  client.publish(ZR_topic, String(ZRtopic).c_str(), true);
  client.loop();
  }
  

void tempTask()
{
   // Start I2C transmission
  Wire.beginTransmission(Addr);
  // Select gyroscope configuration register
  Wire.write(0x1B);
  // Full scale range = 2000 dps
  Wire.write(0x18);
  // Stop I2C transmission
  Wire.endTransmission();
  
  // Start I2C transmission
  Wire.beginTransmission(Addr);
  // Select accelerometer configuration register
  Wire.write(0x1C);
  // Full scale range = +/-16g
  Wire.write(0x18);
  // Stop I2C transmission
  Wire.endTransmission();
  
  // Start I2C transmission
  Wire.beginTransmission(Addr);
  // Select power management register
  Wire.write(0x6B);
  // PLL with xGyro reference
  Wire.write(0x01);
  // Stop I2C transmission
  Wire.endTransmission();
  delay(300);
  
  unsigned int data[6];

  // Start I2C transmission
  Wire.beginTransmission(Addr);
  // Select data register
  Wire.write(0x3B);
  // Stop I2C transmission
  Wire.endTransmission();
  
  // Request 6 bytes of data
  Wire.requestFrom(Addr, 6);
  
  // Read 6 byte of data 
  if(Wire.available() == 6)
  {
    data[0] = Wire.read();
    data[1] = Wire.read();
    data[2] = Wire.read();
    data[3] = Wire.read();
    data[4] = Wire.read();
    data[5] = Wire.read(); 
  }
  
  // Convert the data
  int xAccl = data[0] * 256 + data[1];
  int yAccl = data[2] * 256 + data[3];
  int zAccl = data[4] * 256 + data[5];

  // Start I2C transmission
  Wire.beginTransmission(Addr);
  // Select data register 
  Wire.write(0x43);
  // Stop I2C transmission
  Wire.endTransmission();
  
  // Request 6 bytes of data
  Wire.requestFrom(Addr, 6);
  
  // Read 6 byte of data 
  if(Wire.available() == 6)
  {
    data[0] = Wire.read();
    data[1] = Wire.read();
    data[2] = Wire.read();
    data[3] = Wire.read();
    data[4] = Wire.read();
    data[5] = Wire.read(); 
  }
  // Convert the data
  int xGyro = data[0] * 256 + data[1];
  int yGyro = data[2] * 256 + data[3];
  int zGyro = data[4] * 256 + data[5];
  
  float gyroX = xGyro/262.2;
  float gyroY = yGyro/262.2;
  float gyroZ = zGyro/262.2;

// Manipulate the Gyro data
//  if(xGyro>1000){
//
//    xGyro -=2000;
//    
//    }
//  if(yGyro>1000){
//
//    yGyro -=2000;
//    
//    }
//    if(zGyro>1000){
//
//    zGyro -=2000;
//    
//    }

  float acclX = xAccl/4096;
  float acclY = yAccl/4096;
  float acclZ = zAccl/4096;

//  double dt = (double)(micros() - timer) / 1000000; //This line does three things: 1) stops the timer, 2)converts the timer's output to seconds from microseconds, 3)casts the value as a double saved to "dt".
//  timer = micros(); //start the timer again so that we can calculate the next dt.
  
//  the next two lines calculate the orientation of the accelerometer relative to the earth and convert the output of atan2 from radians to degrees
//  We will use this data to correct any cumulative errors in the orientation that the gyroscope develops.
//  rollangle=atan2(acclY,acclZ)*180/PI; // FORMULA FOUND ON INTERNET
//  pitchangle=atan2(acclX,sqrt(acclY*acclY+acclZ*acclZ))*180/PI; //FORMULA FOUND ON INTERNET

//  THE COMPLEMENTARY FILTER
//  This filter calculates the angle based MOSTLY on integrating the angular velocity to an angular displacement.
//  dt, recall, is the time between gathering data from the MPU6050.  We'll pretend that the 
//  angular velocity has remained constant over the time dt, and multiply angular velocity by 
//  time to get displacement.
//  The filter then adds a small correcting factor from the accelerometer ("roll" or "pitch"), so the gyroscope knows which way is down. 
//  roll = 0.99 * (roll+ gyroX * dt) + 0.01 * rollangle; // Calculate the angle using a Complimentary filter
//  pitch = 0.99 * (pitch + gyroY * dt) + 0.01 * pitchangle; 
  
//  if( yaw1 < 0 ) 
//  yaw1 += 180.0;
//
//  if( yaw2 < 0 ) 
//  yaw2 += 180.0;
//
//  if( yaw3 < 0 ) 
//  yaw3 += 180.0;
//  
  yaw3 = gyroZ;
  yaw2 = gyroY;
  yaw1 = gyroX;
  
  
  Xtopic = acclX;
  Ytopic = acclY;
  Ztopic = acclZ;
  XRtopic = yaw1; 
  YRtopic = yaw2;
  ZRtopic = yaw3;
  
  delay(10);
}
