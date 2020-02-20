#include <WiFi.h>
#include <PubSubClient.h>
#include <ArduinoJson.h>
#include "DFRobot_Aliyun.h"

#define LED1 32  //定义一个灯的引脚，这个灯用于监测程序有没有死机。
#define LED  33  //定义灯的引脚
#define BUTTON 34  //定义按键的引脚

/*定义两个函数用与millis定时执行上传阿里云数据*/
unsigned long previousMillis=0;  //
unsigned long period=300000;   //上传间隔为5分钟一次

/*1602I2C模块地址3F*/
#include <LiquidCrystal_I2C.h>
LiquidCrystal_I2C lcd(0x3f,16,2);//set the LCD address to 0x3F for a 16 chars and 2 line display

/*GY-30使用的库文件和接线*/
#include <Wire.h>
// GY-30
// BH1750FVI
// in ADDR 'L' mode 7bit addr 地址脚接低电平用0b0100011地址
#define ADDR 0b0100011
// addr 'H' mode
// #define ADDR 0b1011100 地址脚接高电平用0b1011100地址
int mlux;
int val; //定义光照变量

/*SHT1x使用的库文件和接线*/
#include <SHT1x.h>
#define dataPin  25
#define clockPin 26
SHT1x sht1x(dataPin, clockPin);
  int Temp; //定义温度变量
  int Humi; //定义湿度变量
  
 
/*配置WIFI名和密码*/
const char * WIFI_SSID     = "TP-LINK_628C";
const char * WIFI_PASSWORD = "13469890713";
 
/*配置设备证书信息*/
String ProductKey = "a10g5ShKDtk";
String ClientId = "12345";/*自定义ID*/
String DeviceName = "ESP32";
String DeviceSecret = "NOUAKtmtkJKPWkrGuDMiUnR3L6mynpIS";
 
/*配置域名和端口号*/
String ALIYUN_SERVER = "iot-as-mqtt.cn-shanghai.aliyuncs.com";
uint16_t PORT = 1883;
 
/*需要操作的产品标识符(温度和湿度，光照标识符)*/
String TempIdentifier = "Temp";
String HumiIdentifier = "Humi";
String mluxIdentifier = "mlux";  //光照的标识符
String Identifier = "LightSwitch"; //灯光开关变量

 
/*需要上报和订阅的两个TOPIC*/
const char * subTopic = "/sys/a10g5ShKDtk/ESP32/thing/service/property/set";//****set
const char * pubTopic = "/sys/a10g5ShKDtk/ESP32/thing/event/property/post";//******post

 
DFRobot_Aliyun myAliyun;
WiFiClient espClient;
PubSubClient client(espClient);

/*ButtonState、LEDState用来存储LED灯同BUTTON按钮的当前值*/
uint8_t ButtonState = 0;  //8位无符号整型数(int)
uint16_t LEDState = 0;    //16位无符号整型数(int)  
/*开灯指令*/
static void openLight(){
  digitalWrite(LED, HIGH);
}
 /*关灯指令*/
static void closeLight(){
  digitalWrite(LED, LOW);
}

/*GY-30子程序*/
 void GY(){
  
  // reset
  Wire.beginTransmission(ADDR);
  Wire.write(0b00000111);
  Wire.endTransmission();
 
  Wire.beginTransmission(ADDR);
  Wire.write(0b00100000);
  Wire.endTransmission();
  // typical read delay 120ms
  delay(120);
  Wire.requestFrom(ADDR, 2); // 2byte every time
  for (val = 0; Wire.available() >= 1; ) {
    char c = Wire.read();
    //Serial.println(c, HEX);
    val = (val << 8) + (c & 0xFF);
  }
  val = val / 1.2;
 //   Serial.print("光照");
 //   Serial.println(val);
  }
 
void connectWiFi(){
  Serial.print("Connecting to ");
  Serial.println(WIFI_SSID);
  WiFi.begin(WIFI_SSID,WIFI_PASSWORD);
  while(WiFi.status() != WL_CONNECTED){
    delay(500);
    Serial.print(".");
  }
  Serial.println();
  Serial.println("WiFi connected");
  Serial.print("IP Adderss: ");
  Serial.println(WiFi.localIP());
}


/*回调函数用于接受阿里云下发的数据
void callback(char * topic, byte * payload, unsigned int len){
  Serial.print("Recevice [");
  Serial.print(topic);
  Serial.print("] ");
  for (int i = 0; i < len; i++){
    Serial.print((char)payload[i]);
  }
  Serial.println();
}
*/
 
  /*回调*/
void callback(char * topic, byte * payload, unsigned int len){
  Serial.print("Recevice [");
  Serial.print(topic);
  Serial.print("] ");
  for (int i = 0; i < len; i++){
    Serial.print((char)payload[i]);
  }
  Serial.println();
  StaticJsonBuffer<300> jsonBuffer;
  JsonObject& root = jsonBuffer.parseObject((const char *)payload);
  if(!root.success()){
    Serial.println("parseObject() failed");
    return;
  }
  const uint16_t LightStatus = root["params"][Identifier];
  if(LightStatus == 1){
    openLight();
  }else{
    closeLight();
  }
  /*当LED灯的状态发生改变后，则上报LED灯的当前状态*/
  if(LightStatus != LEDState)
  {
        LEDState = LightStatus;
        String tempMseg = "{\"id\":"+ClientId+",\"params\":{\""+Identifier+"\":"+(String)LightStatus+"},\"method\":\"thing.event.property.post\"}";
        char sendMseg[tempMseg.length()];
        strcpy(sendMseg,tempMseg.c_str());
        client.publish(pubTopic,sendMseg);
  }
}
 
void ConnectAliyun(){
  while(!client.connected()){
    Serial.print("Attempting MQTT connection...");
    /*根据自动计算的用户名和密码连接到Alinyun的设备，不需要更改*/
    if(client.connect(myAliyun.client_id,myAliyun.username,myAliyun.password)){
      Serial.println("connected");
      client.subscribe(subTopic);
    }else{
      Serial.print("failed, rc=");
      Serial.print(client.state());
      Serial.println(" try again in 5 seconds");
      delay(5000);
    }
  }
}
void setup(){
  Serial.begin(115200);

pinMode(LED1, OUTPUT);  //将32脚的LED设定为输出

  /*将LED（D2）设置为IO输出，BUTTON（D3）设置为IO输入*/
  pinMode(LED,OUTPUT);
  pinMode(BUTTON,INPUT);
  /*连接WIFI*/
  connectWiFi();
   
  /*初始化Alinyun的配置，可自动计算用户名和密码*/
  myAliyun.init(ALIYUN_SERVER,ProductKey,ClientId,DeviceName,DeviceSecret);
   
  client.setServer(myAliyun.mqtt_server,PORT);
   
  /*设置回调函数，当收到订阅信息时会执行回调函数*/
  client.setCallback(callback);
   
  /*连接到Aliyun*/
  ConnectAliyun();
  
  /*GY-30用初始化*/
  Wire.begin();
  Wire.beginTransmission(ADDR);
  Wire.write(0b00000001);
  Wire.endTransmission();
  
  /*初始化1602*/
  lcd.begin(21,22);// initialize the lcd with SDA and SCL pins
 // lcd.backlight(); //这个语句用于打开背光，关掉背光节约电池。
}
 
void loop(){
  unsigned long currentMillis = millis(); // store the current time。存储当前时间
                               
  if(!client.connected()){
      ConnectAliyun();
                         }
  /*5钟上报一次次温湿度信息*/
  if(currentMillis-previousMillis >= period){ //如果当前时间减去上次定时时间大于或是等于间隔时间就执行下面的语句
     previousMillis = currentMillis;  //当前时间赋值给定时时间
   lcd.clear();  //lcd清屏指令
	 GY(); //执行一次光照监测程序
	 Temp = sht1x.readTemperatureC();  //华氏
	 Humi = sht1x.readHumidity();
	 mlux = val;//光照数据
   
	Serial.print("温度");
	Serial.println(Temp);
	Serial.print("湿度");
	Serial.println(Humi);
	Serial.print("光照");
	Serial.println(val);

	lcd.setCursor(0,0);
	lcd.print("T:");
	lcd.print(Temp);  //1602第1列第1排显示光照  
	lcd.setCursor(6,0);
	lcd.print("H:");
	lcd.print(Humi);  //1602第1列第6排显示光照		  
	lcd.setCursor(0,1);
	lcd.print("Illuminance:");
	lcd.print(val);  //1602第2列第一排显示光照
  
  client.publish(pubTopic,("{\"id\":"+ClientId+",\"params\":{\""+TempIdentifier+"\":"+Temp+",\""+HumiIdentifier+"\":"+Humi+",\""+mluxIdentifier+"\":"+mlux+"},\"method\":\"thing.event.property.post\"}").c_str());
   } 

   /*灯光闪烁用于监测ESP32有没有死机*/
   digitalWrite(LED1, HIGH); 
   delay(500); 
   digitalWrite(LED1, LOW);
   delay(500);                               
  client.loop();
}
