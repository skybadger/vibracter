/*
   This program uaes the Huzzah esp8266 board to drive 3 DC brushed motors
   operating torquing reaction wheels to stabilise a platform to a target profile
   provided ahead of time by the source web client.
   In the test case, this profile is captured by a sensor device on a sports device like a racquet=, club or bow and
   the gyro used to stabilise and guide the sportsman control of the trajectory of that device as
   they perform the shot they are practising.
   Practical memory size limits prevent the entire profile from being uploaded ahead of time.
   So each 1000 points are held in memory and when the last 10% is reached the next are requested.
   That requires 2-way web connections.
  // REST URL handling
  // /reset (wo)
  // /pidMasterEnable (r/w)
  // /profile (r/w)
  // /xPid/Ki, Kp, Kd, pidEnable, actual (r/w)
  // /yPid/Ki, Kp, Kd, pidEnable, actual (r/w)
  // /zPid/Ki, Kp, Kd, pidEnable, actual (r/w)
  pin    - motor direction motor 1
  Pin 12 - pwm out to motor 2
  Use http://esp8266/xpid/Ki to retrieve the Ki variable
  Use http://esp8266/xpid PUT {"Ki":"3.14159"]}; to set it

  Pins used:
  Pin 11 - pwm out to motor 1
  pin    - motor direction motor 2
  Pin 13 - pwm out to motor 3
  pin    - motor direction motor 3

  Serial debugout
  pin 15 - Tx out
  pin    - Rx in
  PID library url:
  JSON library URL:// https://github.com/bblanchon/ArduinoJson
  ESP8266 Arduio cross-compile url:
*/

#include <ESP8266WiFi.h>
#include <ESP8266WiFiAP.h>
#include <ESP8266WiFiGeneric.h>
#include <ESP8266WebServer.h>
#include <WiFiClient.h>
//#include <WiFiClientSecure.h>
//#include <WiFiServer.h>
#include <WiFiUdp.h>
#include <dummy.h>
#include <PID_v1.h>
#include <ArduinoJson.h>
#include <time.h>

/* Some of the code is based on examples in:
   "20A-ESP8266__RTOS_SDK__Programming Guide__EN_v1.3.0.pdf"
*/
#ifdef ESP8266
extern "C"
{
#include "user_interface.h"
#include "pwm.h"
}
#endif
#include <string.h>

//External pins used
#define N_PWM 3
int pwm_pin[] = {14, 5, 15};

//Create PID controllers & their settings
enum axiis { X_AXIS, YAXIS, Z_AXIS, TIME_AXIS};
double pidSettings[3][6] = {{ 128, 100, 0, 2, 1, 5 }, { 128, 100, 0, 2, 1, 5 }, {128, 100, 0, 2, 1, 5 } };
//xSetpoint, xInput, xOutput, xKp = 2, xKd = 1, xKi = 5;
PID xPid( &pidSettings[0][0], &pidSettings[0][1], &pidSettings[0][2], pidSettings[0][3], pidSettings[0][4], pidSettings[0][5], DIRECT);
PID yPid( &pidSettings[1][0], &pidSettings[1][1], &pidSettings[1][2], pidSettings[1][3], pidSettings[1][4], pidSettings[1][5], DIRECT);
PID zPid( &pidSettings[2][0], &pidSettings[2][1], &pidSettings[2][2], pidSettings[2][3], pidSettings[2][4], pidSettings[2][5], DIRECT);

//State information
#define MAX_PROFILE_SIZE 1000
#define PROFILE_DIMENSION 3 //last is time in usecs
float profile[MAX_PROFILE_SIZE][PROFILE_DIMENSION + 1 ];
boolean profileLoaded = false;
int profileIndex;
int profileSize = 0;
float** ppfProfile;

boolean masterPidEnable, xPidEnable, yPidEnable, zPidEnable;

//Processor time when profile activated
double usProfileTimeCount = 0;
long int startTime; //Time when profile was activated
long int currentTime;//Time of last profile index update

//Web server data formatter
DynamicJsonBuffer jsonBuffer;

// Create an instance of the server
// specify the port to listen on as an argument
ESP8266WebServer server(80);

//Handler function definitions
void handleRoot();
void handlePutPidVars();
void handleGetPidVars();
void handleGetPidKp();
void handleGetxPidKd();
void handleGetxPidKi();
void handleGetxPidEnable();
void handleGetxPidActual();

void writeResponse( WiFiClient& client, JsonObject& json)
{
  client.println( json.measureLength() );
  json.printTo(client);
}

void setup()
{
  // Reuse default Serial port rate, so the bootloader
  // messages are also readable.

  Serial.begin(115200);
  
  Serial.println();
  Serial.println(F("ESP starting."));

  // System usually boots up in about 200ms.
  Serial.print(F("system_get_time(): "));
  Serial.println(system_get_time());

  //WiFi.begin("BadgerHome", "This is Badger Home");
  WiFi.begin("Skybadger Away", "");

  Serial.print("Connecting");
  while (WiFi.status() != WL_CONNECTED)
  {
    delay(500);
    Serial.print(".");
  }
  Serial.println();

  Serial.print("Connected, IP address: ");
  Serial.println(WiFi.localIP());

  // prepare motor output pins - Dirn and Size - pwm pin
  pinMode(2, OUTPUT);
  digitalWrite(2, 0);

  //Configure output pins - need 3 PWM
  analogWriteRange(1000);
  for (int i = 0; i < N_PWM; i++)
  {
    pinMode(pwm_pin[i], OUTPUT);
    analogWrite(pwm_pin[i], 50);
  }

  //Setup the PID controllers
  //initialize the variables we're linked to
  masterPidEnable = xPidEnable = yPidEnable = zPidEnable = false;

  //Set the PID to standby until are enabled or explicitly called
  xPid.SetMode(MANUAL);
  yPid.SetMode(MANUAL);
  zPid.SetMode(MANUAL);

  //Setup json library

  //Setup webserver handler functions
  //Global settings
  server.on("/", handleRoot);
  server.on("/reset", handleRootReset);
  server.on("/masterPidEnable", HTTP_PUT, handleRootMasterPidEnable );
  server.on("/masterPidEnable", HTTP_GET, handleRootMasterPidEnable );
  //Velocity profile settings
  server.on("/profile", handleProfileEntry );
  //Individual axis settings - X
  server.on("/profile/Pid/", HTTP_PUT, handlePutPidVars );//one to all vars set in one hit
  server.on("/profile/Pid/", HTTP_GET, handleGetPidVars );//All vars retrieved in one go
  server.on("/profile/Pid/x/Kp", HTTP_GET, handleGetPidKp );
  server.on("/profile/Pid/x/Kd", HTTP_GET, handleGetPidKd );
  //server.on("/profile/Pid/x/Ki", HTTP_GET, handleGetPidKi );
  server.on("/profile/Pid/x/enable", HTTP_GET, handleGetxPidEnable);
  server.on("/profile/Pid/x/actual", HTTP_GET, handleGetxPidActual );
  //Individual axis settings - Y
  //server.on("/profile/Pid/y/Kp", handleyPidKp );
  //server.on("/profile/Pid/y/Kd", handleyPidKd );
  //server.on("/profile/yPid/Ki", handleyPidKi );
  //server.on("/profile/yPid/enable", handleyPidEnable);
  //server.on("/profile/yPid/enable", HTTP_GET, handleGetyPidEnable);
  //server.on("/profile/yPid/actual", HTTP_GET, handleGetyPidActual );
  //Individual axis settings - Z
  //server.on("/profile/zPid/Kp", handlezPidKp );
  //server.on("/profile/zPid/Kd", handlezPidKd );
  //server.on("/profile/zPid/Ki", handlezPidKi );
  //server.on("/profile/zPid/enable", handlezPidEnable);
  //server.on("/profile/zPid/enable", HTTP_GET, handleGetzPidEnable);
  //server.on("/profile/zPid/actual", HTTP_GET, handleGetzPidActual );
  server.onNotFound(handleNotFound);
  //Don't need a post handler yet
  //server.onUpload();
  server.begin();
}

String& debugURI( String& message)
{
  message += "URI: ";
  message += server.uri();
  message += "\nMethod: ";
  switch (server.method())
  {
    case HTTP_GET:  message += "GET"; break;
    case HTTP_POST: message += "POST"; break;
    case HTTP_PUT:  message += "PUT"; break;
    case HTTP_PATCH:  message += "PATCH"; break;
    case HTTP_DELETE:  message += "DELETE"; break;
    case HTTP_OPTIONS: message += "OPTIONS"; break;
      break;
    default:
      break;
  }
  message += "\nArguments: ";
  message += server.args();
  message += "\n";
  for (uint8_t i = 0; i < server.args(); i++)
  {
    message += " " + server.argName(i) + ": " + server.arg(i) + "\n";
  }
  return message;
}

void handleNotFound()
{
  String message = "File Not Found\n\n";
  message = debugURI( message);
  server.send(404, "text/plain", message);
}

void handleRoot()
{
  server.send(200, "text/plain", "hello from esp8266!");
  //  Serial.println( json.measureLength() );
  //  json.printTo(client);
}

void handleRootReset()
{
  server.send(200, "text/plain", "esp8266 Resetting");
  Serial.println( "Server resetting" );
}

void handleRootMasterPidEnable()
{
  String message("");
  if ( server.method() == HTTP_PUT )
  {
    message = debugURI(message);
    Serial.println( "masterPidENable: args received: " + message );
    JsonObject& root = jsonBuffer.parseObject ( server.arg(0) );
    if ( root.success() )
    {
      const char* value = root.get<const char*>("masterPidEnable");
      if ( strcmp( value, "true" ) == 0 )
      {
        masterPidEnable = true;
        startTime = system_get_time();
        currentTime = 0L;
      }
      else
      {
        startTime = 0L;
        currentTime = 0L;
        masterPidEnable = false;
      }
      server.send(200, "text/plain", "content accepted" );
    }
    else
    {
      debugURI(message);
      Serial.println( "masterPidEnable: failed to parse upload: " + message );
      server.send( 403, "text/plain", message + "\nentity not found");
    }
  }
  else if ( server.method() == HTTP_GET )
  {
    JsonObject& root = jsonBuffer.createObject();
    root["masterPidEnable"] = (masterPidEnable) ? "true" : "false" ;
    root.printTo(message);
    server.send(200, "application/json", message);
  }
  else
  {
    handleNotFound();
  }
  Serial.println( "masterPidEnable handled" );
}
/*
  Reminder:
  #define MAX_PROFILE_SIZE 1000
  #define PROFILE_DIMENSION 3 //last dimennsions is time in usecs
  float profile[PROFILE_DIMENSION + 1 ][MAX_PROFILE_SIZE];
  boolean profileLoaded = false;
  int profileIndex;
  int profileSize = 0;
*/
/*
    Expect json object { "startIndex":<int>, "length": <int>, "data":[[,,,],[,,,], ... ] }
    length is optional
    data might be single array quad or multiple comma separated quads nested in brackets
    data is <float>,<float>,<float>,<long int>
*/
void handleProfileEntry ()
{
  String message("");
  int startIndex = 0;
  int arrayLength = 0;
  int arrayLpCtr = 0;
  boolean dataParseError = false;
  //JSON buffer size is 2K, Web server buffer size is 2K (?)
  //replace with defined constant
#define MAX_BUFFER_SIZE 2048

  int maxChunkSize = MAX_BUFFER_SIZE / ((3 * sizeof(float)) + sizeof(unsigned int) );

  //Debug
  message = debugURI(message);
  Serial.println( "ProfileEntry: args received: " + message );

  if ( server.method() == HTTP_PUT )
  {
    if ( masterPidEnable )
    {
      //Can't hack profile when its in use
      server.send(403, "text/plain", "Entity in use. Disable and try again." );
      return;
    }

    JsonObject& root = jsonBuffer.parseObject ( server.arg(0) );
    if ( root.success() )
    {
      //If 'startIndex' is present then write data to profile at that index
      if ( root.containsKey("startIndex") )
      {
        startIndex = root.get<int>("startIndex");
      }

      if ( startIndex > profileSize )
      {
        //Invalid start point - needs to be at end of existing data or earlier
        server.send(403, "text/plain", "Entity start past current extents" );
        return;
      }

      if ( root.containsKey("data") )
      {
        //Requires data in to be in the form "data":[[1.0,2.0,3.0,4.0],[5.1,6.2,7.3,8..4], ...  ]
        JsonArray& nestedArray = root["data"];
        arrayLength = nestedArray.size();
        if ( arrayLength > maxChunkSize )
        {
          //Set size to big to handle - needs to be chunked to N values or less.
          String message("Entity out of range - chunk 'put' to ");
          message += maxChunkSize;
          message += " or less data points";
          server.send(403, "text/plain", message.c_str() );
          return;
        }
        if ( arrayLength <= 0 )
        {
          //No data found - assume reset of profile
          profileSize = 0;
          profileIndex = 0;
          server.send(200, "text/plain", "No data found - profile set truncated to zero" );
          return;
        }
        if ( startIndex == 0 )
        {
          //reset profile to new data from scratch.
          profileSize = 0;
        }
        else if ( ( startIndex + arrayLength ) > profileSize && arrayLength > 0 )
        {
          //reset profile Length to start of new data since we can't assume length
          profileSize = startIndex;
        }

        //loop over nested array data
        arrayLpCtr = 0;
        while ( arrayLpCtr < arrayLength && profileSize < (MAX_PROFILE_SIZE - 1)  && !dataParseError )
        {
          JsonArray& subNestedArray = nestedArray[arrayLpCtr].asArray();
          arrayLength = subNestedArray.size();
          if ( arrayLength == 4 )
          {
            //Best way of handling errors here ?
            profile[0][profileSize] = subNestedArray[0].as<float>();
            profile[1][profileSize] = subNestedArray[1].as<float>();
            profile[2][profileSize] = subNestedArray[2].as<float>();
            profile[3][profileSize] = subNestedArray[3].as<float>();
            profileSize++;
            arrayLpCtr++;
          }
          else
            dataParseError = true;
        }
        if (dataParseError)
        {
          //In case of error we finish with profile size updated to where the data failed.
          //Fix is to either resend at original start point with valid data or continue with data from point of failure
          server.send(403, "text/plain", "Entity data not parsed - update failed" );
          return;
        }
      }
    }
    else if ( server.method() == HTTP_GET )
    {
      JsonObject& root = jsonBuffer.createObject();
      root["profileSize"] = profileSize;
      root["profileIndex"] = profileIndex;
      root["maxChunkSize"] = maxChunkSize;
      root["maxProfileSize"] = MAX_PROFILE_SIZE;
      root["profileDuration"] = profile[3][profileIndex] - profile[3][0];
      root.printTo(message);
      server.send(200, "application/json", message);
    }
    else
    {
      handleNotFound();
    }
  }
Serial.println( "handleProfileEntry handled" );  
}
  void handleGetPidVars()
  {
    String message;
    JsonObject& root = jsonBuffer.createObject();
    if (server.uri().indexOf("\/x\/") != -1 || server.uri().indexOf("\\x\\") != -1 )
    {
      root.set("Kp", String(pidSettings[0][3]) );    
      root.set("Kd", String(pidSettings[0][4]) );
      root.set("Ki", String(pidSettings[0][5]) );
      root.set("PidEnable", String( xPidEnable ) );
      root.set("actual", String( pidSettings[0][1]) );
      root.printTo( message );
      Serial.println("GetxPidVars(X): jason output -");
      Serial.println(message);
      server.send(200, "application/json", message);
    }
    else if (server.uri().indexOf("\/y\/") != -1 || server.uri().indexOf("\\y\\") != -1 )
    {
      root.set("Kp", String(pidSettings[1][3]) );    
      root.set("Kd", String(pidSettings[1][4]) );
      root.set("Ki", String(pidSettings[1][5]) );
      root.set("PidEnable", String( yPidEnable ) );
      root.set("actual", String( pidSettings[1][1]) );
      root.printTo( message );
      Serial.println("GetPidVars(Y): jason output -");
      Serial.println(message);
      server.send(200, "application/json", message);      
    }
    else  if (server.uri().indexOf("\/z\/") != -1 || server.uri().indexOf("\\z\\") != -1 )
    {
      root.set("Kp", String(pidSettings[2][3]) );    
      root.set("Kd", String(pidSettings[2][4]) );
      root.set("Ki", String(pidSettings[2][5]) );
      root.set("PidEnable", String( zPidEnable ) );
      root.set("actual", String( pidSettings[2][1]) );
      root.printTo( message );
      Serial.println("GetPidVars(Z): jason output -");
      Serial.println(message);
      server.send(200, "application/json", message);
    }
    else
    {
      debugURI(message);
      Serial.println( "/[XYZ]/GetPidVars: failed to parse axis: " + message );
      server.send( 403, "text/plain", message + "\nentity not found");
    }    
  }

  void handleGetPidKp()
  {
    String message;
    JsonObject& root = jsonBuffer.createObject();
    boolean success = false;
    if (server.uri().indexOf("\/z\/") != -1 || server.uri().indexOf("\\z\\") != -1 )
    {
      root["Kd"] = String(pidSettings[0][3]);  
      root.printTo( message );
      success = true;
    }
    else  if (server.uri().indexOf("\/z\/") != -1 || server.uri().indexOf("\\z\\") != -1 )
    {
      root["Kd"] = String(pidSettings[1][3]);  
      root.printTo( message );
      success = true;
    }
    else  if (server.uri().indexOf("\/z\/") != -1 || server.uri().indexOf("\\z\\") != -1 )
    {
      root["Kd"] = String(pidSettings[2][3]);  
      root.printTo( message );
      success = true;
    }
    else
    {
      debugURI(message);  
    }
    if ( success )
      server.send(200, "application/json", message);
    else
      server.send(403, "application/text", "Failed to find axis" + message);
  }

  void handleGetPidKd()
  {
    String message;
    JsonObject& root = jsonBuffer.createObject();
    boolean success = false;
    if (server.uri().indexOf("\/z\/") != -1 || server.uri().indexOf("\\z\\") != -1 )
    {
      root["Kd"] = String(pidSettings[0][4]);  
      root.printTo( message );
      success = true;
    }
    else  if (server.uri().indexOf("\/z\/") != -1 || server.uri().indexOf("\\z\\") != -1 )
    {
      root["Kd"] = String(pidSettings[1][4]);  
      root.printTo( message );
      success = true;
    }
    else  if (server.uri().indexOf("\/z\/") != -1 || server.uri().indexOf("\\z\\") != -1 )
    {
      root["Kd"] = String(pidSettings[2][4]);  
      root.printTo( message );
      success = true;
    }
    else
    {
      debugURI(message);  
    }
    if ( success )
      server.send(200, "application/json", message);
    else
      server.send(403, "application/text", "Failed to find axis" + message);
  }

  void handleGetxPidKi()
  {
    String message;
    JsonObject& root = jsonBuffer.createObject();
    root["Ki"] = String(pidSettings[0][5]);
    root.printTo( message );
    server.send(200, "application/json", message);
  }
  void handleGetxPidEnable()
  {
    String message;
    JsonObject& root = jsonBuffer.createObject();
    root["PidEnable"] = String( xPidEnable );
    root.printTo( message );
    Serial.println( "GetxPidEnable - response: " + message );
    server.send(200, "application/json", message);
  }

  void handleGetxPidActual()
  {
    String message;
    JsonObject& root = jsonBuffer.createObject();
    root["actual"] = String( pidSettings[0][1] );
    root.printTo( message );
    Serial.println( "GetxPidActual - response: " + message );
    server.send(200, "application/json", message);
  }

  //Generic for all axiis.
  void handlePutPidVars()
  {
    String message;
    int keysCtr = 0;
    int pidSettingsRoot = 0; //default to X to start
    JsonObject& root = jsonBuffer.parseObject ( server.arg(0) );

    if (server.uri().indexOf("\/x\/") != -1 || server.uri().indexOf("\\x\\") != -1 )
    {
      pidSettingsRoot = 0;
    }
    else if (server.uri().indexOf("\/y\/") != -1 || server.uri().indexOf("\\y\\") != -1 )
    {
      pidSettingsRoot = 1;
    }
    else if (server.uri().indexOf("\/z\/") != -1 || server.uri().indexOf("\\z\\") != -1 )
    {
      pidSettingsRoot = 2;
    }

    if ( root.success() )
    {
      root.printTo( message );
      if (root.containsKey("Kp") )
      {
        float value = root.get<float>("Kp");
        pidSettings[pidSettingsRoot][3] = value;
        keysCtr++;
      }
      if ( root.containsKey("Kd") )
      {
        float value = root.get<float>("Kd");
        pidSettings[pidSettingsRoot][4] = value;
        keysCtr++;
      }
      if ( root.containsKey("Ki") )
      {
        float value = root.get<float>("Ki");
        pidSettings[pidSettingsRoot][5] = value;
        keysCtr++;
      }
      if ( root.containsKey("PidEnable") )
      {
        boolean value = root.get<int>("PidEnable");
        switch( pidSettingsRoot)
        {
          case 0:xPidEnable = (value == 1) ? true : false;break;
          case 1:yPidEnable = (value == 1) ? true : false;break;
          case 2:zPidEnable = (value == 1) ? true : false;break;
          default: break;
        }
        keysCtr++;
      }
      if (root.containsKey("actual") )
      {
        float value = root.get<float>("actual");
        pidSettings[pidSettingsRoot][1] = value;
        keysCtr++;
      }
      if ( keysCtr == 0 )
      {
        Serial.println(" xPid PUT - no content found" );
        Serial.println( message );
        server.send(403, "text/plain", "entity not updated");
      }
      else
      {
        server.send(200, "application/json", message);
      }
    }
  }

  /*
    Create code example from Git used for PUT
    void handleCreate()
    {
    if(server.args() == 0)
      return returnFail("BAD ARGS");

    String path = server.arg(0);
    return;
    }
  */
  //Upload handler code from Git used for POST of file attachments.
  //It used to use a SD card file system
  /*
    void handleFileUpload()
    {
    HTTPUpload& upload = server.upload();
    space = malloc( HTTP_UPLOAD_BUFLEN );

    if(upload.status == UPLOAD_FILE_START)
    {
      Serial.println("Upload: START, filename: ");
      //DBG_OUTPUT_PORT.println(upload.filename);
    }
    else if(upload.status == UPLOAD_FILE_WRITE)
    {
      if(space)
        memcpy( upload.buf, upload.currentSize );
      Serial.println("Upload: WRITE, Bytes: " + upload.currentSize);
      //DBG_OUTPUT_PORT.println(upload.currentSize);
    }
    else if(upload.status == UPLOAD_FILE_END)
    {
      if(space)
        free(space);

      //DBG_OUTPUT_PORT.print("Upload: END, Size: "); DBG_OUTPUT_PORT.println(upload.totalSize);
    }
    }
  */

  void loop()
  {
    long int nowTime = system_get_time();
    long int indexTime = startTime - nowTime;

    server.handleClient();

    //Get next target value from stored profile if PID enabled
    if ( masterPidEnable )
    {
      //Keep up with any missed profile timepoints
      while ( !(profile[profileIndex][3] < indexTime && profile[profileIndex + 1][3] > indexTime) )
      {
        profileIndex++;
      }
      if ( profile[profileIndex][3] != currentTime )
      {
        if ( xPidEnable)
        {
          xPid.Compute();
          //set motor speed and direction
        }
        if ( yPidEnable )
        {
          yPid.Compute();
          //set motor speed and direction
        }
        if ( zPidEnable)
        {
          zPid.Compute();
          //set motor speed and direction
        }
      }
      //Track so we apply only once per time interval
      currentTime = profile[profileIndex][3];
    }
  }
