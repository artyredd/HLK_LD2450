#ifndef HLK_LD2450_h
#define HLK_LD2450_h

#include "limits.h"

//#define LOGGING 

static uint16_t _readAndLogSerial()
{
  uint16_t result = Serial1.read();
  Serial.print(result,HEX);
  Serial.print(' ');
  return result;
}

#ifdef LOGGING
#define write_char(_uint16_tValue) Serial1.write(_uint16_tValue);Serial.print(_uint16_tValue,HEX);Serial.print(' ')
#define read_char() _readAndLogSerial()
#define endline() Serial.print('\n')
#define log(...) Serial.print(__VA_ARGS__)
#else
#define write_char(_uint16_tValue) Serial1.write(_uint16_tValue)
#define read_char() Serial1.read()
#define endline() 
#define log(...)
#endif

typedef struct Command{
  uint16_t Word[2];
  size_t Size;
  uint16_t Values[32];
  bool Malformed;
  bool TimedOut;
} Command;

typedef enum AvailableBaudRates
{
  b9600 = 0x1, 
  b19200 = 0x2, 
  b38400 = 0x3, 
  b57600 = 0x4, 
  b115200 = 0x5, 
  b230400 = 0x6, 
  b256000 = 0x7, 
  b460800 = 0x8 
} AvailableBaudRates;

typedef struct _MacAddress{
  // Example: 8F 27 2E B8 0F 65
  char Bytes[6];
} MacAddress;

typedef enum ZoneFilteringType{
  // Disable region filtering
  Disable = 0x0,
  // Detect only the set region
  DetectRegion = 0x1,
  // Do not detect the set area
  DisableRegion = 0x2
} ZoneFilteringType;

// Each vertex is represented by X and Y
// coordinates, respectively, and the coordinate values is
// signed int 16 in millimeters
// All zeros(0) means this region is not used
// Cooridinate system is as follows: (module facing downwards)
// --INVALID INVALID INVALID--
// -------- Module -----------
// +INF,0     0|0       -INF,0
// ...........................
// ........___________........
// ..Start|           |........
// .......|   Region  |.......
// .......|___________|End.....
// ...........................
// +INF,+INF  0|INF   -INF,+INF
typedef struct _ZoneVertex {
  // Distance horizontally from center of module in millimeters
  // Negative denotes located to left of module
  // Positive denotes located to right of module
  int X;
  // Distance from front of module in millimeters
  // Range [0, INF]
  // Negative is invalid according to data sheet
  int Y;
} ZoneVertex;

typedef struct _ZoneRegion{
  ZoneVertex Start;
  ZoneVertex End;
} ZoneRegion;

typedef struct _ZoneFilteringConfig{
  ZoneFilteringType Type;
  ZoneRegion Zone1;
  ZoneRegion Zone2;
  ZoneRegion Zone3;
} ZoneConfiguration;

typedef struct TrackedObject{
  // Distance horizontally from center of module in millimeters
  // Negative denotes located to left of module
  // Positive denotes located to right of module
  int X;
  // Distance from front of module in millimeters
  // Range [0, INF]
  // Negative is invalid according to data sheet
  int Y;
  // In cm/s centimeters per second
  int Speed;
  // individual distance gate size in mm? Whatever that means
  int DistanceResolution;
} TrackedObject;

typedef struct TrackedObjectGroup{
  struct TrackedObject First;
  struct TrackedObject Second;
  struct TrackedObject Third;
} TrackedObjectGroup;

inline static void SendCommand(const struct Command* command);
inline static void WaitForCommand(struct Command* expectedAndOut, bool allowMalformed, size_t timeout_ms, bool timeoutIsSuccess = false, bool resumeRadarIsSuccess = false);
inline static struct Command ReadCommand(unsigned int timeout_ms = UINT_MAX);
inline static void Command_EnableConfig();
inline static void Command_DisableConfigMode();
inline static void Command_SetSingleTargetTracking();
inline static void Command_SetMultiTargetTracking();
inline static unsigned int Command_ReadTrackingMode();
inline static void Command_SetBaudRate(AvailableBaudRates baud);
inline static ZoneConfiguration Command_GetZoneConfiguration();
inline static MacAddress Command_GetMacAddress();
inline static void Command_SetEnableBluetooth(bool enabled);
inline static void Command_ResetToFactorySettings();
inline static void Command_RestartModule();

inline static void InitRadarOnSerial1()
{
  // Start Serial with radar
  // The default baud rate of the radar serial port is 256000, 1 stop bit, no parity bit.
  Serial1.begin(256000);

  delay(3000);

  Command_EnableConfigMode();
  //Command_ReadTrackingMode();
  //Command_SetSingleTargetTracking();
  //Command_ReadTrackingMode();
  //Command_SetMultiTargetTracking();
  //Command_ReadTrackingMode();
  //Command_GetMacAddress();
  //Command_SetEnableBluetooth(false);
  //Command_ResetToFactorySettings();
  //Command_RestartModule();
  //Command_SetBaudRate(AvailableBaudRates::b9600);
  Command_DisableConfigMode();
}


inline static struct Command SendCommandAndWaitForACK(struct Command* command, bool radarResumeIsSuccess = false)
{
  SendCommand(command);
  
  Command expectedAndResponse = {
    .Word = {0xFD, 0xFC}, // ACK RESPONSE
    .Size = 0,
    .Values = {0x00},
    .Malformed = false,
    .TimedOut = false
  };

  const bool allowMalformed = true;
  const unsigned int timeout_ms = 50;
  const bool timeoutIsSuccess = true;

  WaitForCommand( &expectedAndResponse, 
                  allowMalformed, 
                  timeout_ms, 
                  timeoutIsSuccess, 
                  radarResumeIsSuccess );

  return expectedAndResponse;
}

inline static void Command_EnableConfigMode()
{
  const Command EnableConfiguration = {
    .Word = {0xFF, 0x00},
    .Size = 2,
    .Values = {0x01, 0x00},
    .Malformed = false,
    .TimedOut = false
  };

  log("Requesting Configuration Mode\n");
  SendCommandAndWaitForACK(&EnableConfiguration);
  log("\n Configuration Mode Enabled, Waiting for configuration command\n");
}

inline static void Command_DisableConfigMode()
{
  const Command DisableConfiguration = {
    .Word = {0xFE, 0x00},
    .Size = 0,
    .Values = {0x00, 0x00},
    .Malformed = false,
    .TimedOut = false
  };

  log("Exiting Configuration Mode\n");
  // Return Value: 2 byte ACK status (0 success, 1 failure)
  const bool resumeRadarIsSuccess = true;
  Command result = SendCommandAndWaitForACK(&DisableConfiguration, resumeRadarIsSuccess);

  if(result.Values[4] != 0x0)
  {
    log("Failed to exit config mode, retrying\n");
    Command_DisableConfigMode();
  }

  log("\n Configuration Mode Disabled, Resuming RADAR operation\n");
}

inline static void Command_SetSingleTargetTracking()
{
  const Command SetSingleTargetTracking = {
    .Word = {0x80, 0x00},
    .Size = 0,
    .Values = {0x00, 0x00},
    .Malformed = false,
    .TimedOut = false
  };

  log("Setting Tracking mode to single target tracking\n");
  // Return Value: 2 byte ACK status (0 success, 1 failure)
  Command result = SendCommandAndWaitForACK(&SetSingleTargetTracking);

  if(result.Values[2] != 0x0 || result.Values[3] != 0x0)
  {
    log("Failed to set tracking mode, retrying\n");
    Command_SetSingleTargetTracking();
  }

  log("\n Successfully set mode to single target tracking, Resuming RADAR operation\n");
}

inline static void Command_SetMultiTargetTracking()
{
  const Command SetMultiTargetTracking = {
    .Word = {0x90, 0x00},
    .Size = 0,
    .Values = {0x00, 0x00},
    .Malformed = false,
    .TimedOut = false
  };

  log("Setting Tracking mode to multi target tracking\n");
  // Return Value: 2 byte ACK status (0 success, 1 failure)
  Command result = SendCommandAndWaitForACK(&SetMultiTargetTracking);

  if(result.Values[2] != 0x0 || result.Values[3] != 0x0)
  {
    log("Failed to set tracking mode, retrying\n");
    Command_SetMultiTargetTracking();
  }

  log("\n Successfully set mode to multi target tracking, Resuming RADAR operation\n");
}

// 0 is single target tracking
// 1 is multi target tracking
inline static unsigned int Command_ReadTrackingMode()
{
  const Command GetTrackingMode = {
    .Word = {0x91, 0x00},
    .Size = 0,
    .Values = {0x00, 0x00},
    .Malformed = false,
    .TimedOut = false
  };

  log("Reading Tracking Mode\n");
  // Return Value: 2 byte ACK status (0 success, 1 failure)
  Command result = SendCommandAndWaitForACK(&GetTrackingMode);

  if(result.Values[2] != 0x0 || result.Values[3] != 0x0)
  {
    log("Failed to read tracking mode, retrying\n");
    Command_ReadTrackingMode();
  }

  // Radar ACK(success):
  // FD FC FB FA 06 00 91 01 00 00 01 00 04 03 02 01
  // The return value 0x0001 means that it is currently in single-target tracking mode.
  // The return value of 0x0002 means that it is currently in multi-target tracking mode.
  const unsigned int trackingMode = result.Values[4];

  log("\n Current Tracking Mode: ");
  switch(trackingMode)
  {
    case 1:
      log("Single");
      break;
    case 2:
      log("Multi");
      break;
    default:
      log("ERROR");
      break;
  }
  log(", Resuming RADAR operation\n");

  return trackingMode;
}


// This command is used to set the baud rate of the serial port of the module, the configured value is not
// lost when power down, and the configured value takes effect after restarting the module.
inline static void Command_SetBaudRate(AvailableBaudRates baud)
{
  const Command SetBaudRateCommand = {
    .Word = {0xA1, 0x00},
    .Size = 2,
    .Values = {(unsigned int)baud, 0x00},
    .Malformed = false,
    .TimedOut = false
  };

  log("Setting Baud Rate, setting does not apply till restart of module\n");
  // Return Value: 2 byte ACK status (0 success, 1 failure)
  Command result = SendCommandAndWaitForACK(&SetBaudRateCommand);

  if(result.Values[2] != 0x0 || result.Values[3] != 0x0)
  {
    log("Failed to set baud rate, retrying\n");
    Command_SetBaudRate(baud);
  }

  log("\n Successfully set baud rate, Resuming RADAR operation\n");
}

// This command is used to restore all configuration values to unfactory values, and the configuration
// values take effect after rebooting the module.
inline static void Command_ResetToFactorySettings()
{
  const Command ResetCommand = {
    .Word = {0xA2, 0x00},
    .Size = 0,
    .Values = {0x00, 0x00},
    .Malformed = false,
    .TimedOut = false
  };

  log("Resetting module to factory settings, does not apply till module has been restarted\n");
  // Return Value: 2 byte ACK status (0 success, 1 failure)
  Command result = SendCommandAndWaitForACK(&ResetCommand);

  if(result.Values[2] != 0x0 || result.Values[3] != 0x0)
  {
    log("Failed to reset to factory settings, retrying\n");
    Command_ResetToFactorySettings();
  }

  log("\n Successfully set mode to multi target tracking, Resuming RADAR operation\n");
}

inline static void Command_RestartModule()
{
  const Command RestartModule = {
    .Word = {0xA3, 0x00},
    .Size = 0,
    .Values = {0x00, 0x00},
    .Malformed = false,
    .TimedOut = false
  };

  log("Restarting module\n");
  // Return Value: 2 byte ACK status (0 success, 1 failure)
  Command result = SendCommandAndWaitForACK(&RestartModule);

  if(result.Values[2] != 0x0 || result.Values[3] != 0x0)
  {
    log("Failed to restart module, retrying\n");
    Command_RestartModule();
  }

  log("\n Successfully Restarted Module, Module will resume RADAR operation on restart\n");
}

// This command is used to control the Bluetooth on or off, the Bluetooth function of the module is on
// by default. The configured value is not lost when power down, and the configured value takes effect
// after restarting the module.
inline static void Command_SetEnableBluetooth(bool enabled)
{
  const Command EnableOrDisableBluetooth = {
    .Word = {0xA4, 0x00},
    .Size = 0,
    .Values = {enabled ? 0x01 : 0x00, 0x00}, // 0x0100 turn on bluetooth 0x0000 turn off bluetooth (little endian)
    .Malformed = false,
    .TimedOut = false
  };

  log(enabled ? "Enabling " : "Disabling ");
  log("Bluetooth \n");
  // Return Value: 2 byte ACK status (0 success, 1 failure)
  Command result = SendCommandAndWaitForACK(&EnableOrDisableBluetooth);

  if(result.Values[2] != 0x0 || result.Values[3] != 0x0)
  {
    log("Failed to set bluetooth mode, retrying\n");
    Command_SetEnableBluetooth(enabled);
  }

  log("\n Successfully ");
  log(enabled ? "Enabled " : "Disabled ");
  log(" the Bluetooth Module, the new value takes effect after restarting the module.\n");
}

inline static MacAddress Command_GetMacAddress()
{
  const Command GetMacAddress = {
    .Word = {0xA5, 0x00},
    .Size = 2,
    .Values = {0x01, 0x00},
    .Malformed = false,
    .TimedOut = false
  };

  log("Getting MAC Address \n");
  // Return Value: 2 byte ACK status (0 success, 1 failure)
  Command response = SendCommandAndWaitForACK(&GetMacAddress);

  if(response.Values[2] != 0x0 || response.Values[3] != 0x0)
  {
    log("Failed to get MAC, retrying\n");
    return Command_GetMacAddress();
  }

  MacAddress result;

  log("Current MAC Address: ");
  for(int i = 4; i < 9; i++)
  {
    unsigned int c = response.Values[i];
    log(c);
    log(' ');
    result.Bytes[i-4] = c;
  }

  endline();

  return result;
}

inline static ZoneConfiguration Command_GetZoneConfiguration()
{
  log("Command_GetZoneConfiguration Not Implemented\n");
  return {};
  const Command GetZoneConfig = {
    .Word = {0xC1, 0x00},
    .Size = 0,
    .Values = {0x00, 0x00},
    .Malformed = false,
    .TimedOut = false
  };

  log("Getting Zone Configuration \n");
  // Return Value: 2 byte ACK status (0 success, 1 failure)
  Command response = SendCommandAndWaitForACK(&GetZoneConfig);

  if(response.Values[2] != 0x0 || response.Values[3] != 0x0)
  {
    log("Failed to get zone configuration, retrying\n");
    return Command_GetZoneConfiguration();
  }

  ZoneConfiguration result;
  // HEADER      | size  | comm  | null  | type  | zone1 xy/xy               | zone2 xy/xy   | zone3 xy/xy
  // FD FC FB FA | 1E 00 | C1 01 | 00 00 | 01 00 | E803 E803 18FC 8813 | 0000 0000 0000 0000 | 0000 0000 0000 0000 | EOF
  // ............|.......| 0  1  | 2  3  | 4  5  | 6
  result.Type = (ZoneFilteringType)response.Values[4];

  //result.Zone1.Start.X = response.Values[4]


  return result;
}

inline static void SendCommand(const struct Command* command)
{
  if(command->Size >= 32)
  {
    Serial.print("Attempted to send command larger than this program supports: ");
    Serial.print(command->Size);
    Serial.print('\n');
    return;
  }

  log("Sending: ");

  //Header       In-frame data length In-frame data End of frame
  // FD FC FB FA 2 bytes              See Table 3   04 03 02 01

  // send header
  write_char(0xFD);
  write_char(0xFC);
  write_char(0xFB);
  write_char(0xFA);

  // Send Data Size
  write_char(command->Size + 2);
  write_char(0x00);
  
  write_char(command->Word[0]);
  write_char(command->Word[1]);

  for(uint16_t i = 0; i < command->Size; i++)
  {
    write_char(command->Values[i]);
  }

  // Send End Of Frame
  write_char(0x04);
  write_char(0x03);
  write_char(0x02);
  write_char(0x01);

  endline();
}

// Mutates expectedAndOut's Value array with the response data (expectedAndOut->Values)
inline static void WaitForCommand(struct Command* expectedAndOut, bool allowMalformed, size_t timeout_ms, bool timeoutIsSuccess = false, bool radarResumeIsSuccess = false)
{
  struct Command result = ReadCommand(timeout_ms);
  
  if(result.Word[0] == 0xAA && result.Word[1] == 0xFF && radarResumeIsSuccess)
  {
    // Mimic valid response since the radar decides to just resume the radar instead of sending valid data
    expectedAndOut->Size = 4;
    expectedAndOut->Values[0] = expectedAndOut->Word[0];
    expectedAndOut->Values[1] = 0x01;
    expectedAndOut->Values[2] = 0x0;
    expectedAndOut->Values[3] = 0x0;
    expectedAndOut->Values[4] = 0x0;
    return;
  }

  const bool correctResponse = result.Word[0] == expectedAndOut->Word[0] && result.Word[1] == expectedAndOut->Word[1];

  if(result.TimedOut)
  {
    log("T/O\n");

    if(timeoutIsSuccess == false)
    {
      return WaitForCommand(expectedAndOut, allowMalformed, timeout_ms, timeoutIsSuccess, radarResumeIsSuccess);
    }

    // Mimic valid response since the radar decides to just resume the radar instead of sending valid data
    expectedAndOut->Size = 4;
    expectedAndOut->Values[0] = expectedAndOut->Word[0];
    expectedAndOut->Values[1] = 0x01;
    expectedAndOut->Values[2] = 0x0;
    expectedAndOut->Values[3] = 0x0;
    expectedAndOut->Values[4] = 0x0;

    return result;
  }

  if(result.Malformed && !allowMalformed)
  {
    log("MLF\n");
    return WaitForCommand(expectedAndOut, allowMalformed, timeout_ms, timeoutIsSuccess, radarResumeIsSuccess);
  }

  if(!correctResponse)
  {
    log("WRG\n");
    return WaitForCommand(expectedAndOut, allowMalformed, timeout_ms, timeoutIsSuccess, radarResumeIsSuccess);
  }

  log("Received successful response: ");
  log(result.Word[0],HEX);
  log(' ');
  log(result.Word[1],HEX);
  log("  ");
  for(int i = 0; i < 31; ++i)
  {
    const unsigned int c = result.Values[i];
    log(c);
    log(' ');
    expectedAndOut->Values[i] = c;
  }
  endline();
}

inline static struct Command ReadCommand(unsigned int timeout_ms = UINT_MAX)
{
  const uint16_t HeaderStartACK = 0xFD;
  const uint16_t HeaderStartRadar = 0xAA;

  uint16_t __temporary_value;
  #define enforce_read(expectedValue) __temporary_value = read_char(); if(__temporary_value != expectedValue){ if(__temporary_value == HeaderStartRadar){goto radar;} if(__temporary_value == HeaderStartACK){goto ack;}log("Expected: ");log(expectedValue, HEX);log(" Got: ");log(__temporary_value,HEX); result.Malformed = true; return result;}
  
  struct Command result = {
    .Word = {0x00, 0x00},
    .Size = 0,
    .Values = {0x00},
    .Malformed = false,
    .TimedOut = false
  };

  // put your main code here, to run repeatedly:
  unsigned int count = 0;
  while(!Serial1.available()){
    delay(1);
    log(".");
    if(count++>timeout_ms){
      // technically its not malformed but we dont want to accidentally use the command
      // if it timed out
      result.Malformed = true;
      result.TimedOut = true;
      return result;
    }
  }

  log("Received: ");
  uint16_t byte = read_char();

  // configuration header:
  // FD FC FB FA
  if(byte == 0xFD)
  {
// GOTO justification:
// Since we have no clock signal with UART Serial
// And we have no control over communication timing
// Theres a chance we send a command and receive the response in the middle of a different
// reponse where the radar might begin sending radar position data but stop to send an ACK
// So all enforce reads check failures against start headers and will jump to the corresponding
// goto tags as needed this could be done through normal method
// design but for some reason adding a stack frame maes it so we miss data even
// though it should be buffered for us
ack:
    result.Word[0] = 0xFD;
    result.Word[1] = 0xFC;

    // throw away next 3 bytes
    enforce_read(0xFC);
    enforce_read(0xFB);
    enforce_read(0xFA);

    result.Size = read_char();
    // throw away second byte of size we are little endian dont care
    // because we're not likely to receive message greater than 32 bytes
    read_char();

    for(int i = 0; i < result.Size; ++i)
    {
      result.Values[i] = read_char();
    }

    // ensure we read the data correctly by verifying start of End of Frame
    // EOF is 04 03 02 01
    enforce_read(0x04);
    enforce_read(0x03);
    enforce_read(0x02);
    enforce_read(0x01);

  }else if(byte == 0xAA)
  {
radar:
      // HEADER
      // AA FF 03 00
      result.Word[0] = 0xAA;
      result.Word[1] = 0xFF;

      // throw away next 3 bytes
      enforce_read(0xFF);
      enforce_read(0x03);
      enforce_read(0x00);

      result.Size = 24;
      for(int i = 0; i < 24;i++)
      {
        // temporarily throw away bytes
        result.Values[i] = read_char();
      }

      // End of Frame
      // 55 CC
      enforce_read(0x55);
      enforce_read(0xCC);
      endline();
  }else
  {
    result.Word[0] = byte;

    // log("Received Unexpected byte");
    // log(byte, HEX);

    result.Malformed = true;
  }

  endline();

  return result;
}

// DANGER Assumes array of 8 bytes for speed, no bound checking
inline static struct TrackedObject GetTrackedObjectFromBytes(unsigned int* bytes);
inline static struct TrackedObject GetTrackedObjectFromBytes(unsigned int* bytes)
{
  TrackedObject result;

  result.X = (bytes[0] + bytes[1]) * 256;
  result.Y = (bytes[2] + bytes[3]) * 256;
  result.Speed = (bytes[4] + bytes[5]) * 256;
  result.DistanceResolution = (bytes[6] + bytes[7]) * 256;

  return result;
}
inline static struct TrackedObjectGroup GetTrackedObjects();
inline static struct TrackedObjectGroup GetTrackedObjects()
{
  struct Command response = ReadCommand();

  TrackedObjectGroup result;

  result.First = GetTrackedObjectFromBytes(response.Values + 0);
  result.Second = GetTrackedObjectFromBytes(response.Values + 8);
  result.Third = GetTrackedObjectFromBytes(response.Values + 16);

  return result;
}

inline static  void LogTrackedObject(struct TrackedObject* object);
inline static  void LogTrackedObject(struct TrackedObject* object)
{
  log("{ X: ");
  log(object->X, DEC);
  log("mm Y: ");
  log(object->Y, DEC);
  log("mm Speed: ");
  log(object->Speed, DEC);
  log("cm/s Resolution: ");
  log(object->DistanceResolution, DEC);
  log("cm }");
}

inline static  bool EmptyGroup(struct TrackedObjectGroup* group);
inline static  bool EmptyGroup(struct TrackedObjectGroup* group)
{
  char* converted = (char*)(void*)group;
  for(int i = 0; i < sizeof(struct TrackedObjectGroup);i++)
  {
    if(converted[i] != 0)
    {
      return false;
    }
  }
  return true;
}

inline static  void LogTrackedObjectGroup(struct TrackedObjectGroup* group);
inline static  void LogTrackedObjectGroup(struct TrackedObjectGroup* group)
{
  if(EmptyGroup(group))
  {
    return;
  }
  log("{ First: ");
  LogTrackedObject(&group->First);
  log(" Second: ");
  LogTrackedObject(&group->Second);
  log(" Third: ");
  LogTrackedObject(&group->Third);
  log("}");
}

// EXAMPLE
// void loop()
// {
//   struct Command command = ReadCommand();

//   //         00001110 (0E)
//   // 00000011 (03)
//   // 0000001100001110 (0E03) 782
//   // int value = (  0x03 << 8) | 0x0E;
//   // Serial.print(value,DEC);
//   // Zero leading bit corresponds to negative number

//   uint16_t x = (command.Values[1] << 8) | command.Values[0];
//   int y = ((command.Values[3] << 8) | command.Values[2]) - 32768;
//   int speed = 0 - ((command.Values[5] << 8) | command.Values[4]);

//   bool invalidCoords = (x == 0 && y == 0) || (x == -1 || y == -1 || speed == -1);

//   if(invalidCoords)
//   {
//     return;
//   }
  
//   Serial.print(-1000, DEC);
//   Serial.print(' ');
//   Serial.print(1000, DEC);
//   Serial.print(' ');

//   Serial.print(x, DEC);
//   Serial.print(' ');
//   Serial.print(y, DEC);
//   Serial.print(' ');
//   Serial.println(speed, DEC);
// }

// remove convenience stuff so we don't pollute other peoples stuff
#undef write_char(_uint16_tValue)
#undef read_char()
#undef endline() 
#undef log(...)

#endif
