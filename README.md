# HLK_LD2450
Arduino C Implementation for the RADAR position module: HLK_LD2450

Available Methods:
```c
void SendCommand(const struct Command* command);
void WaitForCommand(struct Command* expectedAndOut, bool allowMalformed, size_t timeout_ms, bool timeoutIsSuccess = false, bool resumeRadarIsSuccess = false);
struct Command ReadCommand(unsigned int timeout_ms = UINT_MAX);
void Command_EnableConfig();
void Command_DisableConfigMode();
void Command_SetSingleTargetTracking();
void Command_SetMultiTargetTracking();
unsigned int Command_ReadTrackingMode();
void Command_SetBaudRate(AvailableBaudRates baud);
ZoneConfiguration Command_GetZoneConfiguration();
MacAddress Command_GetMacAddress();
void Command_SetEnableBluetooth(bool enabled);
void Command_ResetToFactorySettings();
void Command_RestartModule();
```

Usage:
```c
struct Command command = ReadCommand();

uint16_t x = (command.Values[1] << 8) | command.Values[0];
int y = ((command.Values[3] << 8) | command.Values[2]) - 32768;
int speed = 0 - ((command.Values[5] << 8) | command.Values[4]);

// OR 
// MUCH SLOWER ON LOW END HARDWARE
struct TrackedObject object = GetTrackedObjectFromBytes(command.Values);
```

Note: I wrote this is like 8 hours to rush to get the module to work, i make no garuntees this is fast or correct but it should work fine for whatever ur using it for