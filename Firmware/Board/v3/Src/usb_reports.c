#include "config.h"
#include "usb_reports.h"

void JoystickInputReport_Init(JoystickInputReport *hid_report) {
  hid_report->id = JOYSTICK_INPUT_REPORT_ID;
  hid_report->steering = 0x0;
  hid_report->accelerator = 0x8000;
  hid_report->brake = 0x8000;
  hid_report->buttons = 0x0;
}

void PIDPoolFeatureReport_Init(PID_PoolFeatureReport* report) {
    report->id = PID_POOL_FEATURE_REPORT_ID;
    report->ramPoolSize = MEMORY_POOL_SIZE;
    report->maxEffects = MAX_EFFECTS;
    report->deviceManagedPool = PID_DEVICE_MANAGED_POOL;
    report->sharedParameterBlocks = PID_SHARED_PARAMETER_BLOCKS;
}

void PIDStateReport_Init(PIDStateReport *report) {
    report->id = PID_STATE_REPORT_ID;
    report->devicePaused = 0;
    report->actuatorsEnabled = 1;
    report->actuatorPower = 1;
    report->actuatorOverrideSwitch = 1;
    report->effectPlaying = 1;
    report->effectBlockIndex = 1;
}

