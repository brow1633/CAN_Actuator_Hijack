#include <Arduino.h>
#include <ESP32CAN.h>
#include <CAN_config.h>

typedef enum {
  DEF_LEFT_CMD = 0xff0100,
	DEF_LEFT_RPT = 0xff0100,
	HIJ_LEFT_CMD = 0xff0400,
	HIJ_LEFT_RPT = 0xff0401,
	DEF_RIGHT_CMD = 0xff0200,
	DEF_RIGHT_RPT = 0xff0200,
	HIJ_RIGHT_CMD = 0xff0500,
	HIJ_RIGHT_RPT = 0xff0501
} act_id;

//Zero/Default/Idle position of actutors
const int zero_pos = 1800;

//ms between EPR messages
const int epr_time = 100;

// Have we been hijaked
bool hijacked = false;
// Can we be hijacked
// This can include many safety checks
// #1: Is default controller trying to send?
bool hijackable = false;
unsigned long hijackable_ctr = millis();
unsigned long time_since_epr = 0;

// Change actuators command ID
void changeCommandId(int cmdId, int newId);

// Change actuators report ID
void changeReportId(int cmdId, int newId);

void hijack(alt_id currCmdId, alt_id newCmdId, alt_id newRptId);

// Read position commands from mower
// Monitor, if change from 0 pos un-hijack
int readPosCommand(uint8_t msg[]);

// spoof's enhanced position report
// We need to check that mower faults
// if remote receives different value
void spoofEPR(int reportId, int val);

// forward CAN frame to different IDs
void forwardCAN(int newId, CAN_frame_t frame);

// process a cmd message from existing controller
void processDefaultCmd(int newCmdId, CAN_frame_t frame);

// process a cmd message from existing controller
void processHijackRpt(int defRptId, CAN_frame_t frame);

void setup() {
  Serial.begin(115200);
  Serial.println("starting CAN");
  CAN_cfg.speed = CAN_SPEED_250KBPS;
  CAN_cfg.tx_pin_id = GPIO_NUM_2;
  CAN_cfg.rx_pin_id = GPIO_NUM_27;
  CAN_cfg.rx_queue = xQueueCreate(10, sizeof(CAN_frame_t));
  // start CAN Module
  ESP32Can.CANInit();
}

void loop() {
  CAN_frame_t rx_frame;

  if (!hijackable) {
    if(millis() - hijackable_ctr > 5000) {
      hijackable = true;
    }
    if(hijacked) {
      //unhijack!
      hijacked = false;
    }
  }

  if(hijackable && !hijacked) {
    hijack(DEF_LEFT_CMD, HIJ_LEFT_CMD, HIJ_LEFT_RPT);
    hijack(DEF_RIGHT_CMD, HIJ_RIGHT_CMD, HIJ_RIGHT_RPT);
    hijacked = true;
  }

  if(hijacked && millis() - time_since_epr > epr_time) {
    time_since_epr = millis();
    spoofEPR(HIJ_LEFT_RPT, zero_pos);
    spoofEPR(HIJ_RIGHT_RPT, zero_pos);
  }

  if (xQueueReceive(CAN_cfg.rx_queue, &rx_frame, 3 * portTICK_PERIOD_MS) == pdTRUE)
  {
    if (rx_frame.FIR.B.RTR == CAN_RTR)
    {
    }
    else
    {
      act_id msgId = rx_frame.MsgId;
      switch(msgId) {
        case DEF_LEFT_CMD:
          processDefaultCmd(HIJ_LEFT_CMD, rx_frame);
          break;
        case DEF_RIGHT_CMD:
          processDefaultCmd(HIJ_RIGHT_CMD, rx_frame);
          break;
        case HIJ_LEFT_RPT:
          processHijackRpt(DEF_LEFT_RPT, rx_frame);
          break;
        case HIJ_RIGHT_RPT:
          processHijackRpt(DEF_RIGHT_RPT, rx_frame);
          break;
      }
    }
  }
}

void processDefaultCmd(int newCmdId, CAN_frame_t frame) {
  uint8_t* msg = frame.data.u8;
  switch(msg[0]) {
    case 0x0F:
      int pos = readPosCommand(rx_frame.data.u8);
      if(pos - zero_pos > 100) {
        hijackable = false;
        hijackable_ctr = millis();
      } 
      break;
    default:
      if(hijacked) {
        forwardCAN(newCmdId, frame);
      }
      break;

  }
}

void processHijackRpt(int defRptId, CAN_frame_t frame) {
  uint8_t* msg = frame.data.u8;

  switch(msg[0]) {
    case 0x98: // enhanced position report
      break;
    default:
      forwardCAN(defRptId, frame);
      break;
  }
}

void hijack(act_id currCmdId, act_id newCmdId, act_id newRptId) {
  if(!hijacked) {
    changeReportId(currCmdId, newRptId);
    changeCommandId(currCmdId, newCmdId);
  }
}

// Change actuators command ID
void changeCommandId(int cmdId, int newId) {
  CAN_frame_t frame;
  frame.MsgId = cmdId;
  frame.FIR.B.DLC = 8;
  frame.data.u8[0] = 0xF7; // Command ID Reassignment message type
  frame.data.u8[1] = 0x01; // Assuming we're changing the first User Defined Command ID
  frame.data.u32[1] = newId; // New ID
  frame.data.u8[6] = 0x00; // Unused
  frame.data.u8[7] = 0x00; // Enable Default Command ID
  ESP32Can.CANWriteFrame(&frame);
}

// Change actuators report ID
void changeReportId(int cmdId, int newId) {
  CAN_frame_t frame;
  frame.MsgId = cmdId;
  frame.FIR.B.DLC = 8;
  frame.data.u8[0] = 0xF7; // Report ID Reassignment message type
  frame.data.u8[1] = 0x00; // Assuming we're changing the User Defined Report ID
  frame.data.u32[1] = newId; // New ID
  frame.data.u8[6] = 0x00; // Unused
  frame.data.u8[7] = 0x01; // Select User Defined Report ID
  ESP32Can.CANWriteFrame(&frame);
}

// Read actuator position commands
int readPosCommand(uint8_t msg[]) {
  // The position is stored in bytes 2 and 3 of the message, with an offset of 500 (0.5")
  // The value is in 0.001" steps
  int pos = (msg[3] << 8) | msg[2];
  pos -= 500; // Remove offset
  return pos;
}

// spoof's enhanced position report
void spoofEPR(int reportId, int val) {
  CAN_frame_t frame;
  frame.MsgId = reportId;
  frame.FIR.B.DLC = 8;
  frame.data.u8[0] = 0x98; // Enhanced Position Report message type
  frame.data.u8[1] = 0x00; // Data Type set to 0
  // Convert inches to 0.001" steps and add offset
  int pos = val + 500;
  frame.data.u8[2] = pos & 0xFF; // Least significant byte
  frame.data.u8[3] = (pos >> 8) & 0xFF; // Most significant byte
  // Rest of the message
  frame.data.u8[4] = 0x00; // No errors
  frame.data.u16[3] = 0x0000; // No motor current
  frame.data.u8[7] = 0x00; // Status reserved for internal use
  ESP32Can.CANWriteFrame(&frame);
}
