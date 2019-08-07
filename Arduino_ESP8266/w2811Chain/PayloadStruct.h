#define PingReplyCmd 65
#define TimeStampCmd 66
#define LightCtrlCmd 70
#define JSONDataCmd  71
#define SingleDataCmd 72


struct pingReply {
  uint32_t Timer;
};

struct payload_sensor_data {
  byte CmdType;
  byte SensorID;
  byte Dimension;
  float SensorValue;
};


typedef struct {
  byte CmdType; //0: Command; 1 : Info Request; 2 : Info Update; 3..255: reserved
  byte LightPattern;
  byte PrevPattern;
  byte InsideOut;
  byte SplitChain;   
  byte Brightness; //valid data 1..100
  byte FrameRate; //valid data for frame rate 1..255
  byte ShiftRate; //used for shifting patterns while frame rate is constant
  byte RGB[3];
  byte RandomSparks; //boolean
} payload_light_controller;


/*
struct payload_t {
  unsigned long ms;
  unsigned long counter;
};
*/
