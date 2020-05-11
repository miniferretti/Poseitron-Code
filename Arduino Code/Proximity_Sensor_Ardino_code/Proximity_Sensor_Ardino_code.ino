
#include <SPI.h>
#include <mcp_can.h>

#define CAN_ID 0x600      // CAN id of the board
#define CAN_ID_RASP 0x701 //CAN id of the ras-pi
#define TRIGGER_PIN 9
const unsigned long MEASURE_TIMEOUT = 25000UL; //25 ms 8m
const float SOUND_SPEED = 340.0 / 1000;

int pin[5] = {4, 8, 7, 6, 5};

#define CAN0_INT 2 // Set INT to pin 2
MCP_CAN CAN0(10);  // Set CS to pin 10

//struct can_frame canMsg;
byte data[5] = {255, 255, 255, 255, 255};
byte dataprev[5] = {255, 255, 255, 255, 255};
int num = 5;
int Flag_Recv = 0;

INT32U *id;
INT8U *ext;
byte len;
byte buf[3];

void setup()
{
  Serial.begin(115200);
  SPI.begin();

  if (CAN0.begin(MCP_STDEXT, CAN_500KBPS, MCP_16MHZ) == CAN_OK)
    Serial.println("MCP2515 Initialized Successfully!");
  else
    Serial.println("Error Initializing MCP2515...");

  pinMode(2, INPUT);

  CAN0.init_Mask(0, 0, 0x1FFFFFFF);
  CAN0.init_Filt(0, 0, 0x00000600);
  CAN0.setMode(MCP_NORMAL);

  //attachInterrupt(0, MCP2515_ISR, FALLING);

  pinMode(TRIGGER_PIN, OUTPUT);
  digitalWrite(TRIGGER_PIN, LOW); // La broche TRIGGER doit être à LOW au repos

  for (int i = 0; i < num; i++)
  {
    pinMode(pin[i], INPUT);
  }
}

void MCP2515_ISR()
{
  Flag_Recv = 1;
  // Flag_Recv_Ctr = Flag_Recv_Ctr + 1; // debugging only
}

void loop()
{

  byte sndStat = 0;
  if (!digitalRead(2))
  {
    Flag_Recv = 0;
    CAN0.readMsgBuf(id, &len, buf);
    sndStat = CAN0.sendMsgBuf(CAN_ID_RASP, 0, 5, data);
    Serial.println("Envois du Message");
  }
  else
  {
    for (int i = 0; i < 5; i++)
    {
      data[i] = mesure(pin[i]);
      Serial.println(data[i]);
    }
  }
}

byte mesure(int pina)
{

  float duration, distance;
  digitalWrite(TRIGGER_PIN, LOW);
  delayMicroseconds(2);

  digitalWrite(TRIGGER_PIN, HIGH);
  delayMicroseconds(10);
  digitalWrite(TRIGGER_PIN, LOW);

  duration = pulseIn(pina, HIGH, MEASURE_TIMEOUT);
  distance = (duration / 2) * 0.0344;

  return (byte)distance;
}
