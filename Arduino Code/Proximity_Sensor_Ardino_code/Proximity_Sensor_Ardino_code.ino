#include <HCSR04.h>

#include <SPI.h>
#include <mcp2515.h>

#define CAN_ID 0x600      // CAN id of the board
#define CAN_ID_RASP 0x701 //CAN id of the ras-pi

int pin[5] = {4, 8, 7, 6, 5};

MCP2515 mcp2515(10);
HCSR04 HCSR04(9, pin, 5);

bool interrupt = false;
struct can_frame canMsg;
int data[5]={3,3,3,3,3};
int num = 5;

void irqHandler()
{
  interrupt = true;
}

void setup()
{
  Serial.begin(115200);
  SPI.begin();

  mcp2515.reset();
  mcp2515.setBitrate(CAN_500KBPS, MCP_8MHZ);
  mcp2515.setNormalMode();
  attachInterrupt(0, irqHandler, FALLING);
}

void loop()
{

 /* for (int i = 0; i < num; i++)
  {
    data[i] = HCSR04.dist(i);
    Serial.print(data[i]);
    Serial.print("  ");
    Serial.println(i);
  }*/
 // Serial.print("ylooooo\r\n");
  if (interrupt)
  {
    interrupt = false;

    uint8_t irq = mcp2515.getInterrupts();

    if (irq & MCP2515::CANINTF_RX0IF)
    {
      if (mcp2515.readMessage(MCP2515::RXB0, &canMsg) == MCP2515::ERROR_OK)
      {
        if (canMsg.can_id == 600)
        {
          Serial.print("Recived message \r\n");
          
          canMsg.can_id = CAN_ID_RASP;
          canMsg.can_dlc = 5;
          canMsg.data[0] = data[0];
          canMsg.data[1] = data[1];
          canMsg.data[2] = data[2];
          canMsg.data[3] = data[3];
          canMsg.data[4] = data[4];
          mcp2515.sendMessage(&canMsg);
        }
      }
    }

    if (irq & MCP2515::CANINTF_RX1IF)
    {
      if (mcp2515.readMessage(MCP2515::RXB1, &canMsg) == MCP2515::ERROR_OK)
      {
        if (canMsg.can_id == 600)
        {
          Serial.print("Recived message \r\n");
          canMsg.can_id = CAN_ID_RASP;
          canMsg.can_dlc = 5;
          canMsg.data[0] = data[0];
          canMsg.data[1] = data[1];
          canMsg.data[2] = data[2];
          canMsg.data[3] = data[3];
          canMsg.data[4] = data[4];
          mcp2515.sendMessage(&canMsg);
        }
      }
    }
  } 
}
