

#include <SPI.h>
#include <mcp2515.h>

#define CAN_ID 0x600      // CAN id of the board
#define CAN_ID_RASP 0x701 //CAN id of the ras-pi
#define TRIGGER_PIN 9
const unsigned long MEASURE_TIMEOUT = 25000UL; //25 ms 8m
const float SOUND_SPEED = 340.0 / 1000;

int pin[5] = {4, 8, 7, 6, 5};

MCP2515 mcp2515(10);

struct can_frame canMsg;
long data[5] = {3, 3, 3, 3, 3};
long dataprev[5] = {255, 255, 255, 255, 255};
int num = 5;
unsigned long t;
unsigned long tprev;
float duration;
float distance;


void setup()
{
  Serial.begin(115200);
  SPI.begin();

  mcp2515.reset();
  mcp2515.setBitrate(CAN_500KBPS, MCP_16MHZ);
  mcp2515.setNormalMode();

  pinMode(TRIGGER_PIN, OUTPUT);
  digitalWrite(TRIGGER_PIN, LOW); // La broche TRIGGER doit être à LOW au repos

  for (int i = 0; i < num; i++)
  {
    pinMode(pin[i], INPUT);
  }


  t = millis();
  tprev = 0;
}

void loop()
{

  if (mcp2515.readMessage(&canMsg) == MCP2515::ERROR_OK)
  {

    if (canMsg.can_id == 0x600)
    {
      // Serial.print("Recieved message \r\n");
      if ((t - tprev) >= 100)
      {
        tprev = t;
        for (int i = 0; i < num; i++)
        {
          data[i] = mesure(pin[i]);
        }
      }

      for (int i = 0; i < num; i++)
      {
        if ((uint8_t)data[i] != 0)
        {
          canMsg.data[i] = (uint8_t)data[i];
          //dataprev[i] = data[i];
        } else {
          canMsg.data[i] = 255;
        }
      }
      canMsg.can_id = 0x701;
      canMsg.can_dlc = 5;
      Serial.print(canMsg.data[0]);
      Serial.print(" ");
      Serial.print(canMsg.data[1]);
      Serial.print(" ");
      Serial.print(canMsg.data[2]);
      Serial.print(" ");
      Serial.print(canMsg.data[3]);
      Serial.print(" ");
      Serial.print(canMsg.data[4]);
      Serial.print("\r\n");

      mcp2515.sendMessage(&canMsg);
    }
  }

  t = millis();
}




//float mesure(int pina)
//{
//
//  /* 1. Lance une mesure de distance en envoyant une impulsion HIGH de 10µs sur la broche TRIGGER */
//  digitalWrite(TRIGGER_PIN, HIGH);
//  delayMicroseconds(10);
//  digitalWrite(TRIGGER_PIN, LOW);
//
//  /* 2. Mesure le temps entre l'envoi de l'impulsion ultrasonique et son écho (si il existe) */
//  long measure = pulseIn(pina, HIGH, MEASURE_TIMEOUT);
//
//  /* 3. Calcul la distance à partir du temps mesuré */
//  float distance_mm = measure / 2.0 * SOUND_SPEED;
//
//  return measure;
//}



float mesure(int pina) {




float duration, distance;
  digitalWrite(TRIGGER_PIN, LOW); 
  delayMicroseconds(2);
 
  digitalWrite(TRIGGER_PIN, HIGH);
  delayMicroseconds(10);
  digitalWrite(TRIGGER_PIN, LOW);
  
  duration = pulseIn(pina, HIGH,MEASURE_TIMEOUT);
  distance = (duration / 2) * 0.0344;

  return distance;

}
