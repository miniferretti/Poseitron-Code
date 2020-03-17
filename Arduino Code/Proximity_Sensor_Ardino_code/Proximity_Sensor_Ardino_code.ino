







#include <SPI.h>
#include <mcp2515.h>

#define CAN_ID 0x600      // CAN id of the board
#define CAN_ID_RASP 0x701 //CAN id of the ras-pi
#define TRIGGER_PIN 9
const unsigned long MEASURE_TIMEOUT = 25000UL;
const float SOUND_SPEED = 340.0 / 1000;

int pin[5] = {4, 8, 7, 6, 5};

MCP2515 mcp2515(10);



bool interrupt = false;
struct can_frame canMsg;
long data[5] = {3, 3, 3, 3, 3};
int num = 5;


void setup()
{
  Serial.begin(115200);
  SPI.begin();

  mcp2515.reset();
  mcp2515.setBitrate(CAN_500KBPS, MCP_16MHZ);
  mcp2515.setNormalMode();

  pinMode(TRIGGER_PIN, OUTPUT);
  digitalWrite(TRIGGER_PIN, LOW); // La broche TRIGGER doit être à LOW au repos

  for (int i = 0; i < num; i++) {
    pinMode(pin[i], INPUT);
  }




}

void loop()
{


  if (mcp2515.readMessage(&canMsg) == MCP2515::ERROR_OK) {


    if (canMsg.can_id == 0x600)
    {
      Serial.print("Recieved message \r\n");
      for (int i = 0; i < num; i++) {
        canMsg.data[i] = mesure(pin[i]) / 10;
        // Serial.println(data[i]);
      }
      canMsg.can_id = CAN_ID_RASP;
      canMsg.can_dlc = 5;

      mcp2515.sendMessage(&canMsg);
    }


  }

}




float mesure(int pina)
{

  /* 1. Lance une mesure de distance en envoyant une impulsion HIGH de 10µs sur la broche TRIGGER */
  digitalWrite(TRIGGER_PIN, HIGH);
  delayMicroseconds(10);
  digitalWrite(TRIGGER_PIN, LOW);

  /* 2. Mesure le temps entre l'envoi de l'impulsion ultrasonique et son écho (si il existe) */
  long measure = pulseIn(pina, HIGH, MEASURE_TIMEOUT);

  /* 3. Calcul la distance à partir du temps mesuré */
  float distance_mm = measure / 2.0 * SOUND_SPEED;


  return measure;


}
