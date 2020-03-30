







#include <SPI.h>
#include <mcp2515.h>

#define CAN_ID 0x600      // CAN id of the board
#define CAN_ID_RASP 0x701 //CAN id of the ras-pi
#define TRIGGER_PIN 9
const unsigned long MEASURE_TIMEOUT = 5000UL; //25 ms 8m
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


  cli();//stop interrupts

  //set timer1 interrupt at 1Hz
  TCCR1A = 0;// set entire TCCR1A register to 0
  TCCR1B = 0;// same for TCCR1B
  TCNT1  = 0;//initialize counter value to 0
  // set compare match register for 1hz increments
  OCR1A = 15624;// = (16*10^6) / (1*1024) - 1 (must be <65536)
  // turn on CTC mode
  TCCR1B |= (1 << WGM12);
  // Set CS12 and CS10 bits for 1024 prescaler
  TCCR1B |= (1 << CS12) | (1 << CS10);
  // enable timer compare interrupt
  TIMSK1 |= (1 << OCIE1A);

  sei();//allow interrupts

}

void loop()
{

  if (mcp2515.readMessage(&canMsg) == MCP2515::ERROR_OK) {


    if (canMsg.can_id == 0x600)
    {
      Serial.print("Recieved message \r\n");
      for (int i = 0; i < num; i++) {
        canMsg.data[i] = (uint8_t)data[i];

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
  float distance_mm =  measure / 2.0 * SOUND_SPEED;


  return measure;


}



ISR(TIMER1_COMPA_vect) { //timer1 interrupt 1Hz toggles pin 13 (LED)
  //generates pulse wave of frequency 1Hz/2 = 0.5kHz (takes two cycles for full wave- toggle high then toggle low)
  for (int i = 0; i < num; i++) {
    data[i] = mesure(pin[i]) / 10;
  }



  Serial.print(data[0]);
  Serial.print(" ");
  Serial.print(data[1]);
  Serial.print(" ");
  Serial.print(data[2]);
  Serial.print(" ");
  Serial.print(data[3]);
  Serial.print(" ");
  Serial.print(data[4]);
  Serial.print("\r\n");
}
