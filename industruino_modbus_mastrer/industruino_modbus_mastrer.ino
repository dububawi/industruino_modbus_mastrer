
/*
  wind speed sensor, anemometer on INDUSTRUINO IND.I/O topboard 1286
  RS485 connection with wind sensor, Modbus protocol
  Industruino = Modbus Master
  Wind sensor = Modbus Slave ID: 2
  read holding register 0
  divide by 10 for wind speed in m/s

  more info on the SimpleModbusMaster library: https://drive.google.com/folderview?id=0B0B286tJkafVYnBhNGo4N3poQ2c&usp=drive_web&tid=0B0B286tJkafVSENVcU1RQVBfSzg#list

*/

#include <SimpleModbusMaster.h>
#include "U8glib.h"
U8GLIB_MINI12864 u8g(21, 20, 19, 22);    // SPI Com: SCK = 21, MOSI = 20, CS = 19, A0 = 22
//////////////////// Port information ///////////////////
#define baud 9600                                                                                  // SENSOR SPEC
#define timeout 1000
#define polling 200 // the scan rate
#define retry_count 10
// used to toggle the receive/transmit pin on the driver
#define TxEnablePin 9                                                                           // INDUSTRUINO RS485
// The total amount of available memory on the master to store data
#define TOTAL_NO_OF_REGISTERS 1                                                // SENSOR SPEC
// sensor sends integer of wind speed * 10 (m/s)
// This is the easiest way to create new packets
// Add as many as you want. TOTAL_NO_OF_PACKETS
// is automatically updated.
enum
{
  PACKET1,                                          // only need 1 type of operation: read wind sensor
  TOTAL_NO_OF_PACKETS // leave this last entry
};
// Create an array of Packets to be configured
Packet packets[TOTAL_NO_OF_PACKETS];
// Masters register array
unsigned int regs[TOTAL_NO_OF_REGISTERS];
void setup()
{
  // Initialize each packet: packet, slave-id, function, start of slave index, number of regs, start of master index
  // READ_HOLDING_REGISTERS (no info in input_registers)                        // SENSOR SPEC
  // slave-id: 2                                                                                               // SENSOR SPEC
  // number of registers to read: only 1 = speed                                       // SENSOR SPEC
  modbus_construct(&packets[PACKET1], 2, READ_HOLDING_REGISTERS, 0, 1, 0);

  // Initialize the Modbus Finite State Machine
  // default SERIAL_8N2 -- wind sensor brochure mentions n,8,1 = 8N1??
  modbus_configure(&Serial1, baud, SERIAL_8N2, timeout, polling, retry_count, TxEnablePin, packets, TOTAL_NO_OF_PACKETS, regs);
  // Serial1 = INDUSTRUINO RS485
  Serial.begin(9600);
  u8g.begin();
  u8g.setRot180();
  pinMode(26, OUTPUT);
  analogWrite(26, 100);
}
void loop()
{
  modbus_update();                                                             // send 1 simple Master request to Slave, as defined above

  float wind_speed = regs[0] / 10.0;

  Serial.print("wind speed (m/s): ");
  Serial.println(wind_speed);

  u8g.firstPage();
  do {
    u8g.setFont(u8g_font_fub20n);
    u8g.setPrintPos(25, 55);
    u8g.print(wind_speed, 1);
    u8g.setFont(u8g_font_unifont);
    u8g.print(" m/s ");
    u8g.setPrintPos(20, 15);
    u8g.print("INDUSTRUINO");
    u8g.setPrintPos(20, 30);
    u8g.print("Modbus RTU");

  } while ( u8g.nextPage() );

  delay(3000);
}

