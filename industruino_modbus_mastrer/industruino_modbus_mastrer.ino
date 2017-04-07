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
#include <U8glib.h>
U8GLIB_MINI12864 u8g(21, 20, 19, 22);    // SPI Com: SCK = 21, MOSI = 20, CS = 19, A0 = 22
//////////////////// Port information ///////////////////
#define baud 9600                                                                                  // SENSOR SPEC
#define timeout 1000
#define polling 200 // the scan rate
#define retry_count 10
// used to toggle the receive/transmit pin on the driver
#define TxEnablePin 9                                                                           // INDUSTRUINO RS485
// The total amount of available memory on the master to store data

int16_t flow_rate_m3_h; /* 006C - 006D 108 - 109 Flow m3n/hr Floating point Read*/
int16_t pressure_bar; /* 009A - 009B 154 - 155 Pressure bar gauge Floating point Read */
int16_t temprature_c; /* 00CC - 00CD 204 - 205 Temperature *C Floating point Read */

uint8_t slave_ids[] = {2};
#define SLAVES_TOTAL_NO (sizeof(slave_ids) / sizeof(slave_ids[0]))

#define TOTAL_NO_OF_REGISTERS 8                                                // SENSOR SPEC
// sensor sends integer of wind speed * 10 (m/s)
// This is the easiest way to create new packets
// Add as many as you want. NO_OF_PACKETS_IN_SLAVE
// is automatically updated.

// Group nearby registers in packets   
// 10, 11 - packet 1
// 45, 46 - packet 2
// 78, 79 - packet 3
enum
{
  PACKET1,                                          // only need 1 type of operation: read wind sensor
  PACKET2,
  PACKET3,
  PACKET4,
  NO_OF_PACKETS_IN_SLAVE // leave this last entry
};

const uint8_t packet_start_register[NO_OF_PACKETS_IN_SLAVE] = {1, 0x006c, 0x009A, 0x00CC};
const uint8_t packet_size[NO_OF_PACKETS_IN_SLAVE] = {2, 2, 2, 2};

// Create an array of Packets to be configured
// Must be onedimetional array for modbus_configure()
Packet packets[NO_OF_PACKETS_IN_SLAVE * SLAVES_TOTAL_NO];
// Masters register array
unsigned int regs[SLAVES_TOTAL_NO][TOTAL_NO_OF_REGISTERS];

volatile uint8_t packet_addr_in_regs[SLAVES_TOTAL_NO][NO_OF_PACKETS_IN_SLAVE] = {0};

void setup()
{
  // Initialize each packet: packet, slave-id, function, start of slave index, number of regs, start of master index
  // READ_HOLDING_REGISTERS (no info in input_registers)                        // SENSOR SPEC
  // slave-id:                                                                  // SENSOR SPEC
  // number of registers to read: only 1 = speed                                // SENSOR SPEC

  uint8_t total_packet_size = 0;
  for (uint8_t i = 0; i < NO_OF_PACKETS_IN_SLAVE; i++)
  {
    total_packet_size += packet_size[i];
  }

//  static uint8_t packet_addr_in_regs[SLAVES_TOTAL_NO][NO_OF_PACKETS_IN_SLAVE] = {0};

  for (uint8_t slave = 0; slave < SLAVES_TOTAL_NO; slave++)
  {
    for (uint8_t packet = 0; packet < NO_OF_PACKETS_IN_SLAVE; packet++)
    {
      if (slave == 0 && packet == 0)
      {
        packet_addr_in_regs[slave][packet] = 0;  
      }
      else if (packet == 0)
      {
        packet_addr_in_regs[slave][packet] = packet_addr_in_regs[slave - 1][NO_OF_PACKETS_IN_SLAVE - 1] + packet_size[NO_OF_PACKETS_IN_SLAVE - 1];
      }
      else
      {
        packet_addr_in_regs[slave][packet] = packet_addr_in_regs[slave][packet - 1] + packet_size[packet];
      }
    }
  }

  uint8_t current_packet = 0;
  for (uint8_t slave = 0; slave < SLAVES_TOTAL_NO; slave++)
  {
    for (uint8_t packet = 0; packet < NO_OF_PACKETS_IN_SLAVE; packet++)
    {
      modbus_construct(&packets[current_packet], 
                        slave_ids[slave], 
                        READ_HOLDING_REGISTERS, 
                        packet_start_register[packet], 
                        packet_size[packet], 
                        packet_addr_in_regs[slave][packet]);

       current_packet++;
    }
  }

  // Initialize the Modbus Finite State Machine
  // default SERIAL_8N2 -- wind sensor brochure mentions n,8,1 = 8N1??
  /* For arduino industrial -- Serial1
     for test on arduino atmga328p -- Serial */
  modbus_configure(&Serial1,
					baud,
					SERIAL_8N1,
					timeout, polling,
					retry_count,
					TxEnablePin,
					packets,
					(NO_OF_PACKETS_IN_SLAVE * SLAVES_TOTAL_NO),
					regs[0]);

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

  float wind_speed = regs[0][0] / 10.0;

  Serial.println("-----------------");

  for (uint8_t slave = 0; slave < SLAVES_TOTAL_NO; slave++)
  {
      for (uint8_t register_master = 0; register_master < TOTAL_NO_OF_REGISTERS; register_master++)
      {
        Serial.print("slave ");
        Serial.print(slave);
        Serial.print(", reg ");
        Serial.print(register_master);
        Serial.print(" ");
        Serial.print(regs[slave][register_master]);
        Serial.println();
      }
  }

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

