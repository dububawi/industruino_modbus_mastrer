/*
  wind speed sensor, anemometer on INDUSTRUINO IND.I/O topboard 1286
  RS485 connection with wind sensor, Modbus protocol
  Industruino = Modbus Master
  Wind sensor = Modbus Slave ID: 2
  read holding register 0
  divide by 10 for wind speed in m/s

  more info on the SimpleModbusMaster library:
  https://drive.google.com/folderview?id=0B0B286tJkafVYnBhNGo4N3poQ2c&usp=drive_web&tid=0B0B286tJkafVSENVcU1RQVBfSzg#list

*/

#include <SimpleModbusMaster.h>
#include <U8glib.h>
// SPI Com: SCK = 21, MOSI = 20, CS = 19, A0 = 22
U8GLIB_MINI12864 u8g(21, 20, 19, 22);
//////////////////// Port information ///////////////////
#define BAUD 9600                                                                                  // SENSOR SPEC
#define TIMEOUT 1000
#define POLLING 200 // the scan rate
#define RETRY_COUNT 10
// used to toggle the receive/transmit pin on the driver
#define TxEnablePin 9                                                                           // INDUSTRUINO RS485
// The total amount of available memory on the master to store data

union modbus_float
{
	float v_float;
	uint16_t ar[2];
};

union modbus_int32
{
	float v_int32;
	uint16_t ar[2];
};

union modbus_float flow_rate_m3_h; /* 0x19 25d  Flow m3n/hr       32-bit Floating point    Read */
union modbus_float pressure_bar;   /* 0x28 40d  Pressure in bar   32-bit Floating point    Read */
union modbus_float temprature_c;   /* 0x48 72d  Temperature *C    32-bit Floating point    Read */
union modbus_float totalizer_m3;   /* 0x88 136d Totalizer m^3     32-bit Floating point    Read / Write */

uint8_t slave_ids[] = {2};
#define SLAVES_TOTAL_NO (sizeof(slave_ids) / sizeof(slave_ids[0]))

#define TOTAL_NO_OF_REGISTERS 8                                                // SENSOR SPEC
// This is the easiest way to create new packets
// Add as many as you want. NO_OF_PACKETS_IN_SLAVE
// is automatically updated.

// Group nearby registers in packets   
// 0x19 -- packet 1
// 0x28 -- packet 2
// 0x48 -- packet 3
// 0x88 -- packet 4
enum
{
  PACKET1,                                          // only need 1 type of operation: read wind sensor
  PACKET2,
  PACKET3,
  PACKET4,
  NO_OF_PACKETS_IN_SLAVE // leave this last entry
};

const uint16_t packet_start_register[NO_OF_PACKETS_IN_SLAVE] = {0x19, 0x28, 0x48, 0x88};
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
					BAUD,
					SERIAL_8N1,
					TIMEOUT, POLLING,
					RETRY_COUNT,
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
  // send 1 simple Master request to Slave, as defined above
  modbus_update();

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
        Serial.print(regs[slave][register_master], HEX);
        Serial.println();
      }
  }

  flow_rate_m3_h.ar[0] = regs[0][0];
  flow_rate_m3_h.ar[1] = regs[0][1];

  pressure_bar.ar[0] = regs[0][2];
  pressure_bar.ar[1] = regs[0][3];

  temprature_c.ar[0] = regs[0][4];
  temprature_c.ar[1] = regs[0][5];

  totalizer_m3.ar[0] = regs[0][6];
  totalizer_m3.ar[1] = regs[0][7];

  Serial.print("flow_rate_m3_h ");
  Serial.println(flow_rate_m3_h.v_float);

  Serial.print("pressure_bar ");
  Serial.println(pressure_bar.v_float);

  Serial.print("temprature_c ");
  Serial.println(temprature_c.v_float);

  Serial.print("totalizer_m3 ");
  Serial.println(totalizer_m3.v_float);

  u8g.firstPage();
  do {

    u8g.setFont(u8g_font_unifont);

    u8g.setPrintPos(20, 15);
    u8g.print(flow_rate_m3_h.v_float);
    u8g.print(" m3/h");

    u8g.setPrintPos(20, 30);
    u8g.print(pressure_bar.v_float);
    u8g.print(" bar");

    u8g.setPrintPos(20, 45);
    u8g.print(temprature_c.v_float);
    u8g.print(" *C");

    u8g.setPrintPos(20, 60);
    u8g.print(totalizer_m3.v_float);
    u8g.print(" m3");

  } while ( u8g.nextPage() );

  delay(3000);
}
