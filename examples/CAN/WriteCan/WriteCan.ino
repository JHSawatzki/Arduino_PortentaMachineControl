/*
  CAN Write Example

  This sketch shows how to use the CAN transceiver on the Machine
  Control and how to transmit data from the TX CAN channel.

  Circuit:
   - Portenta H7
   - Machine Control

*/
#include <Arduino_MachineControl.h>
#include <CAN.h>

#define DATARATE_2MB     2000000
#define DATARATE_1_5MB   1500000
#define DATARATE_1MB     1000000
#define DATARATE_800KB   800000

void setup() {
  Serial.begin(9600);
  while (!Serial) {
    ; // wait for serial port to connect.
  }

  Serial.println("Start CAN initialization");

  MachineControl_CommProtocols.begin();
  MachineControl_CommProtocols.CANEnable();
  MachineControl_CommProtocols.CAN.frequency(DATARATE_800KB);
  Serial.println("Initialization done");
}

int counter = 0;
unsigned char payload = 0x49;
int payload_size = 1;

void loop() {

  mbed::CANMessage msg = mbed::CANMessage(13ul, &payload, payload_size);
  if (MachineControl_CommProtocols.CAN.write(msg)) {
    Serial.println("Message sent");
  } else {
    Serial.println("Transmission Error: ");
    Serial.println(MachineControl_CommProtocols.CAN.tderror());
    MachineControl_CommProtocols.CAN.reset();
  }

  delay(100);
}
