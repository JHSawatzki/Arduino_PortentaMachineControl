/*
  RS485 Half Duplex communication

  This sketch shows how to use the SP335ECR1 on the Machine
  Control as a half duplex (AB) RS485 interface, how to periodically
  send a string on the RS485 TX channel and how to receive data
  from the interface RX channel.

  Circuit:
   - Portenta H7
   - Machine Control
   - A Slave device with RS485 interface
   - Connect TXP to A(+) and TXN to B(-)

*/

#include "Arduino_MachineControl.h"

constexpr unsigned long sendInterval { 1000 };
unsigned long sendNow { 0 };

unsigned long counter { 0 };

void setup()
{

    Serial.begin(115200);
    // Wait for Serial or start after 2.5s
    for (auto const timeout = millis() + 2500; !Serial && timeout < millis(); delay(500))
        ;

    delay(2500);
    Serial.println("Start RS485 initialization");

    // Set the PMC Communication Protocols to default config
    MachineControl_CommProtocols.begin();

    // RS485/RS232 default config is:
    // - RS485 mode
    // - Half Duplex
    // - No A/B and Y/Z 120 Ohm termination enabled

    // Enable the RS485/RS232 system
    MachineControl_CommProtocols.RS485Enable();

    // Specify baudrate, and preamble and postamble times for RS485 communication
    MachineControl_CommProtocols.RS485.begin(115200, 0, 500);
    // Start in receive mode
    MachineControl_CommProtocols.RS485.receive();

    Serial.println("Initialization done!");
}

void loop()
{
    if (MachineControl_CommProtocols.RS485.available())
        Serial.write(MachineControl_CommProtocols.RS485.read());

    if (millis() > sendNow) {
        // Disable receive mode before transmission
        MachineControl_CommProtocols.RS485.noReceive();

        MachineControl_CommProtocols.RS485.beginTransmission();

        MachineControl_CommProtocols.RS485.print("hello ");
        MachineControl_CommProtocols.RS485.println(counter++);

        MachineControl_CommProtocols.RS485.endTransmission();

        // Re-enable receive mode after transmission
        MachineControl_CommProtocols.RS485.receive();

        sendNow = millis() + sendInterval;
    }
}
