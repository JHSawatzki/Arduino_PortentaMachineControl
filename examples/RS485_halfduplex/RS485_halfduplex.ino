/*
 * Portenta Machine Control - RS485 Half Duplex Communication Example
 *
 * This sketch shows the usage of the SP335ECR1 on the Machine Control
 * as a half duplex (AB) RS485 interface. It demonstrates how to periodically send a string
 * on the RS485 TX channel.
 *
 * Circuit:
 *  - Portenta H7
 *  - Portenta Machine Control
 *  - A Slave device with RS485 interface
 *  - Connect TXP to A(+) and TXN to B(-)
 *
 * Initial author: Riccardo Rizzo @Rocketct
 * Authors who have contributed to updates:
 * - Leonardo Cavagnis @leonardocavagnis
 */

#include "Arduino_PortentaMachineControl.h"

constexpr unsigned long sendInterval { 1000 };
unsigned long sendNow { 0 };
unsigned long counter { 0 };

void setup() {
    Serial.begin(9600);
    while (!Serial) {
        ;
    }

    delay(1000);
    Serial.println("Start RS485 initialization");

    // Set the PMC Communication Protocols to default config
    // RS485/RS232 default config is:
    // - RS485 mode
    // - Half Duplex
    // - No A/B and Y/Z 120 Ohm termination enabled
    // Enable the RS485/RS232 system
    MachineControl_RS485Comm.begin(115200, 0, 500); // Specify baudrate, and preamble and postamble times for RS485 communication

    // Start in receive mode
    MachineControl_RS485Comm.receive();

    Serial.println("Initialization done!");
}

void loop() {
    if (MachineControl_RS485Comm.available())
        Serial.write(MachineControl_RS485Comm.read());

    if (millis() > sendNow) {
        // Disable receive mode before transmission
        MachineControl_RS485Comm.noReceive();

        MachineControl_RS485Comm.beginTransmission();

        MachineControl_RS485Comm.print("hello ");
        MachineControl_RS485Comm.println(counter++);

        MachineControl_RS485Comm.endTransmission();

        // Re-enable receive mode after transmission
        MachineControl_RS485Comm.receive();

        sendNow = millis() + sendInterval;
    }
}