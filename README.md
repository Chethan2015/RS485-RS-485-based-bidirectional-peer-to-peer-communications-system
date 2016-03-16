# RS485-RS-485-based-bidirectional-peer-to-peer-communications-system
EE 5314 Embedded Micro-controller System Design Class Project.

1.1 Overview
The goal of this project is to build a node capable of operation on an RS-485 based bidirectional
peer-to peer communications system.

The construction phase will require building a node capable of interfacing with a PC so that text
commands can be entered by the user. Based on the commands, subsequent transmission on the 2-wire RS-485 bus to other nodes will occur.
The node will also extract information out of the asynchronous data stream and to control three
or more devices. It will send acknowledgements back to the controller over the RS-485 physical
layer to indicate successful receipt of the data. The transceiver will also send unsolicited
messages back to the controller in response to changes in input device status.
A collection of most major parts will be provided to each single-person team. The pc boards,
tools, and any optional items are not included in this collection of parts.

1.2 Two-Wire RS-485 Link
The RS-485 specification details the electrical properties of the interface. In particular, it
specifies that differential signals be utilized and also specifies the voltage levels to be used for
signaling a high and low logic level. The RS-485 interface allows up to 32 standard loads
(devices nodes) on a shared data bus.

In the RS-485 2-wire configuration, a single differential pair (link) is provided. The controller
and the device nodes share the same link for communications. The data rate will be 38400 baud.
A termination of 120 ohms should be added across to the physical ends of a long wire run to
allow the bus to be properly terminated.

Since the pair can be driven by more than one node at a time, a potential exists for a collision on
the bus. In this project, any message sent shall be repeated at a delay calculated by a binomial
exponential back off algorithm if an acknowledgement is not received in an acceptable period of
time. To prevent the possibility of duplicate messages being received, a message identifier is
attached to each transmitted message to allow the recipient to determine message uniqueness.
A data is transmitted in groups of 11 bits, consisting of a start bit („0‟), a byte, an address („1‟) /
data bit („0‟), and a stop bit („1‟).

1.3 Node Hardware

The hardware is soldered together.
Microcontroller:
An ARM M4F core (TM4C123GH6PMI microcontroller) is used.
Serial interface:
In EK-TM4C123GXL evaluation board, the UART0 tx/rx pair is routed to the ICDI that
provides a virtual COM port through a USB endpoint.
RS-485 interface:
A SN75HVD12 is used to convert the CMOS logic level signals to/from the ARM to the
differential signals used for the 2-wire RS-485 interface. The receiver enable (RE) pin of the
SN75HVD12 is always enabled. Since only one node can talk on the bus at a time, the ARM
asserts the driver enable (DE) pin of the SN75HVD12 only when it needs to transmit on the bus.
Power LED:
A green LED with a 470ohm current limiting resistor to indicate power on the interface board is
connected with addition to any boards on the EK-TM4C123GXL.
Input device:
One or more input devices can be connected. At a minimum, a pushbutton is used. An on-board
pushbutton on the EK-TM4C123GXL is used.
TX LED:
A red LED is used for this purpose. The on-board red LED on the EK-TM4C123GXL is used.
RX LED:
A green LED is used for this purpose. The on-board green LED on the EK-TM4C123GXL is
used.
Output device:
3 devices or more devices (light bulb (low voltage only), servo, RGB LED, monochrome LED,
or another device) can be connected.
Connections:
A 2-pin terminal block is used to provide access to the RS-485 signals. This hardware can be
connected to another student‟s hardware to test hardware.

1.4 Data Packets
A packet is sent in the following bytes in order:
DST_ADDRESS: the address of the node that will receive the packet (with the 9th bit = „1‟)
SRC_ADDRESS: the address of the node that sent the packet
TX_SEQ_ID: a unique packet sequence number (can be a modulo 256 value)
COMMAND: the action being reported (the “verb”)
CHANNEL: the channel of the node to which this message applies (the “noun”)
SIZE: the number of data bytes being included
DATA[n]: n = 0, 1, … SIZE-1 bytes of data specific the packet being sent
CHECKSUM: the 1‟s compliment of the modulo-256 sum of all 6+SIZE bytes in the packet.
