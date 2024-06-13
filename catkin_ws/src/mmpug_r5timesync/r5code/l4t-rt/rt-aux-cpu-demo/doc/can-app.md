CAN Application (app/can-app.c)
================================
@tableofcontents

This section explains how to use the CAN application on
NVIDIA<sup>&reg;</sup> Jetson<sup>&trade;</sup> TX 2 and AGX.

There are two CAN controllers, which run in Always On (AON) mode. As such, the
APIs that interface with those controllers are called AON CAN controller driver
APIs. This application demonstrates how to use them.

# Compiling the Application #

To control compilation of the CAN application, use the
`ENABLE_CAN_APP` flag in the `soc/*/target_specific.mk` file.

@note It is required to compile device tree and flash board to have device tree
and SCR settings updated after making kernel device tree and SCR changes as
described in following sections.

# Prerequisites #

Before you can to successfully execute the application, you must obtain two CAN
transceivers. The transceivers must be 3.3V I/O compatible, similar to the ones
at:

https://www.amazon.com/SN65HVD230-CAN-Board-Communication-Development/dp/B00KM6XMXO

# Operation #

By default, the CAN AON controller driver sets the bit rate at 500-kilobits per
second and the data bit rate at 2-megabits per second. For more information, see
the `mttcan_controller_init` function in the `tegra-can.c` driver source file.

The application uses both AON CAN controllers. It also connects the CAN 0 bus to
CAN 1, using the above mentioned CAN transceivers, where:
- CAN 0 transmits pre-defined messages and
- CAN 1 receives such messages.

The application prints the message details received at CAN 1 bus. It also
prints the transmit element, which includes information about the transmitted
message. The transmit element is also created when there is a bus state change.
Below is the sample output that is printed on successful transmission.

    Transmited message from CAN 0
    Transmission complete event
    Transmit event element information:
    Message ID: 0xa5, Event Type: Tx event, CAN Frame: <frame details>
    Message received at CAN 1
    Message ID: 0xa5, Message data length: 2
    Message Data:
    0xaa 0x55

# Wiring Details #

@code

------------------    ----------------------        -------------------     ----------------------
|                |    |                    |        |                 |     |                    |
|             TX |--->| TX           CAN H |<------>|CAN H         TX |<----| TX                 |
| AON CAN0       |    |   CAN Transceiver  |        |  CAN Transceiver|     | AON CAN1 Controller|
| Controller  RX |<---| RX           CAN L |<------>|CAN H         RX |---->| RX                 |
|                |    |                    |        |                 |     |                    |
------------------    ----------------------        -------------------     ----------------------

@endcode

@note It is recommended to use termination between CAN devices according to
CAN specifications.


# Jetson TX2 Configuration #

Ensure the CAN is disabled in the kernel device tree
`tegra186-quill-p3310-1000-c03-00-base.dts` as shown below:
```
mttcan@c310000 {
        status = "disabled";
        ...
};
mttcan@c320000 {
        status = "disabled";
        ...
};
```

CAN 0 and 1 controller TX and RX are brought out to 30 pin header J26 as below:
- Pin 2 - 3.3V
- Pin 10 - GND
- Pin 5 - CAN0_RX ---> Connects RX pin of CAN transceiver
- Pin 7 - CAN0_TX ---> Connects TX pin of CAN transceiver
- Pin 15 - CAN1_RX ---> Connects RX pin of CAN transceiver
- Pin 17 - CAN1_TX ---> Connects TX pin of CAN transceiver

Additional 3.3V and GND connections are available at 40 pin header J21 as below:
- Pin 1, 17 - 3.3V
- Pin 25, 39 - GND

# Jetson AGX Configuration #

Ensure the CAN is disabled in the kernel device tree `tegra194-p2888-0001-p2822-0000-common.dtsi`
as shown below:
```
mttcan@c310000 {
        status = "disabled";
        ...
};
mttcan@c320000 {
        status = "disabled";
        ...
};
```

Modify SCR setting in tegra194-mb1-bct-scr-cbb-mini.cfg as follows:

- scr.378.4 = 0x1a0034ff; # CLK_RST_CONTROLLER_AON_SCR_CAN_0
- scr.407.4 = 0x1a0034ff; # CLK_RST_CONTROLLER_CAN1_SCR_FMON_0
- scr.408.4 = 0x1a0034ff; # CLK_RST_CONTROLLER_CAN2_SCR_FMON_0
- scr.2602.6 = 0x19003232; # AON_NOC_CAN1_BLF_CONTROL_REGISTER_0
- scr.2603.6 = 0x19003232; # AON_NOC_CAN2_BLF_CONTROL_REGISTER_0

Modify pinmux setting in tegra19x-mb1-pinmux-p2888-0000-a04-p2822-0000-b01.cfg
as follows:

- pinmux.0x0c303000 = 0x00000400; # can1_dout_paa0: rsvd1, pull-down, tristate-enable, input-enable
- pinmux.0x0c303008 = 0x00000458; # can1_din_paa1: rsvd1, pull-down, tristate-enable, input-enable
- pinmux.0x0c303010 = 0x00000400; # can0_dout_paa2: rsvd1, pull-up, tristate-enable, input-enable
- pinmux.0x0c303018 = 0x00000458; # can0_din_paa3: rsvd1, pull-up, tristate-enable, input-enable

CAN 0 and 1 controller TX and RX are brought out to 40 pin header J30 as below:
- Pin 1, 17 - 3.3V
- Pin 9, 25, 39 - GND
- Pin 29 - CAN0_RX ---> Connects RX pin of CAN transceiver
- Pin 31 - CAN0_TX ---> Connects TX pin of CAN transceiver
- Pin 37 - CAN1_RX ---> Connects RX pin of CAN transceiver
- Pin 33 - CAN1_TX ---> Connects TX pin of CAN transceiver
