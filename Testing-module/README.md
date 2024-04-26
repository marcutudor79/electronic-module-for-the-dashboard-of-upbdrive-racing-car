# ECU Emulator

## Description
In order to debug the CAN bus as well as the dashboard module, this ECU Emulator
injects CAN packets into the CAN bus. It is a replacement for the ECU itself, because
we cannot use it just for electronics debugging, it has to be mounted in the FS car.

## How does it work
This ECU Emulator uses a potentiometer to emulate the accelerator pedal of the FS car
as well as a button to emulate the gear shift. The potentiometer is read via the built-in
ADC module of the Raspberry Pi Pico board, and the button is read using GPIO functions.

### How is the data generated
- rpm is generated using a constant 800 + the potentiometer value + a random number
between 0 - 49 in order to emulate the actual rpm of the FS car. The expected range is
between 800 - 13000.
- current gear is initially 0 (netural) and incremented when the button is pressed, until 4,
then it is set back to 0.
- all the other data is assigned the saw tooth signal between 0 - 250.

The CAN packets are formatted with respect to the CAN Packet allocation excel and then sent via the CAN bus.

## Project structure
```
- Docs     /* folder the documentation files */
- Firmware /
            - components/ - adc                 /* part of the code that enables the onboard
                                                    adc and configures a gpio to read a button */
                          - can                 /* part of the code that enables can communication
                                                    via the can bus transciever
                                                    also formats the packets according to the CAN
                                                    packet allocation */
                          - common              /* contains a header file in which the pins used
                                                    in the project are defined */
                          - heartbeat           /* part of the code that enables the built-in LED
                                                    to blink, for debug purposes */
                          - signal-generation   /* part of the code that generates the rpm data,
                                                   CurrentGear data and a sawtooth signal that
                                                   will replace the other data in the CAN packets */
            - main                /* contains the main source and header file with
                                    which describes the main loop of the program */
            - modules             /* contains can2040 git submodule that enables
                                     the PICO to communicate via CAN using bit banging
                                     on GPIOs */

- README.md /* project README file */
```


