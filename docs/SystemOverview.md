# System Overview

## Gen3 SOS Compressor

## Introduction
This project aims to develop IEC Structured Text source code for a Yaskawa MP2600iec servo drive controller.
The application is a third generation, pre-clinical medical device being used to study an intervention for hypoxia in a university research context.
The intervention is the infusion of a saline solution with oxygen micro bubbles into the blood stream.
The device produces and delivers this solution by dissolving oxygen into a saline solution at a high pressure and discharging the solution to near atmospheric pressure through micro-orifices (leading to the nucleation of micro-bubbles).
The servo system provides the motive force for this process.

## Application process
The process the device performs is as follows.
Step 1: Saline is drawn into a compression chamber, comprised of a cylinder and piston, in excess of the specified volume.
The saline source is closed off from the chamber.

Step 2: A waste stream outlet is opened. Air and excess saline are purged from the compression chamber leaving the specified volume of saline.
The waste stream outlet is closed.

Step 3: An oxygen source inlet is opened to the chamber.
Oxygen is drawn into the chamber.
A pressure transducer in communication with the compression chamber senses the pressure.
The density of the gas is estimated, assuming room temperature.
The volume required to give the specified mass of oxygen is calculated, giving the target end-position for the piston.
The piston moves to that position. The oxygen source inlet is closed.

Step 4: The piston moves to compress the two phases to a pressure sufficient to dissolve the oxygen in the saline solution.
A magnetic stir-bar in the compression chamber is driven by a rotating magnet attached to a DC motor embedded within the piston but outside the compression chamber. This accelerated diffusion of the dissolved oxygen.

Step 5: Concurrent with Step 4, a peristaltic pump is used to drive saline through the passages leading to and away from the compression chamber to flush out residual air.

Step 6: After a sufficient amount of time, the solution is ready for discharge and delivery.
Valves leading to the micro-orifice(s) are opened, the intermediate volume is pressurized, and solution is driven through the orifices.
The piston continues to move and apply pressure to the fluid.
Once the residual fluid in the intermediate volume is driven out, the super-oxygenated saline begins reaches and flow through the orifices, producing the oxygen micro-bubble/saline mixture.

## Servo System
The controller is controlling a Yaskawa Sigma-7 Servo Drive (model SGD7S2R8FE0A000300), driving a single axis.
The axis drives a linear electromechanical actuator (Tolomatic RSA64).
The servo motor is a 400W Yaskawa Sigma-7 motor with a safety brake option and an absolute encoder (model SGM7J-04A6A6C).

The controller is planned to be programmed as a slave to a Simulink Desktop Real Time program.
The servo controller slave shall implement basic modes of operation at the command of the master.
The Simulink Desktop Real Time environment as the master enables faster iteration and a familiar interface for the engineering graduate student/researchers, while enabling a rough but workable interface for operators (via dashboard components or, at a later point an app via Simulink's App Designer).

The master controller is interfacing with the system through a National Instruments (NI) PCI 6251 DAQ, accompanied by a NI SCB-68a breakout board.

We have a MotionWorksIEC Express license for programming the servo controller and drive, which has a smaller feature set than MotionWorksIEC Pro but is adequate this application.

The MP2600iec has:
- eight digital inputs
- eight digital outputs
- one analog input
- one analog output

The servo drive connects to the servo encoder directly without needing to use any of the eight digital inputs.

Control of the brake is being handled by a separate E-stop circuit with input from the NI PIC 6251.
That E-stop circuit controls the brake and the Safety Torque Off (STO) inputs to the drive's IGBT's.
One of the slave's digital outputs will be wired to an additional relay that provides input to the brake's actuation without affecting the STO.
(The brake is normally engaged and the circuit provides 24V power to disengage it allow servo motion.)

For the slave-side software architecture and state machine, see [docs/slave/development/SystemArchitecture.md](./slave/development/SystemArchitecture.md).
For the master-slave I/O and protocol contract, see [docs/master/MasterProtocolGuide.md](./master/MasterProtocolGuide.md).
