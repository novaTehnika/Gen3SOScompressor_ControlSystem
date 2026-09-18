# Test and Verification Plan (Outline)

## Gen3 SOS Compressor Servo Controller
### Software Test Plan Framework

---

## 1.0 Introduction

### 1.1 Purpose
This document outlines the strategy, scope, and procedures for testing the MP2600iec servo controller software. Its purpose is to verify that the software meets all functional requirements and safety specifications.

### 1.2 Scope
This plan covers the verification of the Structured Text application running on the MP2600iec controller. It includes:
-   I/O signal verification.
-   Functional testing of all operational and homing modes.
-   Verification of the master-slave handshake protocol.
-   Fault injection testing and system recovery validation.
-   Performance and stability testing.

This plan does **not** cover the testing of the Simulink master controller software, except where its interaction is required to stimulate the slave.

### 1.3 Reference Documents
-   docs/SystemOverview.md
-   docs/master/MasterProtocolGuide.md
-   docs/master/IOReference.md
-   docs/master/FaultCodeReference.md
-   docs/slave/development/ConfigurationGuide.md

---

## 2.0 Test Environment

### 2.1 Hardware
-   Yaskawa MP2600iec Controller
-   Yaskawa Sigma-7 Servo Drive, Motor, and Brake
-   Tolomatic RSA64 Linear Actuator
-   NI PCI 6251 DAQ with SCB-68a breakout board
-   Test PC with Simulink Desktop Real-Time
-   Power supplies (24VDC)
-   Digital Multimeter, Oscilloscope
-   Physical apparatus for triggering limit switches

### 2.2 Software
-   MotionWorksIEC (for slave debugging)
-   Simulink DTRT (as master controller)
-   NI Measurement & Automation Explorer (for DAQ configuration)
-   A dedicated Simulink test model capable of:
    -   Commanding all modes.
    -   Logging all DI/DO and AI/AO signals at 1kHz.
    -   Automating test sequences (e.g., mode changes, fault resets).

---

## 3.0 Test Procedures and Acceptance Criteria

### 3.1 I/O Verification
-   **Objective:** Verify all digital and analog I/O between the master and slave are correctly wired and functioning.
-   **Procedure:**
    -   Test each digital output from the master and verify the correct digital input is seen by the slave.
    -   Test each digital output from the slave and verify the correct digital input is seen by the master.
    -   Test analog output from the master with specific voltages (-10V, 0V, +10V) and verify the `LREAL` voltage value seen by the slave on `G_aiReference` matches the applied input.
    -   Command the slave to output specific analog voltages via `G_aoPositionOutput` (`LREAL`) and verify the values seen by the master.
-   **Acceptance Criteria:** All signals must match their expected values within the tolerances specified in `IOReference.md`.

### 3.2 Handshake Protocol Testing
-   **Objective:** Verify the mode entry and fault reset handshakes function correctly.
-   **Procedure:**
    -   Test a valid mode entry handshake.
    -   Test a handshake timeout by having the master fail to raise `G_diMotionEnable`.
    -   Test a handshake mismatch by having the master change mode bits after the slave confirms.
    -   Test a valid fault reset handshake with correct fault code mirroring.
    -   Test a failed fault reset due to incorrect code mirroring.
    -   Test a failed fault reset due to `G_diMotionEnable` being HIGH.
-   **Acceptance Criteria:** The slave must enter the correct state (`ST_FAULT` or the new operational state) in each scenario as described in `MasterProtocolGuide.md`.

### 3.3 Operational Mode Testing
-   **Objective:** Verify the correct functionality of each operational mode.
-   **For each mode (Brake Hold, Position, Velocity, Torque):**
    -   **Procedure:**
        1.  Perform a valid handshake to enter the mode.
        2.  Provide command inputs (e.g., analog reference voltage).
        3.  Provide external inputs (e.g., drop `G_diMotionEnable`).
    -   **Acceptance Criteria:**
        -   The slave confirms the correct mode.
        -   The motor behaves as expected (e.g., holds position, follows velocity command).
        -   Dropping `G_diMotionEnable` causes a controlled stop and transition to `ST_HOLD_POSITION`.
        -   Position feedback is accurate.

### 3.4 Homing Sequence Verification
-   **Objective:** Verify that all homing modes function correctly and set the position accurately.
-   **Procedures:**
    -   **Mode 110 (Home to Limit):**
        1.  Start from a position away from the home switch.
        2.  Command Mode 110.
        3.  Verify the axis moves toward the switch, triggers it, backs off, and sets the position to the configured value (`G_cfgHomeLimSetPosition`).
    -   **Mode 111 (Home to EOT):**
        1.  Command Mode 111.
        2.  Verify the fast and slow approach, and confirm that a stall is detected at the hard stop.
        3.  Verify the `G_posEOTOffset` is calculated correctly.
    -   **Mode 101 (Go Home):**
        1.  After completing Mode 110 (so `G_flagAbsHomeRequired = FALSE`), command Mode 101 and verify it moves directly to the home position.
        2.  After a power cycle or fault reset (so `G_flagAbsHomeRequired = TRUE`), command Mode 101 and verify it redirects into Mode 110.
-   **Acceptance Criteria:** All sequences complete successfully, `G_doHomingComplete` is set HIGH, and the final position is correct and repeatable.

### 3.5 Fault Injection Testing
-   **Objective:** Verify the system detects all specified faults and enters a safe state.
-   **For each fault code in `FaultCodeReference.md`:**
    -   **Procedure:** Create the conditions to trigger the fault.
        -   `FAULT_HANDSHAKE`: See 3.2.
        -   `FAULT_DRIVE`: Use drive software to force a fault, or disconnect a motor phase.
        -   `FAULT_POSITION`: Command the axis to move beyond its soft limits.
        -   `FAULT_HOMING_REQ`: Attempt an operational mode before homing.
        -   `FAULT_PISTON_EXIT`: Physically push the actuator back while in torque mode near the exit boundary.
        -   `FAULT_LIMIT_SWITCH`: Manually trigger a limit switch during a normal move.
        -   `FAULT_ENCODER`: Reserved / not emitted by firmware — encoder alarms surface as `FAULT_DRIVE` via the servopack. Trigger by disconnecting the encoder battery or simulating an A.810 alarm; expect `FAULT_DRIVE`.
    -   **Acceptance Criteria:** In every case, the `G_doFaultActive` signal must go HIGH, and the correct fault code must be present on `G_doModeConfBit0-2`. The system must perform a controlled stop.
-   **Limit Recovery (`ST_RECOVERY`):** For `FAULT_LIMIT_SWITCH` and `FAULT_POSITION` with the limit **still active** at reset:
    -   Reset from `ST_FAULT` (fast path) → verify the slave enters `ST_RECOVERY` (drive stays on), not `ST_BRAKE_HOLD`.
    -   Reset from `ST_FAULT_IDLE` (after `G_cfgFaultIdleTimeout`) → verify the drive re-enables and the slave lands in `ST_RECOVERY`.
    -   In `ST_RECOVERY`, select Mode 010/011 + `G_diMotionEnable`; command **away** from the limit → axis retracts; command **toward** the limit → motion is clamped to zero and **no** new fault is raised.
    -   Jog clear of the switch and back inside soft limits, drop `G_diMotionEnable` → verify transition to `ST_HOLD_POSITION` and that normal modes resume without faulting.
    -   Regression: reset with the limit **already cleared** → verify the slave goes straight to `ST_BRAKE_HOLD`/`ST_IDLE` (no recovery detour). Force a drive fault mid re-enable → verify it faults and a later normal activation is not diverted to recovery.
    -   **Acceptance Criteria:** Recovery is reachable from both fault paths, only the safe travel direction is permitted, and a successful retract rejoins normal operation. Drive and piston-exit faults still trigger while in recovery.

### 3.6 Performance and Stability Testing
-   **Objective:** Verify the system is stable and performant under load and over time.
-   **Procedures:**
    -   **Soak Test:** Run the system in a continuous loop (e.g., moving between two points in position control) for an extended period (e.g., 8 hours) and monitor for unexpected faults or behavior.
    -   **Load Test:** Apply expected mechanical loads to the actuator and verify that position/velocity/torque control remains stable and accurate.
    -   **Disturbance Test:** Inject noise into analog signals or rapidly toggle digital inputs (not involved in handshakes) to ensure system stability.
-   **Acceptance Criteria:** The system must operate without unintended faults. Control loop performance must remain within specification.

---

## 4.0 Traceability

A traceability matrix will be created to link each software requirement to one or more test cases in this plan, ensuring complete test coverage.

| Requirement ID | Requirement Description | Test Case ID(s) |
|---|---|---|
| REQ-001 | The system shall support Position Control mode. | 3.3.1, 3.5.3 |
| REQ-002 | The system shall detect and report a Drive Fault. | 3.5.2 |
| ... | ... | ... |
