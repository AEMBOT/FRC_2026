# Repo Memory - FRC_2026

Last updated: 2026-03-05

## Purpose
This file is the persistent memory for repo architecture, runtime behavior, and loop-overrun findings so future debugging does not restart from zero.

## Snapshot
- Java source files: 179 (`src/main/java`)
- Total tracked files discovered in workspace scan: 248
- Main runtime entry: `com.aembot.frc2026.Main` -> `Robot` (`LoggedRobot`)
- Log framework: AdvantageKit (`Logger.recordOutput`, `Logger.processInputs`)

## Runtime Model
- `Robot.robotPeriodic()`:
  - `CommandScheduler.run()`
  - `RobotStateYearly.get().updateLog()`
  - `RobotContainer.logCommands()`
- `Robot.disabledPeriodic()`:
  - `CANStatusLogger.updateAllLogs()`

## Subsystem/IO Pattern
- Most subsystems follow:
  - `periodic()` -> `io.updateInputs(inputs)` -> `updateLog()`
- Drive:
  - `DriveSubsystem.periodic()` updates drive IO, robot state, module logs, odometry stddevs.
- Vision:
  - `AprilVisionSubsystem.periodic()` iterates cameras, logs camera pose, updates IO, pushes observations into robot state.
- Turret:
  - Updates 2 CANCoders each periodic.

## Confirmed Overrun Drivers
1. CAN diagnostics in disabled loop
- Path: `Robot.disabledPeriodic()` -> `CANStatusLogger.updateAllLogs()` -> bus/device logging.
- Overrun stacks hit `CANBus.getStatus`, `CANStatusLogger.logDeviceStatuses`.
- Effect: large disabled-loop stalls and repeated watchdog overruns.

2. Blocking CTRE wait in turret periodic
- `CANCoderIOHardware.updateInputs()` calls `BaseStatusSignal.waitForAll(10.0, signals)` during warmup.
- Turret calls two encoder updates every periodic.
- Overrun stacks repeatedly include `StatusSignalJNI.JNI_WaitForAll` via turret periodic.

3. Heavy per-cycle vision + logging allocations
- `AprilVisionSubsystem.periodic()` records pose outputs and processes inputs for each camera each cycle.
- `Limelight4IOHardware` performs frequent NT interactions and pose conversions.
- Overrun stacks repeatedly include `AprilVisionSubsystem.periodic` and `LogTable.put`.

4. Logging volume and string churn
- Many outputs are recorded each cycle across subsystems.
- Overrun stacks repeatedly include `Logger.periodicAfterUser`, `LogTable.put`, `LogTable.clone`, queue operations.

## Quantitative Evidence (from checked logs)
- `Console 3-4-2026.txt` parse highlights:
  - `robotPeriodic()` max ~896 ms
  - `DriveSubsystem.periodic()` max ~533 ms
  - `AprilVisionSubsystem.periodic()` max ~126 ms
  - `TurretSubsystem.periodic()` max ~84 ms
- `overrun_log.txt` contains direct stack traces in:
  - `CANStatusLogger` path
  - `CANCoderIOHardware.updateInputs` wait path
  - `AprilVisionSubsystem` + `Logger` write path

## Non-Standard/Correctness Issues Found
- String identity comparison used for turret camera filter:
  - `AprilVisionSubsystem`: `if (io.getConfiguration().cameraName == "turret")`
- Duplicate auto setup calls in `RobotContainer` constructor:
  - `setupAutoFactory/registerAutoCommands/setupAutoChooser/putData` block appears twice.
- Gyro registration repeated in `CANStatusLogger.registerSwerveDrivetrain()`
  - gyro device added inside module loop.
- Debug print in drive command:
  - `JoystickDriveCommand.driveWithHeading()` uses `System.out.println("driveWithHeading")`.

## Quick Re-Validation Commands
Run from repo root:

```powershell
rg -n "Loop time of 0.02s overrun|CommandScheduler loop overrun" overrun_log.txt
```

```powershell
rg -n "waitForAll\\(|Logger\\.periodicAfterUser|CANStatusLogger|AprilVisionSubsystem\\.periodic|DriveSubsystem\\.periodic|TurretSubsystem\\.periodic" overrun_log.txt
```

```powershell
rg -n "periodic\\(|updateLog\\(|Logger\\.recordOutput|Logger\\.processInputs" src/main/java
```

## Update Protocol
When new profiling happens:
1. Add date and tested log filenames.
2. Add top 5 loop contributors with max and typical values.
3. Add any new stack-trace hot paths.
4. Mark whether each known driver was still present or fixed.
5. Keep this file current before broad refactors.
