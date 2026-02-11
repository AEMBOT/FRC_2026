package com.aembot.lib.subsystems.drive.simulation;

import com.aembot.lib.config.subsystems.drive.DrivetrainConfiguration;
import com.aembot.lib.config.subsystems.drive.SimSwerveModuleConfiguration;
import com.aembot.lib.config.subsystems.drive.SwerveModuleConfiguration;
import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.Pigeon2;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.sim.Pigeon2SimState;
import com.ctre.phoenix6.swerve.SwerveDrivetrain;
import com.ctre.phoenix6.swerve.SwerveModule;
import com.ctre.phoenix6.swerve.SwerveModuleConstants;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.units.Units;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.Mass;
import edu.wpi.first.units.measure.Time;
import edu.wpi.first.wpilibj.RobotBase;
import java.util.List;
import org.ironmaple.simulation.SimulatedArena;
import org.ironmaple.simulation.drivesims.COTS;
import org.ironmaple.simulation.drivesims.SwerveDriveSimulation;
import org.ironmaple.simulation.drivesims.SwerveModuleSimulation;
import org.ironmaple.simulation.drivesims.configs.DriveTrainSimulationConfig;
import org.ironmaple.simulation.drivesims.configs.SwerveModuleSimulationConfig;

/** Class allowing use of the CTRE swerve sim with MapleSim */
public class MapleSimSwerveDrivetrain {
  private final Pigeon2SimState pigeonSim;
  private final SimSwerveModuleConfiguration[] simSwerveModules;

  public final SwerveDriveSimulation mapleSimSwerveDrivetrain;

  /**
   * Constructs a drivetrain simulation using the specified parameters
   *
   * @param simPeriod the time period of the simulation
   * @param robotMassWithBumpers the total mass of the robot, including bumpers
   * @param bumperLengthX the length of the bumper along the X-axis (influences the collider of the
   *     robot)
   * @param bumperWidthY the width of the bumper along the Y-axis (influences the collider of the
   *     robot)
   * @param driveMotorModel the {@link DCMotor} model for the drive motor, typically <code>
   *     DCMotor.getKrakenX60Foc()
   *     </code>
   * @param steerMotorModel the {@link DCMotor} model for the steer motor, typically <code>
   *     DCMotor.getKrakenX60Foc()
   *     </code>
   * @param wheelCOF the coefficient of friction of the drive wheels
   * @param moduleLocations the locations of the swerve modules on the robot, in the order <code>
   *     FL, FR, BL, BR</code>
   * @param pigeon the {@link Pigeon2} IMU used in the drivetrain
   * @param modules the {@link SwerveModule}s, typically obtained via {@link
   *     SwerveDrivetrain#getModules()}
   * @param moduleConstants the constants for the swerve modules
   */
  public MapleSimSwerveDrivetrain(
      Time simPeriod,
      Mass robotMassWithBumpers,
      Distance bumperLengthX,
      Distance bumperWidthY,
      DCMotor driveMotorModel,
      DCMotor steerMotorModel,
      double wheelCOF,
      Translation2d[] moduleLocations,
      Pigeon2 pigeon,
      SwerveModule<TalonFX, TalonFX, CANcoder>[] modules,
      SwerveModuleConstants<?, ?, ?>[] moduleConstants,
      List<
              SwerveModuleConfiguration<
                  TalonFXConfiguration, TalonFXConfiguration, CANcoderConfiguration>>
          swerveModuleConfigurations) {

    this.pigeonSim = pigeon.getSimState();

    simSwerveModules = new SimSwerveModuleConfiguration[moduleConstants.length];

    DriveTrainSimulationConfig simulationConfig =
        DriveTrainSimulationConfig.Default()
            .withRobotMass(robotMassWithBumpers)
            .withBumperSize(bumperLengthX, bumperWidthY)
            .withGyro(COTS.ofPigeon2())
            .withCustomModuleTranslations(moduleLocations)
            .withSwerveModule(
                new SwerveModuleSimulationConfig(
                    driveMotorModel,
                    steerMotorModel,
                    moduleConstants[0].DriveMotorGearRatio,
                    moduleConstants[0].SteerMotorGearRatio,
                    Units.Volts.of(moduleConstants[0].DriveFrictionVoltage),
                    Units.Volts.of(moduleConstants[0].SteerFrictionVoltage),
                    Units.Meters.of(moduleConstants[0].WheelRadius),
                    Units.KilogramSquareMeters.of(moduleConstants[0].SteerInertia),
                    wheelCOF));

    mapleSimSwerveDrivetrain = new SwerveDriveSimulation(simulationConfig, new Pose2d());

    SwerveModuleSimulation[] moduleSimulations = mapleSimSwerveDrivetrain.getModules();
    for (int i = 0; i < this.simSwerveModules.length; i++)
      simSwerveModules[i] =
          new SimSwerveModuleConfiguration(
              moduleConstants[i],
              moduleSimulations[i],
              modules[i],
              swerveModuleConfigurations.get(i));

    SimulatedArena.overrideSimulationTimings(simPeriod, 1);
    SimulatedArena.getInstance().addDriveTrainSimulation(mapleSimSwerveDrivetrain);
  }

  /** Updates the Maple-Sim simulation and injects the results into simulated CTRE devices */
  public void update() {
    SimulatedArena.getInstance().simulationPeriodic();
    pigeonSim.setRawYaw(
        mapleSimSwerveDrivetrain.getSimulatedDriveTrainPose().getRotation().getMeasure());
    pigeonSim.setAngularVelocityZ(
        Units.RadiansPerSecond.of(
            mapleSimSwerveDrivetrain.getDriveTrainSimulatedChassisSpeedsRobotRelative()
                .omegaRadiansPerSecond));
  }

  // Spotless really wants those two blank lines to be there and idk why
  /**
   *
   *
   * <h2>Regulates all {@link SwerveModuleConstants} for a drivetrain simulation.</h2>
   *
   * <p>This method processes an array of {@link SwerveModuleConstants} to apply necessary
   * adjustments for simulation purposes, ensuring compatibility and avoiding known bugs.
   *
   * <p>Yoinked from 254;
   * https://github.com/Team254/FRC-2025-Public/blob/main/src/main/java/com/team254/frc2025/utils/simulations/MapleSimSwerveDrivetrain.java
   *
   * @see #regulateModuleConstantForSimulation(SwerveModuleConstants)
   */
  public static SwerveModuleConstants<?, ?, ?>[] regulateModuleConstantsForSimulation(
      SwerveModuleConstants<?, ?, ?>[] moduleConstants) {
    for (SwerveModuleConstants<?, ?, ?> moduleConstant : moduleConstants)
      regulateModuleConstantForSimulation(moduleConstant);

    return moduleConstants;
  }

  /**
   *
   *
   * <h2>Regulates the {@link SwerveModuleConstants} for a single module.</h2>
   *
   * <p>This method applies specific adjustments to the {@link SwerveModuleConstants} for simulation
   * purposes. These changes have no effect on real robot operations and address known simulation
   * bugs:
   *
   * <ul>
   *   <li><strong>Inverted Drive Motors:</strong> Prevents drive PID issues caused by inverted
   *       configurations.
   *   <li><strong>Non-zero CanCoder Offsets:</strong> Fixes potential module state optimization
   *       issues.
   *   <li><strong>Steer Motor PID:</strong> Adjusts PID values tuned for real robots to improve
   *       simulation performance.
   * </ul>
   *
   * <h4>Note:This function is skipped when running on a real robot, ensuring no impact on constants
   * used on real robot hardware.</h4>
   *
   * <p>Yoinked from 254;
   * https://github.com/Team254/FRC-2025-Public/blob/main/src/main/java/com/team254/frc2025/utils/simulations/MapleSimSwerveDrivetrain.java
   */
  private static void regulateModuleConstantForSimulation(
      SwerveModuleConstants<?, ?, ?> moduleConstants) {
    // Skip regulation if running on a real robot
    if (RobotBase.isReal()) return;

    // Apply simulation-specific adjustments to module constants
    moduleConstants
        // Disable encoder offsets
        .withEncoderOffset(0)
        // Disable motor inversions for drive and steer motors
        .withDriveMotorInverted(false)
        .withSteerMotorInverted(false)
        // Disable CanCoder inversion
        .withEncoderInverted(false)
        // Adjust steer motor PID gains for simulation
        .withSteerMotorGains(
            moduleConstants
                .SteerMotorGains
                .withKP(70) // Proportional gain
                .withKD(4.5)) // Derivative gain
        // Adjust friction voltages
        .withDriveFrictionVoltage(Units.Volts.of(0.1))
        .withSteerFrictionVoltage(Units.Volts.of(0.15))
        // Adjust steer inertia
        .withSteerInertia(Units.KilogramSquareMeters.of(0.05));
  }

  /**
   *
   *
   * <h2>Regulates all {@link SwerveModuleConstants} for a drivetrain simulation.</h2>
   *
   * <p>This method processes an array of {@link SwerveModuleConstants} to apply necessary
   * adjustments for simulation purposes, ensuring compatibility and avoiding known bugs.
   *
   * <p>Yoinked from 254;
   * https://github.com/Team254/FRC-2025-Public/blob/main/src/main/java/com/team254/frc2025/utils/simulations/MapleSimSwerveDrivetrain.java
   *
   * @see #regulateModuleConstantForSimulation(SwerveModuleConstants)
   */
  public static DrivetrainConfiguration regulateModuleConstantForSimulation(
      DrivetrainConfiguration drivetrainConfiguration) {
    // Skip regulation if running on a real robot
    if (RobotBase.isReal()) return null;

    // Apply simulation-specific adjustments to module constants
    for (SwerveModuleConstants<?, ?, ?> moduleConstants :
        drivetrainConfiguration.ctreModuleConstants) {
      regulateModuleConstantForSimulation(moduleConstants);
    }
    return drivetrainConfiguration;
  }
}
