package com.aembot.lib.subsystems.drive.io;

import com.aembot.frc2026.state.SimulatedRobotStateYearly;
import com.aembot.lib.config.subsystems.drive.DrivetrainConfiguration;
import com.aembot.lib.config.subsystems.drive.SwerveModuleConfiguration;
import com.aembot.lib.config.subsystems.drive.simulation.DrivetrainSimConfiguration;
import com.aembot.lib.subsystems.drive.simulation.MapleSimSwerveDrivetrain;
import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.swerve.SwerveModuleConstants;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.units.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Notifier;
import edu.wpi.first.wpilibj.Timer;
import java.util.List;
import java.util.function.Consumer;

/**
 * Simulation implementation of the drivetrain, extending the hardware implementation because CTRE's
 * swerve library is cool
 */
public class DrivetrainSimIO extends DrivetrainHardwareIO {
  /** Thread updating the drivetrain maplesim simulation */
  private Notifier simulationThread = null;

  private final DrivetrainSimConfiguration simConfig;
  public MapleSimSwerveDrivetrain drivetrainSim = null;
  private SwerveModuleConstants<?, ?, ?>[] moduleConstants;
  private List<
          SwerveModuleConfiguration<
              TalonFXConfiguration, TalonFXConfiguration, CANcoderConfiguration>>
      moduleConfigurations;

  // Update the swerve drive state for the simulation
  private Consumer<SwerveDriveState> simSwerveStateConsumer =
      state -> {
        if (drivetrainSim != null) {
          state.Pose = drivetrainSim.mapleSimSwerveDrivetrain.getSimulatedDriveTrainPose();
        }
        SimulatedRobotStateYearly.get().updateSimulatedPosition(state.Pose);
        swerveTelemetryConsumer.accept(state);
      };

  public DrivetrainSimIO(
      DrivetrainSimConfiguration simConfig,
      DrivetrainConfiguration driveTrainConfiguration,
      List<
              SwerveModuleConfiguration<
                  TalonFXConfiguration, TalonFXConfiguration, CANcoderConfiguration>>
          swerveModuleConfiguration) {
    super(
        MapleSimSwerveDrivetrain.regulateModuleConstantForSimulation(driveTrainConfiguration),
        swerveModuleConfiguration);

    this.simConfig = simConfig;
    this.moduleConstants = driveTrainConfiguration.ctreModuleConstants;
    this.moduleConfigurations = swerveModuleConfiguration;

    registerTelemetry(simSwerveStateConsumer);
    startSimThread();
  }

  /** Start the simulation thread for the maple sim drive train */
  public void startSimThread() {
    if (simulationThread != null) { // This is _probably_ not needed, but just in case.
      DriverStation.reportWarning(
          "DrivetrainSimIO#startSimThread has been called multiple times."
              + "It will not start the thread multiple times.",
          false);
      return;
    }

    drivetrainSim =
        new MapleSimSwerveDrivetrain(
            Units.Seconds.of(simConfig.simLoopPeriodS),
            Units.Pounds.of(simConfig.physicalConfiguration.robotWeightPounds),
            Units.Meters.of(simConfig.physicalConfiguration.bumperWidthMeters),
            Units.Meters.of(simConfig.physicalConfiguration.bumperLengthMeters),
            DCMotor.getKrakenX60(simConfig.driveMotorsPerModule),
            DCMotor.getKrakenX60(simConfig.steerMotorsPerModule),
            simConfig.physicalConfiguration.wheelCoefficientOfFriction,
            getModuleLocations(),
            getPigeon2(),
            getModules(),
            moduleConstants,
            moduleConfigurations);

    // Create and start simulation thread
    simulationThread = new Notifier(drivetrainSim::update);
    simulationThread.setName("DrivetrainSimNotifier");
    simulationThread.startPeriodic(simConfig.simLoopPeriodS);
  }

  @Override
  public SwerveDriveKinematics getSwerveKinematics() {
    return new SwerveDriveKinematics(getModuleLocations());
  }

  /** Teleports the simulated robot to the given pose. */
  public void teleportRobot(Pose2d pose) {
    if (drivetrainSim != null) {
      drivetrainSim.mapleSimSwerveDrivetrain.setSimulationWorldPose(pose);
      Timer.delay(0.05); // Wait one loop
    }
    super.resetOdometry(pose);
  }

  /**
   * {@inheritDoc}
   *
   * <p><strong>In the sim implementation, this will teleport the simulated robot to the given pose
   */
  @Override
  public void resetOdometry(Pose2d pose) {
    teleportRobot(pose);
  }

  /**
   * Get a reference to the underlying maple sim drive train
   *
   * @return The {@link MapleSimSwerveDrivetrain} running the simulation
   */
  public MapleSimSwerveDrivetrain getMapleSimDrive() {
    return drivetrainSim;
  }
}
