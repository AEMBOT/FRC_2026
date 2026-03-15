package com.aembot.frc2026.subsystems;

import com.aembot.frc2026.constants.RobotRuntimeConstants;
import com.aembot.frc2026.constants.field.Field2026;
import com.aembot.frc2026.state.RobotStateYearly;
import com.aembot.frc2026.state.SimulatedRobotStateYearly;
import com.aembot.frc2026.subsystems.indexerKicker.IndexerKickerSubsystem;
import com.aembot.frc2026.subsystems.indexerKicker.io.IndexerKickerMechanismIOReal;
import com.aembot.frc2026.subsystems.indexerKicker.io.IndexerKickerMechanismIOReplay;
import com.aembot.frc2026.subsystems.indexerKicker.io.IndexerKickerMechanismIOSim;
import com.aembot.frc2026.subsystems.indexerSelector.IndexerSelectorSubsystem;
import com.aembot.frc2026.subsystems.indexerSelector.io.IndexerSelectorMechanismIOReal;
import com.aembot.frc2026.subsystems.indexerSelector.io.IndexerSelectorMechanismIOReplay;
import com.aembot.frc2026.subsystems.indexerSelector.io.IndexerSelectorMechanismIOSim;
import com.aembot.frc2026.subsystems.spindexer.SpindexerSubsystem;
import com.aembot.frc2026.subsystems.spindexer.io.SpindexerMechanismIOReplay;
import com.aembot.frc2026.subsystems.turret.TurretSubsystem;
import com.aembot.frc2026.subsystems.turret.io.TalonFXTurretHardwareIO;
import com.aembot.frc2026.subsystems.turret.io.TurretReplayIO;
import com.aembot.frc2026.subsystems.turret.io.TurretSimIO;
import com.aembot.lib.config.motors.MotorFollowersConfiguration;
import com.aembot.lib.config.subsystems.vision.CameraConfiguration;
import com.aembot.lib.config.subsystems.vision.SimulatedCameraConfiguration;
import com.aembot.lib.constants.fields.YearFieldConstantable;
import com.aembot.lib.core.motors.interfaces.MotorIO;
import com.aembot.lib.core.motors.io.MotorIOReplay;
import com.aembot.lib.core.motors.io.MotorIOTalonFX;
import com.aembot.lib.core.motors.io.MotorIOTalonFXSim;
import com.aembot.lib.core.motors.io.containers.CompoundMotorIO;
import com.aembot.lib.core.motors.io.containers.CompoundMotorIOReal;
import com.aembot.lib.core.motors.io.containers.CompoundMotorIOReplay;
import com.aembot.lib.core.motors.io.containers.CompoundMotorIOSim;
import com.aembot.lib.core.sensors.timeOfFlight.io.TimeOfFlightIOCANRange;
import com.aembot.lib.core.sensors.timeOfFlight.io.TimeOfFlightIOReplay;
import com.aembot.lib.core.sensors.timeOfFlight.io.TimeOfFlightSimIOCanRange;
import com.aembot.lib.subsystems.aprilvision.AprilVisionSubsystem;
import com.aembot.lib.subsystems.aprilvision.interfaces.AprilCameraIO;
import com.aembot.lib.subsystems.aprilvision.io.AprilCameraReplayIO;
import com.aembot.lib.subsystems.aprilvision.io.Limelight4IOHardware;
import com.aembot.lib.subsystems.aprilvision.io.Limelight4IOSim;
import com.aembot.lib.subsystems.drive.DriveSubsystem;
import com.aembot.lib.subsystems.drive.io.DrivetrainHardwareIO;
import com.aembot.lib.subsystems.drive.io.DrivetrainIOReplay;
import com.aembot.lib.subsystems.drive.io.DrivetrainSimIO;
import com.aembot.lib.subsystems.flywheel.FlywheelSubsystem;
import com.aembot.lib.subsystems.flywheel.io.FlywheelHardwareIO;
import com.aembot.lib.subsystems.flywheel.io.FlywheelReplayIO;
import com.aembot.lib.subsystems.flywheel.io.FlywheelSimIO;
import com.aembot.lib.subsystems.hood.HoodSubsystem;
import com.aembot.lib.subsystems.hood.io.HoodIOReplay;
import com.aembot.lib.subsystems.hood.io.HoodSimIO;
import com.aembot.lib.subsystems.hood.io.TalonFXHoodHardwareIO;
import com.aembot.lib.subsystems.intake.generic.multimotor.IntakeRollerMultiMotorSubsystem;
import com.aembot.lib.subsystems.intake.over_bumper.deploy.OverBumperIntakeDeploySubsystem;
import com.aembot.lib.subsystems.intake.over_bumper.deploy.io.OverBumperIntakeDeployReplayIO;
import com.aembot.lib.subsystems.intake.over_bumper.deploy.io.OverBumperIntakeDeploySimIO;
import com.aembot.lib.subsystems.intake.over_bumper.deploy.io.TalonFXOverBumperIntakeDeployHardwareIO;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform3d;
import java.util.ArrayList;
import java.util.List;
import java.util.function.BiFunction;
import org.photonvision.simulation.PhotonCameraSim;
import org.photonvision.simulation.VisionSystemSim;

/** Subsystem factories intended to be called from RobotContainer */
public class SubsystemFactory {
  public static DriveSubsystem createDriveSubsystem() {
    switch (RobotRuntimeConstants.MODE) {
      case SIM:
        var io =
            new DrivetrainSimIO(
                RobotRuntimeConstants.ROBOT_CONFIG.getSimulatedDrivetrainConfiguration(),
                RobotRuntimeConstants.ROBOT_CONFIG.getDrivetrainConfiguration(),
                RobotRuntimeConstants.ROBOT_CONFIG.getSwerveConfigurations());

        SimulatedRobotStateYearly.get()
            .simulatedIntakeState
            .setDriveSim(io.drivetrainSim.mapleSimSwerveDrivetrain);

        return new DriveSubsystem(
                RobotRuntimeConstants.ROBOT_CONFIG.getDrivetrainConfiguration(),
                io,
                RobotStateYearly.get())
            .withSetPose(new Pose2d(2.5, 4, Rotation2d.fromDegrees(0)));
      case REPLAY:
        return new DriveSubsystem(
            RobotRuntimeConstants.ROBOT_CONFIG.getDrivetrainConfiguration(),
            new DrivetrainIOReplay(),
            RobotStateYearly.get());
      case REAL:
      default:
        return new DriveSubsystem(
            RobotRuntimeConstants.ROBOT_CONFIG.getDrivetrainConfiguration(),
            new DrivetrainHardwareIO(
                RobotRuntimeConstants.ROBOT_CONFIG.getDrivetrainConfiguration(),
                RobotRuntimeConstants.ROBOT_CONFIG.getSwerveConfigurations()),
            RobotStateYearly.get());
    }
  }

  public static HoodSubsystem createHoodSubsystem() {

    switch (RobotRuntimeConstants.MODE) {
      case SIM:
        return new HoodSubsystem(
            RobotRuntimeConstants.ROBOT_CONFIG.getSimHoodConfig(),
            new HoodSimIO(RobotRuntimeConstants.ROBOT_CONFIG.getSimHoodConfig()),
            RobotStateYearly.get().hoodState);

      case REPLAY:
        return new HoodSubsystem(
            RobotRuntimeConstants.ROBOT_CONFIG.getHoodConfig(),
            new HoodIOReplay(),
            RobotStateYearly.get().hoodState);
      case REAL:

      default:
        return new HoodSubsystem(
            RobotRuntimeConstants.ROBOT_CONFIG.getHoodConfig(),
            new TalonFXHoodHardwareIO(RobotRuntimeConstants.ROBOT_CONFIG.getHoodConfig()),
            RobotStateYearly.get().hoodState);
    }
  }

  public static SpindexerSubsystem createSpindexerSubsystem() {
    var spindexerConfig = RobotRuntimeConstants.ROBOT_CONFIG.getSpindexerConfiguration();
    var indexerCompoundState = RobotStateYearly.get().indexerCompoundState;

    switch (RobotRuntimeConstants.MODE) {
      case SIM:
        return new SpindexerSubsystem(
            spindexerConfig,
            new SpindexerMechanismIOReplay(), // TODO remove spindexer more cleanly
            indexerCompoundState::getSpindexerCommandedState);
      case REPLAY:
        return new SpindexerSubsystem(
            spindexerConfig,
            new SpindexerMechanismIOReplay(),
            indexerCompoundState::getSpindexerCommandedState);
      case REAL:
      default:
        return new SpindexerSubsystem(
            spindexerConfig,
            new SpindexerMechanismIOReplay(),
            indexerCompoundState::getSpindexerCommandedState);
    }
  }

  public static IndexerSelectorSubsystem createIndexerSelectorSubsystem() {
    var selectorConfig = RobotRuntimeConstants.ROBOT_CONFIG.getIndexerSelectorConfiguration();
    var indexerCompoundState = RobotStateYearly.get().indexerCompoundState;

    switch (RobotRuntimeConstants.MODE) {
      case SIM:
        var timeOfFlightIO = new TimeOfFlightSimIOCanRange(selectorConfig.kTimeOfFlightConfig);

        SimulatedRobotStateYearly.get()
            .simulatedIndexerCompoundState
            .setSelectorTimeOfFlightDistanceConsumer(timeOfFlightIO::setSimulatedDistance);

        return new IndexerSelectorSubsystem(
            selectorConfig,
            new IndexerSelectorMechanismIOSim(selectorConfig),
            timeOfFlightIO,
            indexerCompoundState::getSelectorCommandedState,
            indexerCompoundState::updateGamePieceInSelector);
      case REPLAY:
        return new IndexerSelectorSubsystem(
            selectorConfig,
            new IndexerSelectorMechanismIOReplay(),
            new TimeOfFlightIOReplay(selectorConfig.kTimeOfFlightConfig),
            indexerCompoundState::getSelectorCommandedState,
            indexerCompoundState::updateGamePieceInSelector);
      case REAL:
      default:
        return new IndexerSelectorSubsystem(
            selectorConfig,
            new IndexerSelectorMechanismIOReal(selectorConfig),
            new TimeOfFlightIOCANRange(selectorConfig.kTimeOfFlightConfig),
            indexerCompoundState::getSelectorCommandedState,
            indexerCompoundState::updateGamePieceInSelector);
    }
  }

  public static IndexerKickerSubsystem createIndexerKickerSubsystem() {
    var kickerConfig = RobotRuntimeConstants.ROBOT_CONFIG.getIndexerKickerConfiguration();
    var indexerCompoundState = RobotStateYearly.get().indexerCompoundState;

    switch (RobotRuntimeConstants.MODE) {
      case SIM:
        return new IndexerKickerSubsystem(
            kickerConfig,
            new IndexerKickerMechanismIOSim(kickerConfig),
            indexerCompoundState::getKickerCommandedState);
      case REPLAY:
        return new IndexerKickerSubsystem(
            kickerConfig,
            new IndexerKickerMechanismIOReplay(),
            indexerCompoundState::getKickerCommandedState);
      case REAL:
      default:
        return new IndexerKickerSubsystem(
            kickerConfig,
            new IndexerKickerMechanismIOReal(kickerConfig),
            indexerCompoundState::getKickerCommandedState);
    }
  }

  public static OverBumperIntakeDeploySubsystem createIntakeDeploySubsystem() {
    switch (RobotRuntimeConstants.MODE) {
      case SIM:
        return new OverBumperIntakeDeploySubsystem(
            RobotRuntimeConstants.ROBOT_CONFIG.getIntakeDeployConfig(),
            new OverBumperIntakeDeploySimIO(
                RobotRuntimeConstants.ROBOT_CONFIG.getIntakeDeployConfig()),
            (state) -> RobotStateYearly.get().updateIntakeDeployState(state));
      case REPLAY:
        return new OverBumperIntakeDeploySubsystem(
            RobotRuntimeConstants.ROBOT_CONFIG.getIntakeDeployConfig(),
            new OverBumperIntakeDeployReplayIO(),
            (state) -> RobotStateYearly.get().updateIntakeDeployState(state));
      case REAL:

      default:
        return new OverBumperIntakeDeploySubsystem(
            RobotRuntimeConstants.ROBOT_CONFIG.getIntakeDeployConfig(),
            new TalonFXOverBumperIntakeDeployHardwareIO(
                RobotRuntimeConstants.ROBOT_CONFIG.getIntakeDeployConfig()),
            (state) -> RobotStateYearly.get().updateIntakeDeployState(state));
    }
  }

  public static IntakeRollerMultiMotorSubsystem createIntakeRollerSubsystem() {
    // switch (RobotRuntimeConstants.MODE) {
    //   case SIM:
    //     return new IntakeRollerMultiMotorSubsystem(
    //         RobotRuntimeConstants.ROBOT_CONFIG.getIntakeRollerConfig(),
    //         new IntakeRollerSimIO(RobotRuntimeConstants.ROBOT_CONFIG.getIntakeRollerConfig()),
    //         (state) -> RobotStateYearly.get().updateIntakeRollerState(state));
    //   case REPLAY:
    //     return new IntakeRollerMultiMotorSubsystem(
    //         RobotRuntimeConstants.ROBOT_CONFIG.getIntakeRollerConfig(),
    //         new IntakeRollerReplayIO(),
    //         (state) -> RobotStateYearly.get().updateIntakeRollerState(state));
    //   case REAL:

    //   default:
    //     return new IntakeRollerMultiMotorSubsystem(
    //         RobotRuntimeConstants.ROBOT_CONFIG.getIntakeRollerConfig(),
    //         new TalonFXIntakeRollerHardwareIO(
    //             RobotRuntimeConstants.ROBOT_CONFIG.getIntakeRollerConfig()),
    //         (state) -> RobotStateYearly.get().updateIntakeRollerState(state));
    // }
    ArrayList<MotorIO> motorIOs = new ArrayList<>();

    /* -- ADD FOLLOWER(S) -- */
    for (MotorFollowersConfiguration.FollowerConfiguration<TalonFXConfiguration> conf :
        RobotRuntimeConstants.ROBOT_CONFIG.getIntakeRollerConfig()
            .kMotorConfigs
            .followerConfigurations) {
      switch (RobotRuntimeConstants.MODE) {
        case SIM:
          motorIOs.add(new MotorIOTalonFXSim(conf.simConfig));
          break;
        case REPLAY:
          motorIOs.add(new MotorIOReplay());
          break;
        default:
        case REAL:
          motorIOs.add(new MotorIOTalonFX(conf.config));
          break;
      }
    }

    CompoundMotorIO<MotorIO> ioContainer;

    /* -- ADD LEADER & BUILD IO CONTAINER -- */
    switch (RobotRuntimeConstants.MODE) {
      case SIM:
        motorIOs.add(
            0,
            new MotorIOTalonFXSim(
                RobotRuntimeConstants.ROBOT_CONFIG.getIntakeRollerConfig()
                    .kMotorConfigs
                    .leaderSimConfig));
        ioContainer = new CompoundMotorIOSim<>(motorIOs);
        break;
      case REPLAY:
        motorIOs.add(0, new MotorIOReplay());
        ioContainer = new CompoundMotorIOReplay<>(motorIOs);
        break;
      default:
      case REAL:
        motorIOs.add(
            0,
            new MotorIOTalonFX(
                RobotRuntimeConstants.ROBOT_CONFIG.getIntakeRollerConfig()
                    .kMotorConfigs
                    .leaderConfig));
        ioContainer = new CompoundMotorIOReal<>(motorIOs);
        break;
    }

    /* -- MAKE ACTUAL SUBSYSTEM -- */
    return new IntakeRollerMultiMotorSubsystem(
        RobotRuntimeConstants.ROBOT_CONFIG.getIntakeRollerConfig(),
        RobotStateYearly.get().intakeRollerState,
        ioContainer);
  }

  public static AprilVisionSubsystem createAprilVisionSubsystem() {
    List<CameraConfiguration> configs =
        RobotRuntimeConstants.ROBOT_CONFIG.getCameraConfigurations();
    List<SimulatedCameraConfiguration> simConfigs =
        RobotRuntimeConstants.ROBOT_CONFIG.getSimulatedCameraConfigurations();

    BiFunction<PhotonCameraSim, Transform3d, VisionSystemSim> simCameraRegistrar =
        (cameraSim, cameraTransform) ->
            SimulatedRobotStateYearly.get().addCameraToVisionSimulation(cameraSim, cameraTransform);

    YearFieldConstantable fieldConstants = new Field2026();

    AprilCameraIO[] cameraIOs = new AprilCameraIO[configs.size()];

    for (int i = 0; i < cameraIOs.length; i++) {
      switch (RobotRuntimeConstants.MODE) {
        case SIM:
          cameraIOs[i] =
              new Limelight4IOSim(
                  simConfigs.get(i), fieldConstants, RobotStateYearly.get(), simCameraRegistrar);
          break;
        case REPLAY:
          cameraIOs[i] = new AprilCameraReplayIO(configs.get(i));
          break;
        default:
        case REAL:
          cameraIOs[i] =
              new Limelight4IOHardware(configs.get(i), fieldConstants, RobotStateYearly.get());
          break;
      }
    }

    return new AprilVisionSubsystem(RobotStateYearly.get(), cameraIOs);
  }

  public static FlywheelSubsystem createFlywheelSubsystem() {
    switch (RobotRuntimeConstants.MODE) {
      case SIM:
        return new FlywheelSubsystem(
            RobotRuntimeConstants.ROBOT_CONFIG.getFlywheelConfiguration(),
            new FlywheelSimIO(RobotRuntimeConstants.ROBOT_CONFIG.getFlywheelConfiguration()),
            RobotStateYearly.get().shooterFlywheelState);
      case REPLAY:
        return new FlywheelSubsystem(
            RobotRuntimeConstants.ROBOT_CONFIG.getFlywheelConfiguration(),
            new FlywheelReplayIO(),
            RobotStateYearly.get().shooterFlywheelState);
      case REAL:

      default:
        return new FlywheelSubsystem(
            RobotRuntimeConstants.ROBOT_CONFIG.getFlywheelConfiguration(),
            new FlywheelHardwareIO(RobotRuntimeConstants.ROBOT_CONFIG.getFlywheelConfiguration()),
            RobotStateYearly.get().shooterFlywheelState);
    }
  }

  public static TurretSubsystem createTurretSubsystem() {
    switch (RobotRuntimeConstants.MODE) {
      case SIM:
        return new TurretSubsystem(
            RobotRuntimeConstants.ROBOT_CONFIG.getTurretConfig(),
            new TurretSimIO(RobotRuntimeConstants.ROBOT_CONFIG.getTurretConfig()),
            RobotStateYearly.get().turretState);
      case REPLAY:
        return new TurretSubsystem(
            RobotRuntimeConstants.ROBOT_CONFIG.getTurretConfig(),
            new TurretReplayIO(),
            RobotStateYearly.get().turretState);
      case REAL:

      default:
        return new TurretSubsystem(
            RobotRuntimeConstants.ROBOT_CONFIG.getTurretConfig(),
            new TalonFXTurretHardwareIO(RobotRuntimeConstants.ROBOT_CONFIG.getTurretConfig()),
            RobotStateYearly.get().turretState);
    }
  }
}
