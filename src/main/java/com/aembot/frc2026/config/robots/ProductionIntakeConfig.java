package com.aembot.frc2026.config.robots;

import com.aembot.lib.config.motors.MotorConfiguration;
import com.aembot.lib.config.motors.SimulatedMotorConfiguration;
import com.aembot.lib.config.subsystems.intake.overBumper.deploy.TalonFXOverBumperIntakeDeployConfiguration;
import com.aembot.lib.config.subsystems.intake.overBumper.run.TalonFXOverBumperIntakeRollerConfiguration;
import com.aembot.lib.core.can.CANDeviceID;
import com.aembot.lib.core.motors.interfaces.MotorIO.NeutralMode;
import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.signals.InvertedValue;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.util.Units;
import org.ironmaple.simulation.IntakeSimulation.IntakeSide;

public class ProductionIntakeConfig {

  public final int DEPLOY_CAN_ID = 51;

  public final int ROLLER_CAN_ID = 50;

  public final double UP_DEPLOY_ANGLE = 92.897821;

  public final double DOWN_DEPLOY_ANGLE = 8.206357; // Less temporary

  public final double STARTING_ANGLE_DEG = 131.67;

  public final double ZERO_ANGLE_DEG = 0;

  public final boolean DEPLOY_MOTOR_INVERTED = true;

  public final NeutralMode DEPLOY_NEUTRAL_MODE = NeutralMode.BRAKE;

  public final double ZEROING_VOLTAGE = 2.0;

  public final String SUBSYSTEM_NAME = "IntakeSubsystem";

  public final double DEPLOY_GEAR_RATIO = 18400.0 / 243.0;

  public final double DEPLOY_CRUISE_VELOCITY_DEG_PER_SEC = 90;

  public final double DEPLOY_ACCELERATION_DEG_PER_SEC = 180;

  public final double ROLLER_GEAR_RATIO = 1;

  public final double ROLLER_VOLTAGE = 8;

  public final NeutralMode ROLLER_NEUTRAL_MODE = NeutralMode.BRAKE;

  public final IntakeSide DEPLOY_SIDE = IntakeSide.FRONT;

  public final double DEPLOY_EXTENSION_METERS = Units.inchesToMeters(11);

  public final double INTAKE_WIDTH_METERS = Units.inchesToMeters(26.75);

  public final Pose3d DEPLOY_PIVOT_POINT =
      new Pose3d(0.298443, 0, 0.189832, new Rotation3d(0, Math.PI / 2, 0));

  public final MotorConfiguration<TalonFXConfiguration> DEPLOY_MOTOR_CONFIG =
      new MotorConfiguration<TalonFXConfiguration>()
          .withMotorConfig(
              new TalonFXConfiguration()
                  .withMotionMagic(
                      new MotionMagicConfigs()
                          .withMotionMagicCruiseVelocity(
                              Units.degreesToRotations(DEPLOY_CRUISE_VELOCITY_DEG_PER_SEC)
                                  * DEPLOY_GEAR_RATIO)
                          .withMotionMagicAcceleration(
                              Units.degreesToRotations(DEPLOY_ACCELERATION_DEG_PER_SEC)
                                  * DEPLOY_GEAR_RATIO))
                  // constants copies from hood config
                  .withSlot0(new Slot0Configs().withKP(1).withKV(0.121).withKS(0.375))
                  .withMotorOutput(
                      new MotorOutputConfigs()
                          .withInverted(
                              DEPLOY_MOTOR_INVERTED
                                  ? InvertedValue.CounterClockwise_Positive
                                  : InvertedValue.Clockwise_Positive)
                          .withNeutralMode(DEPLOY_NEUTRAL_MODE.toCTRENeutralMode()))
                  .withCurrentLimits(new CurrentLimitsConfigs().withSupplyCurrentLimit(5)))
          .withCANDevice(
              new CANDeviceID(
                  DEPLOY_CAN_ID,
                  SUBSYSTEM_NAME + "DeployMotor",
                  SUBSYSTEM_NAME + "Deploy",
                  CANDeviceID.CANDeviceType.TALON_FX))
          .withName(SUBSYSTEM_NAME + "DeployMotor")
          .withUnitToRotorRotationRatio(Units.rotationsToDegrees(1 / DEPLOY_GEAR_RATIO))
          .withMaxPositionUnits(UP_DEPLOY_ANGLE)
          .withMinPositionUnits(DOWN_DEPLOY_ANGLE);

  public final MotorConfiguration<TalonFXConfiguration> ROLLER_MOTOR_CONFIG =
      new MotorConfiguration<TalonFXConfiguration>()
          .withMotorConfig(
              new TalonFXConfiguration()
                  .withMotorOutput(
                      new MotorOutputConfigs()
                          .withInverted(InvertedValue.Clockwise_Positive)
                          .withNeutralMode(ROLLER_NEUTRAL_MODE.toCTRENeutralMode())))
          .withCANDevice(
              new CANDeviceID(
                  ROLLER_CAN_ID,
                  SUBSYSTEM_NAME + "RollerMotor",
                  SUBSYSTEM_NAME + "Roller",
                  CANDeviceID.CANDeviceType.TALON_FX))
          .withName(SUBSYSTEM_NAME + "RollerMotor")
          .withUnitToRotorRotationRatio(Units.rotationsToDegrees(1 / ROLLER_GEAR_RATIO));

  public final SimulatedMotorConfiguration<TalonFXConfiguration> DEPLOY_SIM_MOTOR_CONFIG =
      new SimulatedMotorConfiguration<TalonFXConfiguration>()
          .withRealConfiguration(DEPLOY_MOTOR_CONFIG)
          .withStartingRotation(STARTING_ANGLE_DEG)
          .withSimMotorConstants(DCMotor.getKrakenX60(1));

  public final SimulatedMotorConfiguration<TalonFXConfiguration> ROLLER_SIM_MOTOR_CONFIG =
      new SimulatedMotorConfiguration<TalonFXConfiguration>()
          .withRealConfiguration(ROLLER_MOTOR_CONFIG)
          .withStartingRotation(0)
          .withSimMotorConstants(DCMotor.getKrakenX60(1));

  public final TalonFXOverBumperIntakeDeployConfiguration DEPLOY_CONFIG =
      new TalonFXOverBumperIntakeDeployConfiguration(SUBSYSTEM_NAME + "Deploy")
          .withRealMotorConfiguration(DEPLOY_MOTOR_CONFIG)
          .withSimulatedMotorConfiguration(DEPLOY_SIM_MOTOR_CONFIG)
          .withZeroingVoltage(ZEROING_VOLTAGE)
          .withIntakeSide(DEPLOY_SIDE)
          .withExtensionMeters(DEPLOY_EXTENSION_METERS)
          .withWidthMeters(INTAKE_WIDTH_METERS)
          .withPivotPoint(DEPLOY_PIVOT_POINT)
          .withInitialAngleDeg(STARTING_ANGLE_DEG)
          .withDownwardsZeroAngleDeg(ZERO_ANGLE_DEG);

  public final TalonFXOverBumperIntakeRollerConfiguration ROLLER_CONFIG =
      new TalonFXOverBumperIntakeRollerConfiguration(SUBSYSTEM_NAME + "Roller")
          .withRealMotorConfiguration(ROLLER_MOTOR_CONFIG)
          .withSimMotorConfiguration(ROLLER_SIM_MOTOR_CONFIG)
          .withIntakeVoltage(ROLLER_VOLTAGE);
}
