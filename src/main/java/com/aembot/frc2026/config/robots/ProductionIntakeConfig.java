package com.aembot.frc2026.config.robots;

import com.aembot.lib.config.motors.MotorConfiguration;
import com.aembot.lib.config.motors.SimulatedMotorConfiguration;
import com.aembot.lib.config.subsystems.intake.overBumper.deploy.TalonFXOverBumperIntakeDeployConfiguration;
import com.aembot.lib.config.subsystems.intake.overBumper.run.TalonFXOverBumperIntakeRollerConfiguration;
import com.aembot.lib.core.can.CANDeviceID;
import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.util.Units;

public class ProductionIntakeConfig {

  public final int DEPLOY_CAN_ID = 16;

  public final int ROLLER_CAN_ID = 17;

  public final double MAX_DEPLOY_ANGLE = 90;

  public final double MIN_DEPLOY_ANGLE = 0;

  public final double ZEROING_SPEED = 45;

  public final String SUBSYSTEM_NAME = "IntakeSubsystem";

  public final double DEPLOY_GEAR_RATIO = 18400.0 / 243.0;

  public final double DEPLOY_CRUISE_VELOCITY_DEG_PER_SEC = 90;

  public final double DEPLOY_ACCELERATION_DEG_PER_SEC = 180;

  public final double ROLLER_GEAR_RATIO = 1;

  public final double ROLLER_CRUISE_VELOCITY_ROT_PER_MIN = 200;

  public final double ROLLER_ACCELERATION_ROT_PER_MIN = 400;

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
                  .withSlot0(new Slot0Configs().withKP(.1).withKV(.12)))
          .withCANDevice(
              new CANDeviceID(
                  DEPLOY_CAN_ID,
                  SUBSYSTEM_NAME + "DeployMotor",
                  SUBSYSTEM_NAME + "Deploy",
                  CANDeviceID.CANDeviceType.TALON_FX))
          .withName(SUBSYSTEM_NAME + "DeployMotor")
          .withUnitToRotorRotationRatio(Units.rotationsToDegrees(1 / DEPLOY_GEAR_RATIO))
          .withMaxPositionUnits(MAX_DEPLOY_ANGLE)
          .withMinPositionUnits(MIN_DEPLOY_ANGLE);

  public final MotorConfiguration<TalonFXConfiguration> ROLLER_MOTOR_CONFIG =
      new MotorConfiguration<TalonFXConfiguration>()
          .withMotorConfig(
              new TalonFXConfiguration()
                  .withMotionMagic(
                      new MotionMagicConfigs()
                          .withMotionMagicCruiseVelocity(
                              (ROLLER_CRUISE_VELOCITY_ROT_PER_MIN / 60) * ROLLER_GEAR_RATIO)
                          .withMotionMagicAcceleration(
                              (ROLLER_ACCELERATION_ROT_PER_MIN / 60) * ROLLER_GEAR_RATIO))
                  // constants copies from hood config
                  .withSlot0(new Slot0Configs().withKP(.1).withKV(.12)))
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
          .withStartingRotation(90)
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
          .withZeroingSpeed(ZEROING_SPEED);

  public final TalonFXOverBumperIntakeRollerConfiguration ROLLER_CONFIG =
      new TalonFXOverBumperIntakeRollerConfiguration(SUBSYSTEM_NAME + "Roller")
          .withRealMotorConfiguration(ROLLER_MOTOR_CONFIG)
          .withSimMotorConfiguration(ROLLER_SIM_MOTOR_CONFIG)
          .withTargetSpeed(ROLLER_CRUISE_VELOCITY_ROT_PER_MIN);
}
