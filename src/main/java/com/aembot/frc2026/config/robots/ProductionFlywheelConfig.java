package com.aembot.frc2026.config.robots;

import com.aembot.lib.config.motors.MotorConfiguration;
import com.aembot.lib.config.motors.SimulatedMotorConfiguration;
import com.aembot.lib.config.subsystems.flywheel.TalonFXFlywheelConfiguration;
import com.aembot.lib.config.wrappers.ConfigureSlot0Gains;
import com.aembot.lib.core.can.CANDeviceID;
import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.math.system.plant.DCMotor;

public class ProductionFlywheelConfig {
  //  TODO Placeholder values, values need to be changed cuz they don't make sense

  public final double MAX_VELOCITY = 60.0;
  public final double MAX_ACCELERATION = 10.0;
  public final double JERK = 0.0;

  public final double GEAR_RATIO = 1.0;

  public final double SHOOTER_WHEEL_RADIUS = 0.3;
  public final double SHOOTER_WHEEL_CIRCUMFERENCE = 2.0 * Math.PI * SHOOTER_WHEEL_RADIUS;

  public final double UNITS_TO_ROTOR_RATIO = 1.0 / (SHOOTER_WHEEL_CIRCUMFERENCE * GEAR_RATIO);
  public final double UNITS_TO_MECHANISM_ROTATION_RATIO = 1.0 / SHOOTER_WHEEL_CIRCUMFERENCE;

  public final double J_KG_METERS_SQ = 2.0;

  public final ConfigureSlot0Gains MOTOR_GAINS =
      new ConfigureSlot0Gains(1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0);

  private static final int LEADER_MOTOR_ID = 53;
  private static final String LEADER_MOTOR_NAME = "FlywheelMotor";
  private static final double LEADER_MOTOR_CURRENT_LIMIT = 50.0;

  private static final String FLYWHEEL_SUBSYTEM_NAME = "FlywheelSubsystem";

  public final MotorConfiguration<TalonFXConfiguration> MOTOR_CONFIG =
      new MotorConfiguration<TalonFXConfiguration>()
          .withMotorConfig(
              new TalonFXConfiguration()
                  .withMotorOutput(new MotorOutputConfigs().withNeutralMode(NeutralModeValue.Coast))
                  .withCurrentLimits(
                      new CurrentLimitsConfigs()
                          .withStatorCurrentLimit(LEADER_MOTOR_CURRENT_LIMIT)
                          .withStatorCurrentLimitEnable(true))
                  .withSlot0(MOTOR_GAINS)
                  .withMotionMagic(
                      new MotionMagicConfigs()
                          .withMotionMagicCruiseVelocity(MAX_VELOCITY)
                          .withMotionMagicAcceleration(MAX_ACCELERATION)
                          .withMotionMagicJerk(JERK)))
          .withCANDevice(
              new CANDeviceID(
                  LEADER_MOTOR_ID,
                  LEADER_MOTOR_NAME,
                  FLYWHEEL_SUBSYTEM_NAME,
                  CANDeviceID.CANDeviceType.TALON_FX))
          .withName("FlywheelSubsystem")
          .withUnitToRotorRotationRatio(UNITS_TO_ROTOR_RATIO)
          .withMomentOfInertia(J_KG_METERS_SQ)
          .withUnitToMechanismRotationRatio(UNITS_TO_MECHANISM_ROTATION_RATIO);

  public final SimulatedMotorConfiguration<TalonFXConfiguration> SIM_MOTOR_CONFIG =
      new SimulatedMotorConfiguration<TalonFXConfiguration>()
          .withRealConfiguration(MOTOR_CONFIG)
          .withStartingRotation(0)
          .withSimMotorConstants(DCMotor.getKrakenX60(1));

  public final TalonFXFlywheelConfiguration CONFIG =
      new TalonFXFlywheelConfiguration(FLYWHEEL_SUBSYTEM_NAME)
          .withRealMotorConfig(MOTOR_CONFIG)
          .withSimulatedMotorConfig(SIM_MOTOR_CONFIG);
}
