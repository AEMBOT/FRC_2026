package com.aembot.frc2026.config.robots;

import com.aembot.frc2026.constants.RobotRuntimeConstants;
import com.aembot.lib.config.motors.MotorConfiguration;
import com.aembot.lib.config.motors.SimulatedMotorConfiguration;
import com.aembot.lib.config.subsystems.flywheel.TalonFXFlywheelConfiguration;
import com.aembot.lib.config.wrappers.ConfigureSlot0Gains;
import com.aembot.lib.constants.RuntimeConstants.RuntimeMode;
import com.aembot.lib.core.can.CANDeviceID;
import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.util.Units;

public class ProductionFlywheelConfig {
  //  TODO Placeholder values, values need to be changed cuz they don't make sense
  public static final double CRUISE_VELOCITY_METERS_PER_SEC = 16.0;
  public static final double ACCELERATION_METERS_PER_SEC = 32.0;
  public static final double JERK = 0.0;

  public static final double VELOCITY_TOLERANCE_METERS_PER_SEC = 0.2;

  public static final double GEAR_RATIO = 1.0;

  public static final double J_KG_METERS_SQ = 0.01;

  public static final double SHOOTER_WHEEL_RADIUS = Units.inchesToMeters(2.0);
  public static final double SHOOTER_WHEEL_CIRCUMFERENCE = 2.0 * Math.PI * SHOOTER_WHEEL_RADIUS;

  public static final double UNITS_TO_ROTOR_RATIO = SHOOTER_WHEEL_CIRCUMFERENCE / GEAR_RATIO;
  public static final double UNITS_TO_MECHANISM_ROTATION_RATIO = SHOOTER_WHEEL_CIRCUMFERENCE;

  public static final ConfigureSlot0Gains MOTOR_GAINS =
      (RobotRuntimeConstants.MODE == RuntimeMode.REAL)
          ? new ConfigureSlot0Gains(0.0, 0.0, 0.0, 0.0, 0.4, 0.132, 0.0)
          : new ConfigureSlot0Gains(5, 0, 0, 0, 0, 0.1, 0);

  public static final int MOTOR_ID = 53;
  public static final String MOTOR_NAME = "FlywheelMotor";
  public static final double MOTOR_CURRENT_LIMIT = 40.0;

  public static final String FLYWHEEL_SUBSYTEM_NAME = "FlywheelSubsystem";

  public final double AUTO_AIM_LENCIANCY = 0.7;

  public final MotorConfiguration<TalonFXConfiguration> MOTOR_CONFIG =
      new MotorConfiguration<TalonFXConfiguration>()
          .withMotorConfig(
              new TalonFXConfiguration()
                  .withMotorOutput(
                      new MotorOutputConfigs()
                          .withNeutralMode(NeutralModeValue.Coast)
                          .withInverted(InvertedValue.Clockwise_Positive))
                  .withCurrentLimits(
                      new CurrentLimitsConfigs()
                          .withSupplyCurrentLimit(MOTOR_CURRENT_LIMIT)
                          .withSupplyCurrentLimitEnable(true))
                  .withSlot0(MOTOR_GAINS)
                  .withMotionMagic(
                      new MotionMagicConfigs()
                          .withMotionMagicCruiseVelocity(
                              CRUISE_VELOCITY_METERS_PER_SEC / UNITS_TO_ROTOR_RATIO)
                          .withMotionMagicAcceleration(
                              ACCELERATION_METERS_PER_SEC / UNITS_TO_ROTOR_RATIO)
                          .withMotionMagicJerk(JERK)))
          .withCANDevice(
              new CANDeviceID(
                  MOTOR_ID, MOTOR_NAME, FLYWHEEL_SUBSYTEM_NAME, CANDeviceID.CANDeviceType.TALON_FX))
          .withName(FLYWHEEL_SUBSYTEM_NAME + "Motor")
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
          .withSpeedToleranceUnitsPerSecond(VELOCITY_TOLERANCE_METERS_PER_SEC)
          .withRealMotorConfig(MOTOR_CONFIG)
          .withSimulatedMotorConfig(SIM_MOTOR_CONFIG)
          .withAutoAimLeniance(AUTO_AIM_LENCIANCY);
}
