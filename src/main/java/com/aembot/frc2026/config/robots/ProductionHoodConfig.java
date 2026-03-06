package com.aembot.frc2026.config.robots;

import com.aembot.frc2026.constants.RobotRuntimeConstants;
import com.aembot.lib.config.motors.MotorConfiguration;
import com.aembot.lib.config.motors.SimulatedMotorConfiguration;
import com.aembot.lib.config.subsystems.hood.TalonFXHoodConfiguration;
import com.aembot.lib.config.subsystems.hood.simulation.SimulatedHoodConfiguration;
import com.aembot.lib.config.wrappers.ConfigureSlot0Gains;
import com.aembot.lib.constants.RuntimeConstants.RuntimeMode;
import com.aembot.lib.core.can.CANDeviceID;
import com.aembot.lib.core.motors.interfaces.MotorIO.NeutralMode;
import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.signals.InvertedValue;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.util.Units;

public class ProductionHoodConfig {

  public final double GEAR_RATIO = 144;

  public final double CRUISE_VELOCITY_DEG_PER_SEC = 360;

  public final double ACCELERATION_DEG_PER_SEC = 720;

  public final String SUBSYSTEM_NAME = "HoodSubsystem";

  public final double AUTO_AIM_LENIANCY = 2;

  public final Pose3d HOOD_ORIGIN_POSE =
      new Pose3d(0.134944, 0.000127, 0.083591, new Rotation3d(0, Units.degreesToRadians(20), 0));

  public final Translation3d GAMEPIECE_EXIT_POINT_FROM_TURRET = new Translation3d();

  public final boolean MOTOR_INVERTED = false;

  public final NeutralMode MOTOR_NEUTRAL_MODE = NeutralMode.BRAKE;

  public final double HARDSTOP_POS_DEGREES = 68;

  public final ConfigureSlot0Gains SLOT_0_CONFIGS =
      (RobotRuntimeConstants.MODE == RuntimeMode.REAL)
          ? new ConfigureSlot0Gains(0.0, 0.0, 0.0, 0.0, 0.4, 0.98, 0.0)
          : new ConfigureSlot0Gains(0.05, 0.0, 0.0, 0.0, 0.0, 0.12, 0.0);

  public final MotorConfiguration<TalonFXConfiguration> MOTOR_CONFIG =
      new MotorConfiguration<TalonFXConfiguration>()
          .withMotorConfig(
              new TalonFXConfiguration()
                  .withMotionMagic(
                      new MotionMagicConfigs()
                          .withMotionMagicCruiseVelocity(
                              Units.degreesToRotations(CRUISE_VELOCITY_DEG_PER_SEC) * GEAR_RATIO)
                          .withMotionMagicAcceleration(
                              Units.degreesToRotations(ACCELERATION_DEG_PER_SEC) * GEAR_RATIO))
                  .withSlot0(SLOT_0_CONFIGS)
                  .withMotorOutput(
                      new MotorOutputConfigs()
                          .withInverted(
                              MOTOR_INVERTED
                                  ? InvertedValue.CounterClockwise_Positive
                                  : InvertedValue.Clockwise_Positive)
                          .withNeutralMode(MOTOR_NEUTRAL_MODE.toCTRENeutralMode()))
                    .withCurrentLimits(new CurrentLimitsConfigs().withSupplyCurrentLimit(30)))
          .withCANDevice(
              new CANDeviceID(
                  57, SUBSYSTEM_NAME + "Motor", SUBSYSTEM_NAME, CANDeviceID.CANDeviceType.TALON_FX))
          .withName(SUBSYSTEM_NAME + "Motor")
          .withUnitToRotorRotationRatio(Units.rotationsToDegrees(1 / GEAR_RATIO))
          .withMaxPositionUnits(HARDSTOP_POS_DEGREES)
          .withMinPositionUnits(30);

  public final SimulatedMotorConfiguration<TalonFXConfiguration> SIM_MOTOR_CONFIG =
      new SimulatedMotorConfiguration<TalonFXConfiguration>()
          .withRealConfiguration(MOTOR_CONFIG)
          .withStartingRotation(30)
          .withSimMotorConstants(DCMotor.getKrakenX60(1));

  public final TalonFXHoodConfiguration HOOD_CONFIG =
      new TalonFXHoodConfiguration(MOTOR_CONFIG, SUBSYSTEM_NAME)
          .withHoodOriginPose(HOOD_ORIGIN_POSE)
          .withGamePieceExitPoint(GAMEPIECE_EXIT_POINT_FROM_TURRET)
          .withUpwardsHardStopUnits(HARDSTOP_POS_DEGREES)
          .withAutoAimLeniance(AUTO_AIM_LENIANCY);

  public final SimulatedHoodConfiguration SIMULATED_HOOD_CONFIG =
      new SimulatedHoodConfiguration(SIM_MOTOR_CONFIG, SUBSYSTEM_NAME);
}
