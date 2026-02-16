package com.aembot.frc2026.config.robots;

import com.aembot.frc2026.config.subsystems.TalonFXTurretConfiguration;
import com.aembot.lib.config.encoders.AEMCANCoderConfiguration;
import com.aembot.lib.config.motors.MotorConfiguration;
import com.aembot.lib.config.motors.SimulatedMotorConfiguration;
import com.aembot.lib.core.can.CANDeviceID;
import com.aembot.lib.core.can.CANDeviceID.CANDeviceType;
import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.util.Units;

public class ProductionTurretConfig {

  public final String SUBSYSTEM_NAME = "TurretSubsystem";

  public final double CRUISE_VELOCITY_DEG_PER_SEC = 180;

  public final double ACCELERATION_DEG_PER_SEC = 360;

  public final double GEAR_RATIO = 80;

  public final int MOTOR_ID = 18;

  public final int CANCODER_A_ID = 19;

  public final int CANCODER_B_ID = 19;

  public final AEMCANCoderConfiguration CANCODER_A_CONFIG =
      new AEMCANCoderConfiguration()
          .withDevice(
              new CANDeviceID(
                  CANCODER_A_ID,
                  SUBSYSTEM_NAME + "CANCoderA",
                  SUBSYSTEM_NAME,
                  CANDeviceType.CANCODER))
          .withConfiguration(new CANcoderConfiguration());

  public final int CANCODER_A_GEAR_TEETH = 17;

  public final AEMCANCoderConfiguration CANCODER_B_CONFIG =
      new AEMCANCoderConfiguration()
          .withDevice(
              new CANDeviceID(
                  CANCODER_B_ID,
                  SUBSYSTEM_NAME + "CANCoderB",
                  SUBSYSTEM_NAME,
                  CANDeviceType.CANCODER))
          .withConfiguration(new CANcoderConfiguration());

  public final int CANCODER_B_GEAR_TEETH = 13;

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
                  .withSlot0(new Slot0Configs().withKP(.1).withKV(.1)))
          .withCANDevice(
              new CANDeviceID(
                  MOTOR_ID, SUBSYSTEM_NAME + "Motor", SUBSYSTEM_NAME, CANDeviceType.TALON_FX))
          .withName(SUBSYSTEM_NAME + "Motor")
          .withUnitToRotorRotationRatio(Units.rotationsToDegrees(1 / GEAR_RATIO))
          .withMaxPositionUnits(380)
          .withMinPositionUnits(-20);

  public final SimulatedMotorConfiguration<TalonFXConfiguration> SIM_MOTOR_CONFIG =
      new SimulatedMotorConfiguration<TalonFXConfiguration>()
          .withRealConfiguration(MOTOR_CONFIG)
          .withStartingRotation(180)
          .withSimMotorConstants(DCMotor.getKrakenX60(1));

  public final TalonFXTurretConfiguration TURRET_CONFIG =
      new TalonFXTurretConfiguration(SUBSYSTEM_NAME)
          .withCANcoderAConfig(CANCODER_A_CONFIG)
          .withCANcoderAGearTeeth(CANCODER_A_GEAR_TEETH)
          .withCANcoderBConfig(CANCODER_B_CONFIG)
          .withCANcoderBGearTeeth(CANCODER_B_GEAR_TEETH)
          .withRealMotorConfig(MOTOR_CONFIG)
          .withSimMotorConfig(SIM_MOTOR_CONFIG);
}
