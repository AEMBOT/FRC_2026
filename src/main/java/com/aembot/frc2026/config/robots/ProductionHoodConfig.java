package com.aembot.frc2026.config.robots;

import com.aembot.lib.config.motors.MotorConfiguration;
import com.aembot.lib.config.subsystems.hood.TalonFXHoodConfiguration;
import com.aembot.lib.config.subsystems.hood.simulation.SimulatedHoodConfiguration;
import com.aembot.lib.core.can.CANDeviceID;
import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import edu.wpi.first.math.util.Units;

public class ProductionHoodConfig {

  public final double GEAR_RATIO = 80;

  public final double CURISE_VELOCITY_DEG_PER_SEC = 90;

  public final double ACCELERATION_DEG_PER_SEC = 180;

  public final String SUBSYSTEM_NAME = "HoodSubsystem";

  public final MotorConfiguration<TalonFXConfiguration> MOTOR_CONFIG =
      new MotorConfiguration<TalonFXConfiguration>()
          .withMotorConfig(
              new TalonFXConfiguration()
                  .withMotionMagic(
                      new MotionMagicConfigs()
                          .withMotionMagicCruiseVelocity(
                              Units.degreesToRotations(CURISE_VELOCITY_DEG_PER_SEC) * GEAR_RATIO)
                          .withMotionMagicAcceleration(
                              Units.degreesToRadians(ACCELERATION_DEG_PER_SEC) * GEAR_RATIO))
                              .withSlot0(
                                new Slot0Configs()
                                .withKP(10)))
          .withCANDevice(
              new CANDeviceID(
                  1, SUBSYSTEM_NAME + "Motor", SUBSYSTEM_NAME, CANDeviceID.CANDeviceType.TALON_FX))
          .withName(SUBSYSTEM_NAME + "Motor")
          .withUnitToRotorRotationRatio(Units.rotationsToDegrees(1 / GEAR_RATIO));

  public final TalonFXHoodConfiguration HOOD_CONFIG =
      new TalonFXHoodConfiguration(MOTOR_CONFIG, SUBSYSTEM_NAME);

  public final SimulatedHoodConfiguration SIMULATED_HOOD_CONFIG =
      new SimulatedHoodConfiguration(MOTOR_CONFIG, SUBSYSTEM_NAME);
}
