package com.aembot.frc2026.config.subsystems;

import com.aembot.lib.config.encoders.AEMCANCoderConfiguration;
import com.aembot.lib.config.motors.MotorConfiguration;
import com.aembot.lib.config.motors.SimulatedMotorConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import edu.wpi.first.math.MathUtil;

public class TalonFXTurretConfiguration {

  /** Name of the turret subsystem */
  public final String kName;

  /** Configuration of the real motor */
  public MotorConfiguration<TalonFXConfiguration> kRealMotorConfig;

  /** Configuration of the simulated motor */
  public SimulatedMotorConfiguration<TalonFXConfiguration> kSimMotorConfig;

  /** Configuration of CANcoder A */
  public AEMCANCoderConfiguration kCANcoderAConfig;

  /** Configuration of CANcoder B */
  public AEMCANCoderConfiguration kCANcoderBConfig;

  /** Number of teeth on the gear connecting the rotor to CANcoder A */
  public int kCANcoderAGearTeeth;

  /** Number of teeth on the gear connecting the rotor to CANcoder B */
  public int kCANcoderBGearTeeth;

  /**
   * Create a new turret configuration
   *
   * @param name Name of the turret subsystem
   */
  public TalonFXTurretConfiguration(String name) {
    this.kName = name;
  }

  public TalonFXTurretConfiguration withRealMotorConfig(
      MotorConfiguration<TalonFXConfiguration> realMotorConfig) {
    this.kRealMotorConfig = realMotorConfig;
    return this;
  }

  public TalonFXTurretConfiguration withSimMotorConfig(
      SimulatedMotorConfiguration<TalonFXConfiguration> simMotorConfig) {
    this.kSimMotorConfig = simMotorConfig;
    return this;
  }

  public TalonFXTurretConfiguration withCANcoderAConfig(AEMCANCoderConfiguration CANcoderAConfig) {
    this.kCANcoderAConfig = CANcoderAConfig;
    return this;
  }

  public TalonFXTurretConfiguration withCANcoderBConfig(AEMCANCoderConfiguration CANcoderBConfig) {
    this.kCANcoderBConfig = CANcoderBConfig;
    return this;
  }

  public TalonFXTurretConfiguration withCANcoderAGearTeeth(int CANcoderAGearTeeth) {
    this.kCANcoderAGearTeeth = CANcoderAGearTeeth;
    return this;
  }

  public TalonFXTurretConfiguration withCANcoderBGearTeeth(int CANcoderBGearTeeth) {
    this.kCANcoderBGearTeeth = CANcoderBGearTeeth;
    return this;
  }

  /**
   * Get the absolute position of the mechanism from the encoder positions
   *
   * @param rawCANcoderAPos position in rotations of CANcoder A
   * @param rawCANcoderBPos position in rotations of CANcoder B
   * @return The absolute position of the mechanism in configured units
   */
  public double getMechanismRotationsFromEncoders(double rawCANcoderAPos, double rawCANcoderBPos) {

    double encoderATeeth = rawCANcoderAPos * kCANcoderAGearTeeth;
    double encoderBTeeth = rawCANcoderBPos * kCANcoderBGearTeeth;

    for (double testPos = encoderATeeth;
        testPos < kCANcoderAGearTeeth * kCANcoderBGearTeeth;
        testPos += kCANcoderAGearTeeth) {

      double encoderBTestPos = testPos % kCANcoderBGearTeeth;

      if (MathUtil.isNear(encoderBTeeth, encoderBTestPos, 0.1)) {
        return kRealMotorConfig.getMechanismRotationsToUnits(
            testPos / kRealMotorConfig.getGearRatio());
      }
    }

    return 0;
  }
}
