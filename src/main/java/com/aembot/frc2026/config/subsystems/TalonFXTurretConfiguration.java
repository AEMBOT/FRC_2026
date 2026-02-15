package com.aembot.frc2026.config.subsystems;

import com.aembot.lib.config.encoders.AEMCANCoderConfiguration;
import com.aembot.lib.config.motors.MotorConfiguration;
import com.aembot.lib.config.motors.SimulatedMotorConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import edu.wpi.first.math.MathUtil;

public class TalonFXTurretConfiguration {

  public final String kName;

  public MotorConfiguration<TalonFXConfiguration> kRealMotorConfig;

  public SimulatedMotorConfiguration<TalonFXConfiguration> kSimMotorConfig;

  public AEMCANCoderConfiguration kCANcoderAConfig;

  public AEMCANCoderConfiguration kCANcoderBConfig;

  public int kCANcoderAGearTeeth;

  public int kCANcoderBGearTeeth;

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

  public double getMechanismRotationsFromEncoders(double rawCANcoderAPos, double rawCANcoderBPos) {

    double encoderAPos = rawCANcoderAPos * kCANcoderAGearTeeth;
    double encoderBPos = rawCANcoderBPos * kCANcoderBGearTeeth;

    for (double testPos = encoderAPos;
        testPos < kCANcoderAGearTeeth * kCANcoderBGearTeeth;
        testPos += kCANcoderAGearTeeth) {

      double encoderBTestPos = testPos % kCANcoderBGearTeeth;
      if (MathUtil.isNear(encoderBPos, encoderBTestPos, 0.1)) {
        return testPos;
      }
    }

    return 0;
  }
}
