package com.aembot.frc2026.subsystems.turret.io;

import com.aembot.frc2026.config.subsystems.TalonFXTurretConfiguration;
import com.aembot.frc2026.subsystems.turret.TurretInputs;
import com.aembot.lib.core.encoders.interfaces.CANCoderIO;
import com.aembot.lib.core.encoders.io.CANCoderIOHardware;
import com.aembot.lib.core.motors.interfaces.MotorIO;
import com.aembot.lib.core.motors.io.MotorIOTalonFX;

public class TalonFXTurretHardwareIO implements TurretIO {

  private final MotorIOTalonFX motor;

  @SuppressWarnings("unused")
  private final CANCoderIO CANcoderA;

  @SuppressWarnings("unused")
  private final CANCoderIO CANcoderB;

  @SuppressWarnings("unused")
  private final TalonFXTurretConfiguration config;

  public TalonFXTurretHardwareIO(TalonFXTurretConfiguration config) {
    this.motor = new MotorIOTalonFX(config.kRealMotorConfig);
    this.CANcoderA = new CANCoderIOHardware(config.kCANcoderAConfig);
    this.CANcoderB = new CANCoderIOHardware(config.kCANcoderBConfig);
    this.config = config;
  }

  @Override
  public MotorIO getMotor() {
    return motor;
  }

  @Override
  public CANCoderIO getCANcoderA() {
    return CANcoderA;
  }

  @Override
  public CANCoderIO getCANcoderB() {
    return CANcoderB;
  }

  @Override
  public void updateInputs(TurretInputs inputs) {}

  @Override
  public void updateLog(String standartPrefix, String inputPrefix) {}
}
