package com.aembot.frc2026.subsystems.turret.io;

import com.aembot.frc2026.config.subsystems.TalonFXTurretConfiguration;
import com.aembot.frc2026.subsystems.turret.TurretInputs;
import com.aembot.lib.core.encoders.interfaces.CANCoderIO;
import com.aembot.lib.core.encoders.io.CANCoderIOHardware;
import com.aembot.lib.core.motors.interfaces.MotorIO;
import com.aembot.lib.core.motors.io.MotorIOTalonFX;
import com.aembot.lib.tracing.Traced;

/** Hardware implementation for turret io */
public class TalonFXTurretHardwareIO implements TurretIO {

  /** Internal motor used */
  private final MotorIOTalonFX motor;

  /** CANcoder A used for absolute positioning */
  private final CANCoderIO CANcoderA;

  /** CANcoder B used for absolute positioning */
  private final CANCoderIO CANcoderB;

  /** Configuration for this turret */
  @SuppressWarnings("unused")
  private final TalonFXTurretConfiguration config;

  /**
   * Create a new hardware turret IO
   *
   * @param config configuration to use for this turret
   */
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
  @Traced(category = "Turret")
  public void updateInputs(TurretInputs inputs) {}

  @Override
  @Traced(category = "Turret")
  public void updateLog(String standardPrefix, String inputPrefix) {}
}
