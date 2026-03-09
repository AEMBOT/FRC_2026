package com.aembot.frc2026.subsystems.turret.io;

import com.aembot.frc2026.subsystems.turret.TurretInputs;
import com.aembot.lib.core.encoders.interfaces.CANCoderIO;
import com.aembot.lib.core.encoders.io.CANCoderReplayIO;
import com.aembot.lib.core.motors.interfaces.MotorIO;
import com.aembot.lib.core.motors.io.MotorIOReplay;
import com.aembot.lib.tracing.Traced;

/** Replay implementation for turret IO */
public class TurretReplayIO implements TurretIO {

  private final MotorIOReplay motor = new MotorIOReplay();
  private final CANCoderReplayIO cancoderA = new CANCoderReplayIO();
  private final CANCoderReplayIO cancoderB = new CANCoderReplayIO();

  @Override
  public MotorIO getMotor() {
    return motor;
  }

  @Override
  public CANCoderIO getCANcoderA() {
    return cancoderA;
  }

  @Override
  public CANCoderIO getCANcoderB() {
    return cancoderB;
  }

  @Override
  @Traced
  public void updateLog(String standardPrefix, String inputPrefix) {}

  @Override
  @Traced
  public void updateInputs(TurretInputs inputs) {}
}
