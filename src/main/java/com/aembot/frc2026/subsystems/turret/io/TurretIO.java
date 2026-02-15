package com.aembot.frc2026.subsystems.turret.io;

import com.aembot.frc2026.subsystems.turret.TurretInputs;
import com.aembot.lib.core.encoders.interfaces.CANCoderIO;
import com.aembot.lib.core.logging.Loggable;
import com.aembot.lib.core.motors.interfaces.MotorIO;

public interface TurretIO extends Loggable {

  public MotorIO getMotor();

  public CANCoderIO getCANcoderA();

  public CANCoderIO getCANcoderB();

  public void updateInputs(TurretInputs inputs);
}
