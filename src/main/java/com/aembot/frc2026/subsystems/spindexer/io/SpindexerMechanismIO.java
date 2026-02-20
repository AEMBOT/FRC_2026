package com.aembot.frc2026.subsystems.spindexer.io;

import com.aembot.frc2026.subsystems.spindexer.SpindexerMechanismInputs;
import com.aembot.lib.core.logging.Loggable;
import com.aembot.lib.core.motors.interfaces.MotorIO;

public interface SpindexerMechanismIO extends Loggable {
  /**
   * @return The underlying motor that this io uses
   */
  public MotorIO getMotor();

  /**
   * Update the inputs of this io
   *
   * @param inputs New set of inputs
   */
  public void updateInputs(SpindexerMechanismInputs inputs);
}
