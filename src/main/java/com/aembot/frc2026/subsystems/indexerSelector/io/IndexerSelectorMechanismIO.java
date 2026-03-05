package com.aembot.frc2026.subsystems.indexerSelector.io;

import com.aembot.frc2026.subsystems.indexerSelector.IndexerSelectorMechanismInputs;
import com.aembot.lib.core.logging.Loggable;
import com.aembot.lib.core.motors.interfaces.MotorIO;

public interface IndexerSelectorMechanismIO extends Loggable {
  /**
   * @return The underlying motor that this io uses
   */
  public MotorIO getMotor();

  /**
   * Update the inputs of this io
   *
   * @param inputs New set of inputs
   */
  public void updateInputs(IndexerSelectorMechanismInputs inputs);
}
