package com.aembot.lib.subsystems.intake.over_bumper.run.io;

import com.aembot.lib.core.logging.Loggable;
import com.aembot.lib.core.motors.interfaces.MotorIO;
import com.aembot.lib.subsystems.intake.over_bumper.run.OverBumperIntakeRollerInputs;

/** Base IO layer for an over the bumper intake roller subsystem */
public interface OverBumperIntakeRollerIO extends Loggable {

  /**
   * @return The underlying motor that the io uses
   */
  public MotorIO getMotor();

  /**
   * Update the inputs of this io
   *
   * @param inputs New set of inputs
   */
  public void updateInputs(OverBumperIntakeRollerInputs inputs);
}
