package com.aembot.lib.subsystems.intake.over_bumper.deploy.io;

import com.aembot.lib.core.logging.Loggable;
import com.aembot.lib.core.motors.interfaces.MotorIO;
import com.aembot.lib.subsystems.intake.over_bumper.deploy.OverBumperIntakeDeployInputs;

/** Base IO layer for an over the bumper intake deployment subsystem */
public interface OverBumperIntakeDeployIO extends Loggable {

  /**
   * @return The underlying motor that the io uses
   */
  public MotorIO getMotor();

  /**
   * Update the inputs of this io
   *
   * @param inputs New set of inputs
   */
  public void updateInputs(OverBumperIntakeDeployInputs inputs);
}
