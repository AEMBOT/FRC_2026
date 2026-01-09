package com.aembot.frc2026.constants;

import edu.wpi.first.wpilibj.RobotBase;

/** Class to hold constants that aren't specific to a single subsystem */
public class GeneralConstants {

  /** The current mode of the robot, is either REAL, SIM, or REPLAY */
  public static final Mode currentMode =
      RobotBase.isReal() ? Mode.REAL : Mode.SIM; // There is no (simple) way to determine in code

  // whether we are running sim or replay

  public static enum Mode {
    /** Running on a real robot */
    REAL,
    /** Running on a physics simulator */
    SIM,
    /** Replaying code from a log file */
    REPLAY
  }

  /** Time between each cycle of code */
  public static final double UPDATE_PERIOD = 0.02;
}
