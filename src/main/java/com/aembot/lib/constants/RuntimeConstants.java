package com.aembot.lib.constants;

import com.aembot.frc2026.constants.RobotRuntimeConstants;
import com.aembot.lib.config.RobotID;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.RobotBase;
import java.util.Optional;

/**
 * Year-agnostic (assuming FIRST doesn't do wierd things) constants determined at runtime. Should be
 * extended per-season.
 */
public class RuntimeConstants {
  /** Tracks the current runtime state of the robot */
  public enum RuntimeMode {
    /* This is real on robot code running */
    REAL,

    /* This code is running in a simulation */
    SIM,

    /* This code is replaying from a log file */
    REPLAY
  }

  /** The runtime mode: real bot, simulated bot or replaying log file */
  public static final RuntimeMode MODE = RobotBase.isReal() ? RuntimeMode.REAL : RuntimeMode.REPLAY;

  // This is maybe wierd and cursed. The intent is to create a singular point for lib to access this
  // season-specific field,
  // to make porting between years a bit easier and less spaghettiful. TODO have other humans check
  // this
  public static final RobotID ROBOT_ID = RobotRuntimeConstants.ROBOT_ID;

  /**
   * Are on the red alliance currently
   *
   * @return true if alliance assigned and red alliance
   */
  public static boolean isRedAlliance() {
    return DriverStation.getAlliance().isPresent()
        && DriverStation.getAlliance().equals(Optional.of(Alliance.Red));
  }

  /**
   * Are on the blue alliance currently
   *
   * @return true if alliance assigned and blue alliance
   */
  public static boolean isBlueAlliance() {
    return DriverStation.getAlliance().isPresent()
        && DriverStation.getAlliance().equals(Optional.of(Alliance.Blue));
  }
}
