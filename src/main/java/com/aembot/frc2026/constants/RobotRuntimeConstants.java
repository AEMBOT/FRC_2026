package com.aembot.frc2026.constants;

import com.aembot.frc2026.config.RobotIDYearly;
import com.aembot.lib.constants.RuntimeConstants;
import edu.wpi.first.wpilibj.RobotBase;

public class RobotRuntimeConstants extends RuntimeConstants {
  public static final RobotIDYearly ROBOT_ID = (RobotIDYearly) RobotIDYearly.getIdentification();

  // TODO config
  public enum RuntimeMode {
    REAL,

    SIM,

    REPLAY
  }

  public static final RuntimeMode CURRENT_RUNTIME_MODE =
      RobotBase.isReal()
          ? RuntimeMode.REAL
          : Boolean.getBoolean("robot.replay") ? RuntimeMode.REPLAY : RuntimeMode.SIM;

  public static final RobotConfig CURRENT_ROBOT_CONFIGS =
      RobotConfig.getRobotConstants((RobotIDYearly) ROBOT_ID);
}
