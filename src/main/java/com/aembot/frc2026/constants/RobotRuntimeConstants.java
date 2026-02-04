package com.aembot.frc2026.constants;

import com.aembot.frc2026.config.RobotConfiguration;
import com.aembot.frc2026.config.RobotIDYearly;
import com.aembot.lib.config.RobotID;
import com.aembot.lib.constants.RuntimeConstants;

public class RobotRuntimeConstants extends RuntimeConstants {
  public static final RobotID ROBOT_ID = RobotIDYearly.getIdentification();

  public static final RobotConfiguration ROBOT_CONFIG =
      RobotConfiguration.getRobotConstants((RobotIDYearly) ROBOT_ID);
}
