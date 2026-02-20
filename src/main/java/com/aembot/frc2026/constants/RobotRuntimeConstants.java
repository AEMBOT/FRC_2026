package com.aembot.frc2026.constants;

import com.aembot.frc2026.config.RobotConfiguration;
import com.aembot.frc2026.config.RobotIDYearly;
import com.aembot.lib.constants.RuntimeConstants;

public class RobotRuntimeConstants extends RuntimeConstants {
  public static final RobotIDYearly ROBOT_ID = (RobotIDYearly) RobotIDYearly.getIdentification();

  public static final RobotConfiguration ROBOT_CONFIG =
      RobotConfiguration.getRobotConstants((RobotIDYearly) ROBOT_ID);

      // TODO tune value
  public static final double AUTO_AIM_LATENCY_COMPENSATION_MS = 25;
}
