package com.aembot.frc2026.config.robots;

import com.aembot.frc2026.config.RobotConfiguration;
import com.aembot.lib.config.robot.PhysicalConfiguration;
import edu.wpi.first.math.util.Units;
import java.util.List;

// TODO pretty much everything here is a placeholder
/** {@link RobotConfiguration} for the robot Timothy */
public class ProductionConfig extends RobotConfiguration {
  private static final String ROBOT_NAME = "Timothy";

  private static final String DRIVETRAIN_BUS_NAME = "drivetrain";
  private static final List<String> CAN_BUS_NAMES = List.of(DRIVETRAIN_BUS_NAME);

  private static final PhysicalConfiguration PHYSICAL_CONFIGURATION =
      new PhysicalConfiguration()
          .withRobotWeightPounds(150)
          .withWheelBaseLengthM(Units.inchesToMeters(22.75))
          .withWheelTrackWidthM(Units.inchesToMeters(22.75))
          .withBumperLengthM(Units.inchesToMeters(35.625))
          .withBumperWidthM(Units.inchesToMeters(35.625))
          .withWheelCoefficientOfFriction(1.2);

  @Override
  public String getRobotName() {
    return ROBOT_NAME;
  }

  @Override
  public List<String> getCANBusNames() {
    return CAN_BUS_NAMES;
  }
}
