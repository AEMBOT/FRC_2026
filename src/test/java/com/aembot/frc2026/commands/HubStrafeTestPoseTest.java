package com.aembot.frc2026.commands;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertTrue;

import com.aembot.frc2026.constants.field.Field2026;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import org.junit.jupiter.api.Test;

class HubStrafeTestPoseTest {
  @Test
  void blueStartsThreeMetersFromHubInsideShootingZone() {
    Pose2d pose = DriveCommands.getHubStrafeTestPose(Alliance.Blue);
    assertEquals(3.0, pose.getTranslation().getDistance(Field2026.BLUE_HUB_CENTER), 1e-9);
    assertEquals(Field2026.BLUE_HUB_CENTER.getY(), pose.getY(), 1e-9);
    assertTrue(pose.getX() < 4.02844);
    // Blue table yaw is zero here. Relative yaw must be 180, away from the turret limits.
    assertEquals(-1.0, pose.getRotation().getCos(), 1e-9);
  }

  @Test
  void redMirrorsPositionAndKeepsTurretCenteredInsideShootingZone() {
    Pose2d pose = DriveCommands.getHubStrafeTestPose(Alliance.Red);
    var layout = Field2026.get().getFieldLayout();
    Translation2d hub =
        new Translation2d(
            layout.getFieldLength() - Field2026.BLUE_HUB_CENTER.getX(),
            layout.getFieldWidth() - Field2026.BLUE_HUB_CENTER.getY());
    assertEquals(3.0, pose.getTranslation().getDistance(hub), 1e-9);
    assertEquals(hub.getY(), pose.getY(), 1e-9);
    assertTrue(pose.getX() > 12.512548);
    // Red adds 180 to the blue-frame table yaw; heading zero leaves the turret at 180.
    assertEquals(1.0, pose.getRotation().getCos(), 1e-9);
  }
}
