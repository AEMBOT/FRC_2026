package com.aembot.frc2026.state;

import com.aembot.frc2026.constants.RobotRuntimeConstants;
import com.aembot.frc2026.state.subsystems.indexer.IndexerCompoundState;
import com.aembot.frc2026.state.subsystems.turret.TurretState;
import com.aembot.lib.math.PositionUtil;
import com.aembot.lib.state.RobotState;
import com.aembot.lib.state.subsystems.flywheel.FlywheelState;
import com.aembot.lib.state.subsystems.hood.HoodState;
import com.aembot.lib.state.subsystems.intake.over_bumper.deploy.OverBumperIntakeDeployState;
import com.aembot.lib.state.subsystems.intake.over_bumper.run.OverBumperIntakeRollerState;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.util.Units;
import java.util.concurrent.atomic.AtomicReference;
import org.littletonrobotics.junction.Logger;

public class RobotStateYearly extends RobotState {
  // Ppl on the interwebs say this is good & thread safe
  private static final RobotStateYearly INSTANCE = new RobotStateYearly();

  private final Pose3d[] robotcentricMechanismPositions = new Pose3d[3];

  public final AtomicReference<OverBumperIntakeDeployState> intakeDeployState =
      new AtomicReference<OverBumperIntakeDeployState>();
  public final AtomicReference<OverBumperIntakeRollerState> intakeRollerState =
      new AtomicReference<OverBumperIntakeRollerState>();

  // note: this doesn't need to be atomic because all of its fields should be thread-safe.
  public final IndexerCompoundState indexerCompoundState =
      new IndexerCompoundState(
          RobotRuntimeConstants.ROBOT_CONFIG.getSpindexerConfiguration().kName,
          RobotRuntimeConstants.ROBOT_CONFIG.getIndexerSelectorConfiguration().kName,
          RobotRuntimeConstants.ROBOT_CONFIG.getIndexerKickerConfiguration().kName);

  public final HoodState hoodState = new HoodState();

  public final FlywheelState shooterFlywheelState = new FlywheelState();

  public TurretState turretState = new TurretState();

  public void updateIntakeDeployState(OverBumperIntakeDeployState state) {
    intakeDeployState.set(state);
  }

  public OverBumperIntakeDeployState getIntakeDeployState() {
    return intakeDeployState.get();
  }

  public void updateIntakeRollerState(OverBumperIntakeRollerState state) {
    intakeRollerState.set(state);
  }

  public OverBumperIntakeRollerState getIntakeRollerState() {
    return intakeRollerState.get();
  }

  public static RobotStateYearly get() {
    return INSTANCE;
  }

  @Override
  public void updateLog(String standardPrefix, String inputPrefix) {
    super.updateLog(standardPrefix, inputPrefix);

    indexerCompoundState.updateLog("SensorRobotState/IndexerCompound", "");
    turretState.updateLog("SensorRobotState/Turret", "");
    hoodState.updateLog("SensorRobotState/Hood", "");
    shooterFlywheelState.updateLog("SensorRobotState/ShooterFlywheel", "");

    // 10Hz update bcuz allocating these poses is 'spensive
    if (expensiveLogNoopCounter >= 5) {
      logMechanismPositions("SensorRobotState/MechanismPositions");
      expensiveLogNoopCounter = -1;
    }
  }

  private void logMechanismPositions(String prefix) {
    Pose3d turretPose =
        RobotRuntimeConstants.ROBOT_CONFIG
            .getTurretConfig()
            .kTurretOriginPose
            .plus(
                new Transform3d(0, 0, 0, new Rotation3d(turretState.turretYaw.get().unaryMinus())));

    Pose3d intakePose =
        RobotRuntimeConstants.ROBOT_CONFIG
            .getIntakeDeployConfig()
            .kPivotPoint
            .plus(
                new Transform3d(
                    0,
                    0,
                    0,
                    new Rotation3d(
                        0,
                        -Units.degreesToRadians(intakeDeployState.get().deployPositionUnits),
                        0)));

    Pose3d hoodPose =
        turretPose.transformBy(
            PositionUtil.toTransform3d(
                RobotRuntimeConstants.ROBOT_CONFIG
                    .getHoodConfig()
                    .kHoodOriginPose
                    .plus(
                        new Transform3d(
                            0,
                            0,
                            0,
                            new Rotation3d(0, -hoodState.getHoodAngle().getRadians(), 0)))));

    robotcentricMechanismPositions[0] = turretPose;
    robotcentricMechanismPositions[1] = intakePose;
    robotcentricMechanismPositions[2] = hoodPose;

    Logger.recordOutput("SensorRobotState/MechanismPositions", robotcentricMechanismPositions);
  }
}
