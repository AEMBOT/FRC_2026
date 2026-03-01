package com.aembot.frc2026.state;

import com.aembot.frc2026.constants.RobotRuntimeConstants;
import com.aembot.frc2026.constants.field.Field2026;
import com.aembot.frc2026.state.subsystems.indexer.SimulatedIndexerCompoundState;
import com.aembot.lib.state.SimulatedRobotState;
import com.aembot.lib.state.subsystems.flywheel.SimulatedShooterFlywheelState;
import com.aembot.lib.state.subsystems.intake.over_bumper.SimulatedOverBumperIntakeState;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import java.util.ArrayList;
import java.util.Arrays;
import org.ironmaple.simulation.SimulatedArena;
import org.littletonrobotics.junction.Logger;

public class SimulatedRobotStateYearly extends SimulatedRobotState {
  private SimulatedRobotStateYearly() {
    visionSimulation.addAprilTags(Field2026.get().getFieldLayout());

    SimulatedArena.getInstance().placeGamePiecesOnField();
  }

  // Ppl on the interwebs say this is good & thread safe
  private static final SimulatedRobotStateYearly INSTANCE = new SimulatedRobotStateYearly();

  public static SimulatedRobotStateYearly get() {
    return INSTANCE;
  }

  public final SimulatedOverBumperIntakeState simulatedIntakeState =
      new SimulatedOverBumperIntakeState(
          () -> RobotStateYearly.get().intakeDeployState.get(),
          () -> RobotStateYearly.get().intakeRollerState.get(),
          RobotRuntimeConstants.ROBOT_CONFIG.getIntakeDeployConfig());

  public final SimulatedIndexerCompoundState simulatedIndexerCompoundState =
      new SimulatedIndexerCompoundState(RobotStateYearly.get().indexerCompoundState);

  public final SimulatedShooterFlywheelState simulatedShooterFlywheelState =
      new SimulatedShooterFlywheelState(RobotStateYearly.get());

  @Override
  public void updateState() {
    super.updateState();

    simulatedIntakeState.update();
    if (simulatedIndexerCompoundState.getRoomInIndexer() && simulatedIntakeState.pullGamePiece())
      simulatedIndexerCompoundState.addSimulatedGamePiece();
    simulatedIndexerCompoundState.update();
    if (simulatedIndexerCompoundState.pullFromKicker().isPresent()) {
      simulatedShooterFlywheelState.simulateShot(
          new Transform3d(
              RobotRuntimeConstants.ROBOT_CONFIG
                  .getTurretConfig()
                  .kTurretOriginPose
                  .getTranslation(),
              new Rotation3d(
                  0,
                  RobotStateYearly.get().hoodState.getHoodAngleRadians(),
                  RobotStateYearly.get().getLatestFieldRobotPose().getRotation().getRadians()
                      + RobotStateYearly.get().turretState.turretYawRadians.get()
                      + Math.PI)),
          RobotStateYearly.get().shooterFlywheelState.flywheelSpeedUnitsPerSecond.get());
    }
  }

  @Override
  public void updateLog(String standardPrefix, String inputPrefix) {
    super.updateLog(standardPrefix, inputPrefix);

    // Get the positions of the fuel (both on the field and in the air)
    ArrayList<Pose3d> fuelPoses =
        new ArrayList<>(
            Arrays.asList(SimulatedArena.getInstance().getGamePiecesArrayByType("Fuel")));

    for (Transform3d fuelInRobot : simulatedIndexerCompoundState.getRenderedGamePiecePositions()) {
      fuelPoses.add(new Pose3d(this.getLatestFieldRobotPose()).plus(fuelInRobot));
    }

    // Publish to telemetry using AdvantageKit
    Logger.recordOutput("SimulatedRobotState/FuelPositions", fuelPoses.toArray(new Pose3d[0]));

    simulatedIndexerCompoundState.updateLog("SimulatedRobotState/IndexerCompound", "");
    simulatedIntakeState.updateLog("SimulatedRobotState/Intake", "");
    simulatedShooterFlywheelState.updateLog("SimulatedRobotState/ShooterFlywheel", "");
  }
}
