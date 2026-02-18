package com.aembot.frc2026.state;

import com.aembot.frc2026.constants.RobotRuntimeConstants;
import com.aembot.frc2026.state.subsystems.indexer.IndexerCompoundState;
import com.aembot.lib.state.RobotState;
import com.aembot.lib.state.subsystems.hood.HoodState;
import com.aembot.lib.state.subsystems.intake.over_bumper.deploy.OverBumperIntakeDeployState;
import com.aembot.lib.state.subsystems.intake.over_bumper.run.OverBumperIntakeRollerState;
import java.util.concurrent.atomic.AtomicReference;

public class RobotStateYearly extends RobotState {
  // Ppl on the interwebs say this is good & thread safe
  private static final RobotStateYearly INSTANCE = new RobotStateYearly();

  public AtomicReference<OverBumperIntakeDeployState> intakeDeployState =
      new AtomicReference<OverBumperIntakeDeployState>();
  public AtomicReference<OverBumperIntakeRollerState> intakeRollerState =
      new AtomicReference<OverBumperIntakeRollerState>();

  // note: this doesn't need to be atomic because all of its fields should be thread-safe.
  public final IndexerCompoundState indexerCompoundState =
      new IndexerCompoundState(
          RobotRuntimeConstants.ROBOT_CONFIG.getSpindexerConfiguration().kName,
          RobotRuntimeConstants.ROBOT_CONFIG.getIndexerSelectorConfiguration().kName,
          RobotRuntimeConstants.ROBOT_CONFIG.getIndexerKickerConfiguration().kName);
          
  public HoodState hoodState = new HoodState();

  public void updateIntakeDeployState(OverBumperIntakeDeployState state) {
    intakeDeployState = new AtomicReference<OverBumperIntakeDeployState>(state);
  }

  public OverBumperIntakeDeployState getIntakeDeployState() {
    return intakeDeployState.get();
  }

  public void updateIntakeRollerState(OverBumperIntakeRollerState state) {
    intakeRollerState = new AtomicReference<OverBumperIntakeRollerState>(state);
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
    hoodState.updateLog("SensorRobotState/Hood", "");
  }
}
