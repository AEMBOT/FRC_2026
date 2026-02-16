package com.aembot.frc2026.state;

import com.aembot.lib.state.RobotState;
import org.ironmaple.simulation.SimulatedArena;
import org.littletonrobotics.junction.Logger;

public class RobotStateYearly extends RobotState {
  // Ppl on the interwebs say this is good & thread safe
  private static final RobotStateYearly INSTANCE = new RobotStateYearly();

  public static RobotStateYearly get() {
    return INSTANCE;
  }

  @Override
  public void updateLog(String standardPrefix, String inputPrefix) {
    super.updateLog(standardPrefix, inputPrefix);

    Logger.recordOutput(
        "FieldSimulation/Fuel", SimulatedArena.getInstance().getGamePiecesArrayByType("Fuel"));
  }
}
