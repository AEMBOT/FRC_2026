package com.aembot.lib.subsystems.flywheel.visualization;

import com.aembot.lib.core.logging.Loggable;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj.util.Color8Bit;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.mechanism.LoggedMechanism2d;
import org.littletonrobotics.junction.mechanism.LoggedMechanismLigament2d;
import org.littletonrobotics.junction.mechanism.LoggedMechanismRoot2d;

public class FlywheelVisualizer implements Loggable {
  private Color8Bit white = new Color8Bit(Color.kWhite);
  private LoggedMechanism2d viz2d = new LoggedMechanism2d(1.27, 2.032);
  private final LoggedMechanismRoot2d root = viz2d.getRoot("flywheelRoot", 0.75, 0.51);
  private final LoggedMechanismLigament2d flywheelLigament =
      new LoggedMechanismLigament2d("flywheelLigament", 0.13, 0.0, 20.0, white);

  public FlywheelVisualizer() {
    root.append(flywheelLigament);
  }

  // Represent the flywheel with a 2d mechanism that extends based on the velocity of the flywheel
  public void updateVis(double velocity) {
    this.flywheelLigament.setLength(velocity);
  }

  @Override
  public void updateLog(String standardPrefix, String inputPrefix) {
    Logger.recordOutput(standardPrefix + "/FlywheelViz", this.viz2d);
  }
}
