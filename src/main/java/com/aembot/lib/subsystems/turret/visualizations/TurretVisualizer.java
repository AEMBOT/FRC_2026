package com.aembot.lib.subsystems.turret.visualizations;

import com.aembot.lib.core.logging.Loggable;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj.util.Color8Bit;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.mechanism.LoggedMechanism2d;
import org.littletonrobotics.junction.mechanism.LoggedMechanismLigament2d;
import org.littletonrobotics.junction.mechanism.LoggedMechanismRoot2d;

// mechanism visualizer for turret
// horizontal line the radius of the turret that rotates
// modified from ElevatorVisualizer
public class TurretVisualizer implements Loggable {
  private Color8Bit white = new Color8Bit(Color.kWhite);
  private LoggedMechanism2d vis2d = new LoggedMechanism2d(2, 1);
  private final LoggedMechanismRoot2d root = vis2d.getRoot("turretRoot", 1, 1);
  private final LoggedMechanismLigament2d turretLigament =
      new LoggedMechanismLigament2d("turretLigament", 1, 0, 8.0, white);

  public TurretVisualizer() {
    root.append(turretLigament);
  }

  public void updateViz(double angle) {
    this.root.setPosition(0.75, 0.51 + angle); // 0.51 is arbitrary and will change
  }

  @Override
  public void updateLog(String standardPrefix, String inputPrefix) {
    Logger.recordOutput(standardPrefix + "/TurretVis", this.vis2d);
  }
}
// public class TurretVisualizer {

// }
