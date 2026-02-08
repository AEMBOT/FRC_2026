package com.aembot.lib.core.motors.visualization;

import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj.util.Color8Bit;
import org.littletonrobotics.junction.mechanism.LoggedMechanism2d;
import org.littletonrobotics.junction.mechanism.LoggedMechanismLigament2d;
import org.littletonrobotics.junction.mechanism.LoggedMechanismRoot2d;

public class SimulatedTalonFXVisualization {
  private final LoggedMechanism2d mech2d;
  private final LoggedMechanismRoot2d root;

  private final LoggedMechanismLigament2d maxAngleLig;

  private final LoggedMechanismLigament2d minAngleLig;

  private final LoggedMechanismLigament2d curAngleLig;

  public SimulatedTalonFXVisualization(double maxAngle, double minAngle, double startingAngle) {
    this.mech2d = new LoggedMechanism2d(2, 2);
    this.mech2d.setBackgroundColor(new Color8Bit(Color.kBlack));
    this.root = mech2d.getRoot("root", 1, 1);

    this.maxAngleLig =
        new LoggedMechanismLigament2d(
            "MaxAngleLigament", 1, maxAngle, 2, new Color8Bit(Color.kGray));
    this.minAngleLig =
        new LoggedMechanismLigament2d(
            "MinAngleLigament", 1, minAngle, 2, new Color8Bit(Color.kGray));
    this.curAngleLig =
        new LoggedMechanismLigament2d(
            "CurAngleLigament", 1, startingAngle, 4, new Color8Bit(Color.kMediumAquamarine));

    this.root.append(maxAngleLig);
    this.root.append(minAngleLig);
    this.root.append(curAngleLig);
  }

  public void updateAngle(double newAngle) {
    this.curAngleLig.setAngle(newAngle);
  }

  public LoggedMechanism2d getMech2d() {
    return mech2d;
  }
}
