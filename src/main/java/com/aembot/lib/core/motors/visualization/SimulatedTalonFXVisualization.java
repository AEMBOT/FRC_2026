package com.aembot.lib.core.motors.visualization;

import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj.util.Color8Bit;
import org.littletonrobotics.junction.mechanism.LoggedMechanism2d;
import org.littletonrobotics.junction.mechanism.LoggedMechanismLigament2d;
import org.littletonrobotics.junction.mechanism.LoggedMechanismRoot2d;

/** Helper class to create and view a simulated motor rotation */
public class SimulatedTalonFXVisualization {

  /* Internal mechanism 2d that is used to view the motor */
  private final LoggedMechanism2d mech2d;

  /** Root of the internal mechanism 2d */
  private final LoggedMechanismRoot2d root;

  /** Mechanism ligament at the maximum angle the motor can reach */
  private final LoggedMechanismLigament2d maxAngleLig;

  /** Mechanism ligament at the minimum angle the motor can reach */
  private final LoggedMechanismLigament2d minAngleLig;

  /** Mechanism ligament at the current angle of the motor */
  private final LoggedMechanismLigament2d curAngleLig;

  /**
   * Construct a new SimulatedTalonFX Visualization
   *
   * @param maxAngle Maximum angle of the mechanism in degrees
   * @param minAngle Minumum angle of the mechanism in degrees
   * @param startingAngle Angle to start the mechanism at in degrees
   */
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

  /**
   * Set the angle of the mechanism visualization
   *
   * @param newAngle The new angle of the mechanism in degrees
   */
  public void updateAngle(double newAngle) {
    this.curAngleLig.setAngle(newAngle);
  }

  /**
   * Supply this to Logger.recordOutput in order to add visualization to advantage scope
   *
   * @return The internal mechanism 2d used in this visualization
   */
  public LoggedMechanism2d getMech2d() {
    return mech2d;
  }
}
