package com.aembot.lib.math.mechanics;

import java.util.ArrayList;
import java.util.List;

/**
 * Modular class to allow for quick and easy computations of gear box ratios
 */
public class MultistageGearBox {

  /**
   * Represents and individual "stage" in the gear box, a stage in this context is the linkage between 2 gears
   */
  public static final class Stage {
    private final int kDrivingGearTeethCount;
    private final int kDrivenGearTeethCount;
    private final String kName;

    /**
     * Create a new gear box stage
     * @param drivingGearTeethCount Number of teeth on the gear the input is applied to
     * @param drivenGearTeethCount Number of teeth on the gear the is being driven
     */
    public Stage(int drivingGearTeethCount, int drivenGearTeethCount) {
      this.kDrivingGearTeethCount = drivingGearTeethCount;
      this.kDrivenGearTeethCount = drivenGearTeethCount;
      this.kName = null;
      if (this.kDrivingGearTeethCount == 0) {
        throw new IllegalArgumentException("Driving gear teeth count must be larger than 0");
      }
    }

    /**
     * Create a new gear box stage
     * @param drivingGearTeethCount Number of teeth on the gear the input is applied to
     * @param drivenGearTeethCount Number of teeth on the gear the is being driven       * @param name Name of the gearbox stage
     */
    public Stage(int drivingGearTeethCount, int drivenGearTeethCount, String name) {
      this.kDrivingGearTeethCount = drivingGearTeethCount;
      this.kDrivenGearTeethCount = drivenGearTeethCount;
      this.kName = name;
      if (this.kDrivingGearTeethCount == 0) {
        throw new IllegalArgumentException("Driving gear teeth count must be larger than 0");
      }
    }
    /**
     * Retrieve the overall gear ratio between the driving gear (motor) and the driven gear (output shaft or wheel).
     * 
     * The gear ratio is defined as:
     * <pre>
     *     Gear Ratio = (Teeth on Driven Gear) / (Teeth on Driving Gear)
     * </pre>
     * 
     * A higher gear ratio increases torque and reduces speed,
     * while a lower ratio increases speed and reduces torque.
     * 
     * This handles div by 0
     *
     * @return The gear ratio as a double (driven / driving)
     */  
    public double getGearRatio() {
      return (double) kDrivenGearTeethCount / (double) kDrivenGearTeethCount;
    }

    /**
     * @return The number of teeth on the driving gear
     */
    public int getDrivingTeeth() {
      return kDrivingGearTeethCount;
    }

    /**
     * @return The number of teeth on the driven gear
     */
    public int getDrivenTeeth() {
      return kDrivenGearTeethCount;
    }

    /**
     * Returns an empty string if the name is not set
     * @return The name of this stage
     */
    public String getName() {
      if (kName == null) {
        return "";
      } else {
        return kName;
      }
    }
  }
 
  // List of stages
  private List<Stage> stages = new ArrayList<Stage>();
  private double currentGearBoxRatio = 0;

  // Constructor
  public MultistageGearBox() {}

  /**
   * Add a gear stage to this multi-stage gearbox configuration.
   * 
   * The order in which stages are added should match the physical order in which
   * they are encountered starting from the motor output toward the final driven gear.
   * 
   * Each stage represents a pair of meshed gears (driving and driven) that
   * contribute to the total gear reduction or increase of the gearbox.
   * The overall gearbox ratio is the product of all individual stage ratios.
   *
   * For example, a two-stage gearbox with a 12→36 and 15→45 gear pair 
   * would yield a total reduction of (36/12) × (45/15) = 9:1.
   * 
   * Each time a stage is added, the total ratio is automatically recomputed.
   *
   * @param drivingGearTeeth The number of teeth on the driving gear (input side of the stage)
   * @param drivenGearTeeth The number of teeth on the driven gear (output side of the stage)
   * @return The {@link MultistageGearBox} being configured (for method chaining)
   */
  public MultistageGearBox addStage(int drivingGearTeeth, int drivenGearTeeth) {
    stages.add(new Stage(drivingGearTeeth, drivenGearTeeth));
    computeGearBoxRatio();
    return this;
  }

  /**
   * Calculate the output value from the input using the current total ratio.
   * Example: input motor rotations / ratio = output shaft rotations.
   *
   * @param input value to scale (speed, rotations, etc.)
   * @return scaled output value; 0 if ratio is zero or unset
   */
  public double calculate(double input) {
    if (currentGearBoxRatio != 0d) {
      return input / currentGearBoxRatio;
    } else {
      return 0d;
    }
  }

  /**
   * @return current total gearbox ratio (product of all stage ratios).
   *         Returns 0 if no stages are present.
   */
  public double getTotalRatio() {
    return currentGearBoxRatio;
  }

  /**
   * Get a stage from the gear box by index
   * @param index Index of the stage to retrieve from the gear box
   * @return The stage itself
   */
  public Stage getStage(int index) {
    try {
      return stages.get(index);
    } catch (IndexOutOfBoundsException e) {
      return null;
    }
  }

  /**
   * Recompute the total gearbox ratio as the product of all stage ratios.
   * Safe for empty lists.
   */
  private void computeGearBoxRatio() {
    if (stages.isEmpty()) {
      currentGearBoxRatio = 0d;
    } else {
      double ratio = 1d;
      for (Stage s : stages) {
        ratio *= s.getGearRatio();
      }
      currentGearBoxRatio = ratio;
    }
  }
}
