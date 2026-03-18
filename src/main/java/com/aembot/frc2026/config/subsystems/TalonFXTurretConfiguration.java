package com.aembot.frc2026.config.subsystems;

import com.aembot.lib.config.encoders.AEMCANCoderConfiguration;
import com.aembot.lib.config.motors.MotorConfiguration;
import com.aembot.lib.config.motors.SimulatedMotorConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import edu.wpi.first.math.geometry.Pose3d;

public class TalonFXTurretConfiguration {

  /** Name of the turret subsystem */
  public final String kName;

  /** Configuration of the real motor */
  public MotorConfiguration<TalonFXConfiguration> kRealMotorConfig;

  /** Configuration of the simulated motor */
  public SimulatedMotorConfiguration<TalonFXConfiguration> kSimMotorConfig;

  /** Configuration of CANcoder A */
  public AEMCANCoderConfiguration kCANcoderAConfig;

  /** Configuration of CANcoder B */
  public AEMCANCoderConfiguration kCANcoderBConfig;

  /** Number of teeth on the gear connecting the rotor to CANcoder A */
  public int kCANcoderAGearTeeth;

  /** Number of teeth on the gear connecting the rotor to CANcoder B */
  public int kCANcoderBGearTeeth;

  /** Number of teeth on the big mechanism gear next to both CANcoders */
  public int kMechanismTeeth;

  /** Magnet offset for CANcoder A */
  public double kCANcoderAOffset;

  /** Magnet offset for CANcoder B */
  public double kCANcoderBOffset;

  /** The origin pose of the turret for visualization in advantagescope. */
  public Pose3d kTurretOriginPose;

  public double startingRotation;

  /** How far we can be off in units for auto aim to still shoot */
  public double kAutoAimLeniance;

  /**
   * Create a new turret configuration
   *
   * @param name Name of the turret subsystem
   */
  public TalonFXTurretConfiguration(String name) {
    this.kName = name;
  }

  public TalonFXTurretConfiguration withRealMotorConfig(
      MotorConfiguration<TalonFXConfiguration> realMotorConfig) {
    this.kRealMotorConfig = realMotorConfig;
    return this;
  }

  public TalonFXTurretConfiguration withSimMotorConfig(
      SimulatedMotorConfiguration<TalonFXConfiguration> simMotorConfig) {
    this.kSimMotorConfig = simMotorConfig;
    return this;
  }

  public TalonFXTurretConfiguration withCANcoderAConfig(AEMCANCoderConfiguration CANcoderAConfig) {
    this.kCANcoderAConfig = CANcoderAConfig;
    return this;
  }

  public TalonFXTurretConfiguration withCANcoderBConfig(AEMCANCoderConfiguration CANcoderBConfig) {
    this.kCANcoderBConfig = CANcoderBConfig;
    return this;
  }

  public TalonFXTurretConfiguration withCANcoderAGearTeeth(int CANcoderAGearTeeth) {
    this.kCANcoderAGearTeeth = CANcoderAGearTeeth;
    return this;
  }

  public TalonFXTurretConfiguration withCANcoderBGearTeeth(int CANcoderBGearTeeth) {
    this.kCANcoderBGearTeeth = CANcoderBGearTeeth;
    return this;
  }

  public TalonFXTurretConfiguration withMechanismTeeth(int MechanismTeeth) {
    this.kMechanismTeeth = MechanismTeeth;
    return this;
  }

  public TalonFXTurretConfiguration withCANcoderAOffset(double CANcoderAOffset) {
    this.kCANcoderAOffset = CANcoderAOffset;
    return this;
  }

  public TalonFXTurretConfiguration withCANcoderBOffset(double CANcoderBOffset) {
    this.kCANcoderBOffset = CANcoderBOffset;
    return this;
  }

  /**
   * Set the origin pose of the turret for visualization in advantagescope.
   *
   * @return this {@link TalonFXTurretConfiguration} for chaining
   */
  public TalonFXTurretConfiguration withTurretOriginPose(Pose3d turretOriginPose) {
    this.kTurretOriginPose = turretOriginPose;
    return this;
  }

  public TalonFXTurretConfiguration withStartingRotation(double offet) {
    this.startingRotation = offet;
    return this;
  }

  /**
   * Set the amount of units that we can be off in order to still shoot
   *
   * <p>Counts both directions, so for example if this was 10, we could have a deviance of -10
   * through +10
   *
   * @return this {@link TalonFXTurretConfiguration} for chaining
   */
  public TalonFXTurretConfiguration withAutoAimLeniance(double autoAimLeniance) {
    this.kAutoAimLeniance = autoAimLeniance;
    return this;
  }

  /**
   * Get the absolute position of the mechanism from the encoder positions.
   *
   * <p>Uses Chinese Remainder Theorem (CRT) to determine absolute position from two encoders with
   * coprime gear ratios. Uses continuous interpolation for robustness to noise.
   *
   * @param encAPos position in rotations of CANcoder A [0, 1)
   * @param encBPos position in rotations of CANcoder B [0, 1)
   * @param teethA number of teeth on CANcoder A pinion
   * @param teethB number of teeth on CANcoder B pinion
   * @param teethMech number of teeth on the output mechanism gear
   * @return The absolute position of the mechanism in rotations [0, 1). Returns -1 if no valid
   *     match found.
   */
  public double getMechanismRotationsFromEncoders(
      double encAPos, double encBPos, int teethA, int teethB, int teethMech) {

    // Gear ratios: how many encoder rotations per output rotation
    double ratioA = (double) teethMech / teethA; // e.g., 100/13 = 7.69
    double ratioB = (double) teethMech / teethB; // e.g., 100/17 = 5.88

    double bestTheta = -1;
    double bestError = Double.MAX_VALUE;

    // Search over candidate output positions based on encoder A
    // Encoder A wraps ratioA times per output rotation, so there are ceil(ratioA) candidates
    int numCandidates = (int) Math.ceil(ratioA);

    for (int k = 0; k < numCandidates; k++) {
      // Candidate output position (in rotations, can be > 1)
      double theta = (encAPos + k) / ratioA;

      // What would encoder B read at this theta?
      double expectedB = (theta * ratioB) % 1.0;

      // Circular distance between expected and actual encoder B
      double diff = Math.abs(expectedB - encBPos);
      double circularDiff = Math.min(diff, 1.0 - diff);

      if (circularDiff < bestError) {
        bestError = circularDiff;
        bestTheta = theta;
      }
    }

    // Only accept if error is small (0.03 rotations = ~11 degrees on encoder)
    if (bestError < 0.03) {
      // Return position within single output rotation
      return bestTheta % 1.0;
    }

    return -1;
  }
}
