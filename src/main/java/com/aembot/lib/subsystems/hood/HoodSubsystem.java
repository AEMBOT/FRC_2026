package com.aembot.lib.subsystems.hood;

import com.aembot.lib.config.motors.MotorConfiguration;
import com.aembot.lib.config.subsystems.hood.TalonFXHoodConfiguration;
import com.aembot.lib.core.motors.MotorInputs;
import com.aembot.lib.core.motors.interfaces.MotorIO;
import com.aembot.lib.math.geometry.Pose2dMut;
import com.aembot.lib.math.geometry.Rotation2dMut;
import com.aembot.lib.math.geometry.Translation2dMut;
import com.aembot.lib.state.subsystems.hood.HoodState;
import com.aembot.lib.subsystems.base.MotorSubsystem;
import com.aembot.lib.subsystems.hood.io.HoodIO;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Timer;
import org.littletonrobotics.junction.Logger;

/** Extension of the motor subsystem to add hood functionality */
public class HoodSubsystem
    extends MotorSubsystem<MotorInputs, MotorIO, MotorConfiguration<TalonFXConfiguration>> {

  /** IO layer to use for this subsystem */
  private final HoodIO io;

  /** Configuration to use for this subsystem */
  @SuppressWarnings("unused") // Currently unused but that may change in the future
  private final TalonFXHoodConfiguration config;

  /** The HoodState instance to update */
  private final HoodState state;

  private final Rotation2dMut test = new Rotation2dMut(1);
  private final Translation2dMut test2 = new Translation2dMut(1.0, 2.0);
  private final Pose2dMut test3 = new Pose2dMut(test2, test);

  /**
   * Construct a new hood subsystem
   *
   * @param config Configuration to use for this subsystem
   * @param io IO layer to use for this subsystem
   */
  public HoodSubsystem(TalonFXHoodConfiguration config, HoodIO io, HoodState state) {
    super(config.kName, new MotorInputs(), io.getMotor(), config.kMotorConfig);
    this.io = io;
    this.config = config;
    this.state = state;

    this.setEncoderPosition(config.upwardsHardStopUnits);
  }

  @Override
  public void periodic() {
    double timestamp = Timer.getFPGATimestamp();
    super.periodic();

    state.updateHoodAngle(Units.degreesToRadians(inputs.positionUnits));

    // Log latency with time between periodic being called and finishing
    Logger.recordOutput(
        logPrefixStandard + "/LatencyPeriodicMS", (Timer.getFPGATimestamp() - timestamp) * 1000);

    test.add(0.1);
    test3.transformBy(new Pose2d(0.1, 0, Rotation2d.kZero));

    Logger.recordOutput("testRot", test);
    Logger.recordOutput("TestTrans", test2);
    Logger.recordOutput("TestPose", test3);
  }

  @Override
  public void updateLog(String standardPrefix, String inputPrefix) {
    Logger.processInputs(inputPrefix, inputs);
    io.updateLog(standardPrefix, inputPrefix);
    super.updateLog(standardPrefix, inputPrefix);
  }
}
