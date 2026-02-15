package com.aembot.frc2026.subsystems.turret;

import com.aembot.frc2026.config.subsystems.TalonFXTurretConfiguration;
import com.aembot.frc2026.subsystems.turret.io.TurretIO;
import com.aembot.lib.config.motors.MotorConfiguration;
import com.aembot.lib.core.motors.MotorInputs;
import com.aembot.lib.core.motors.interfaces.MotorIO;
import com.aembot.lib.subsystems.base.MotorSubsystem;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import org.littletonrobotics.junction.Logger;

public class TurretSubsystem
    extends MotorSubsystem<MotorInputs, MotorIO, MotorConfiguration<TalonFXConfiguration>> {
  public final TurretIO io;

  public TurretSubsystem(TalonFXTurretConfiguration config, TurretIO io) {

    super(config.kName, new MotorInputs(), io.getMotor(), config.kRealMotorConfig);

    this.io = io;

    setEncoderPosition(
        config.getMechanismRotationsFromEncoders(
            io.getCANcoderA().getRawAngle(), io.getCANcoderB().getRawAngle()));
  }

  @Override
  public void periodic() {
    super.periodic();
  }

  @Override
  public void updateLog(String standardPrefix, String inputPrefix) {
    Logger.processInputs(inputPrefix, inputs);
    io.updateLog(standardPrefix, inputPrefix);
    super.updateLog(standardPrefix, inputPrefix);
  }
}
