package com.aembot.frc2026.subsystems.turret;

// need to switch everythign from motorfollower to motorsubsystem

import com.aembot.lib.config.motors.MotorConfiguration;
import com.aembot.lib.core.motors.MotorInputs;
import com.aembot.lib.core.motors.interfaces.MotorIO;
import com.aembot.lib.subsystems.base.MotorSubsystem;
import com.aembot.lib.subsystems.base.turret.TurretIO;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import edu.wpi.first.wpilibj2.command.Command;

public class TurretSubsystem
    extends MotorSubsystem< // will extend CRTturretconfiguration?
        MotorInputs,
        MotorIO,
        MotorConfiguration<
            TalonFXConfiguration> // replaced by crtturretconfiguration? probably not?
    > {
  public final TurretIO turret;

  public TurretSubsystem(MotorConfiguration<TalonFXConfiguration> config, TurretIO turret) {

    super(new MotorInputs(), turret.getSingleMotor(), config);

    this.turret = turret;

    zeroEncoderPosition();
    setDefaultCommand(gotoPositionCommand());
  }

  @Override
  public void periodic() {
    super.periodic();
  }

  public Command gotoPositionCommand() { // not sure if this is the right thing to do here
    return smartPositionSetpointCommand(this::getPositionSetpointUnits)
        .withName("gotoPosition")
        .ignoringDisable(false);
  }

  @Override
  public void updateLog(String standardPrefix, String inputPrefix) {
    turret.updateLog(standardPrefix, inputPrefix);
  }
}
