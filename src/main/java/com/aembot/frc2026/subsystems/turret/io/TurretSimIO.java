package com.aembot.frc2026.subsystems.turret.io;

import com.aembot.frc2026.config.subsystems.TalonFXTurretConfiguration;
import com.aembot.frc2026.subsystems.turret.TurretInputs;
import com.aembot.lib.core.encoders.interfaces.CANCoderIO;
import com.aembot.lib.core.encoders.io.CANCoderSimIO;
import com.aembot.lib.core.motors.interfaces.MotorIO;
import com.aembot.lib.core.motors.io.MotorIOTalonFXSim;
import edu.wpi.first.wpilibj.Notifier;

public class TurretSimIO implements TurretIO {

  private final MotorIOTalonFXSim simMotor;

  private final CANCoderSimIO simCANcoderA;
  private final CANCoderSimIO simCANcoderB;

  private final TalonFXTurretConfiguration config;

  private final Notifier simNotifier;

  public TurretSimIO(TalonFXTurretConfiguration config) {
    this.config = config;
    this.simMotor = new MotorIOTalonFXSim(config.kSimMotorConfig);
    this.simCANcoderA = new CANCoderSimIO(config.kCANcoderAConfig);
    this.simCANcoderB = new CANCoderSimIO(config.kCANcoderBConfig);
    this.simNotifier = new Notifier(() -> updateSim());
    simNotifier.setName(config.kName + "Notifier");
    simNotifier.startPeriodic(0.005);
  }

  private void updateSim() {
    simMotor.updateSimState();

    simCANcoderA.updateSimState(
        simMotor.getTalon().getPosition().getValueAsDouble() / config.kCANcoderAGearTeeth,
        simMotor.getTalon().getVelocity().getValueAsDouble() / config.kCANcoderAGearTeeth,
        simMotor.getTalon().getAcceleration().getValueAsDouble() / config.kCANcoderAGearTeeth);

    simCANcoderA.updateSimState(
        simMotor.getTalon().getPosition().getValueAsDouble() / config.kCANcoderBGearTeeth,
        simMotor.getTalon().getVelocity().getValueAsDouble() / config.kCANcoderBGearTeeth,
        simMotor.getTalon().getAcceleration().getValueAsDouble() / config.kCANcoderBGearTeeth);
  }

  @Override
  public MotorIO getMotor() {
    return simMotor;
  }

  @Override
  public CANCoderIO getCANcoderA() {
    return simCANcoderA;
  }

  @Override
  public CANCoderIO getCANcoderB() {
    return simCANcoderB;
  }

  @Override
  public void updateInputs(TurretInputs inputs) {}

  @Override
  public void updateLog(String standardPrefix, String inputPrefix) {
    simMotor.logSim(standardPrefix, inputPrefix);
    simCANcoderA.logSim(standardPrefix, inputPrefix);
    simCANcoderB.logSim(standardPrefix, inputPrefix);
  }
}
