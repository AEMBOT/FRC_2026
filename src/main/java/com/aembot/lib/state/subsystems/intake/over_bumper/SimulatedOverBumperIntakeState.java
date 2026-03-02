package com.aembot.lib.state.subsystems.intake.over_bumper;

import static edu.wpi.first.units.Units.Meters;

import com.aembot.lib.config.subsystems.intake.overBumper.deploy.TalonFXOverBumperIntakeDeployConfiguration;
import com.aembot.lib.core.logging.Loggable;
import com.aembot.lib.state.subsystems.intake.over_bumper.deploy.OverBumperIntakeDeployState;
import com.aembot.lib.state.subsystems.intake.over_bumper.run.OverBumperIntakeRollerState;
import java.util.concurrent.atomic.AtomicReference;
import java.util.function.Supplier;
import org.ironmaple.simulation.IntakeSimulation;
import org.ironmaple.simulation.drivesims.AbstractDriveTrainSimulation;
import org.littletonrobotics.junction.Logger;

public class SimulatedOverBumperIntakeState implements Loggable {
  private final Supplier<OverBumperIntakeDeployState> kDeployStateSupplier;
  private final Supplier<OverBumperIntakeRollerState> kRollerStateSupplier;

  /** The config of the deploy subsystem. Fields used to initialize intake sim */
  private final TalonFXOverBumperIntakeDeployConfiguration deployConfig;

  /** The maplesim drivetrain simulation. Needed to initialize intake sim */
  private AtomicReference<AbstractDriveTrainSimulation> driveSim = new AtomicReference<>();

  private AtomicReference<IntakeSimulation> intakeSim = new AtomicReference<>();

  public SimulatedOverBumperIntakeState(
      Supplier<OverBumperIntakeDeployState> deployStateSupplier,
      Supplier<OverBumperIntakeRollerState> rollerStateSupplier,
      TalonFXOverBumperIntakeDeployConfiguration deployConfig) {
    this.kDeployStateSupplier = deployStateSupplier;
    this.kRollerStateSupplier = rollerStateSupplier;
    this.deployConfig = deployConfig;
  }

  private void init() {
    if (intakeSim.get() == null && driveSim.get() != null) {
      intakeSim.set(
          IntakeSimulation.OverTheBumperIntake(
              "Fuel",
              driveSim.get(),
              Meters.of(deployConfig.kWidthMeters),
              Meters.of(deployConfig.kExtensionMeters),
              deployConfig.kSide,
              1));

      // Only intake when the rollers are active
      intakeSim
          .get()
          .setCustomIntakeCondition(
              (_gamePiece) -> {
                OverBumperIntakeRollerState rollerState = kRollerStateSupplier.get();
                return rollerState != null && rollerState.isActive;
              });
    }
  }

  public void update() {
    if (intakeSim.get() != null && kDeployStateSupplier.get() != null) {
      if (kDeployStateSupplier.get().isDeployed) {
        // "starting" the intake regardless of whether the rollers are on, because the collider
        // should be active regardless of roller state if deployed
        intakeSim.get().startIntake();
      } else {
        intakeSim.get().stopIntake();
      }
    }
  }

  /** Supply a maplesim drivetrain to the intake sim. Required for initalization. */
  public void setDriveSim(AbstractDriveTrainSimulation driveSim) {
    if (this.driveSim.get() != null) {
      throw new IllegalStateException("Drive sim already set");
    }
    this.driveSim.set(driveSim);
    init();
  }

  /**
   * Pull a game piece from the intake. Used to obtain a game piece from the intake and move it a
   * feeder/shooter.
   *
   * @return True if a game piece was pulled from the intake, false if the intake was empty.
   * @see IntakeSimulation#obtainGamePieceFromIntake() obtainGamePieceFromIntake(); MapleSim method
   *     wrapped by this
   */
  public boolean pullGamePiece() {
    if (intakeSim.get() != null) {
      return intakeSim.get().obtainGamePieceFromIntake();
    } else {
      return false;
    }
  }

  @Override
  public void updateLog(String standardPrefix, String inputPrefix) {
    Logger.recordOutput(standardPrefix + "/Initialized", intakeSim.get() != null);
    Logger.recordOutput(
        standardPrefix + "/HeldGamePieces",
        intakeSim.get() != null ? intakeSim.get().getGamePiecesAmount() : -1);
  }
}
