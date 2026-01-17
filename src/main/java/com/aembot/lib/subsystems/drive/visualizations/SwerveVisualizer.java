package com.aembot.lib.subsystems.drive.visualizations;

import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.mechanism.LoggedMechanism2d;
import org.littletonrobotics.junction.mechanism.LoggedMechanismLigament2d;

import com.aembot.lib.core.logging.Loggable;
import com.ctre.phoenix6.swerve.SwerveDrivetrain.SwerveDriveState;

import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj.util.Color8Bit;

/** Visualizer for the drivetrain with AScope */
public class SwerveVisualizer implements Loggable {
    /** Max speed of the robot in m/s */
    private final double maxRobotSpeed;
    
    /* ----- MODULE VISUALIZATION ----- */
    /** Mechanisms to represent the swerve module states */
    private final LoggedMechanism2d[] moduleMechanisms = new LoggedMechanism2d[] {
            new LoggedMechanism2d(1, 1),
            new LoggedMechanism2d(1, 1),
            new LoggedMechanism2d(1, 1),
            new LoggedMechanism2d(1, 1)
    };

    /** Direction and length changing ligaments for speed representation */
    private final LoggedMechanismLigament2d[] moduleSpeeds = new LoggedMechanismLigament2d[] {
            moduleMechanisms[0]
                    .getRoot("RootSpeed", 0.5, 0.5)
                    .append(new LoggedMechanismLigament2d("Speed", 0.5, 0)),
            moduleMechanisms[1]
                    .getRoot("RootSpeed", 0.5, 0.5)
                    .append(new LoggedMechanismLigament2d("Speed", 0.5, 0)),
            moduleMechanisms[2]
                    .getRoot("RootSpeed", 0.5, 0.5)
                    .append(new LoggedMechanismLigament2d("Speed", 0.5, 0)),
            moduleMechanisms[3]
                    .getRoot("RootSpeed", 0.5, 0.5)
                    .append(new LoggedMechanismLigament2d("Speed", 0.5, 0)),
    };

    /** Direction changing and length constant ligaments for module direction */
    private final LoggedMechanismLigament2d[] moduleDirections = new LoggedMechanismLigament2d[] {
            moduleMechanisms[0]
                    .getRoot("RootDirection", 0.5, 0.5)
                    .append(
                            new LoggedMechanismLigament2d(
                                    "Direction", 0.1, 0, 0,
                                    new Color8Bit(Color.kWhite))),
            moduleMechanisms[1]
                    .getRoot("RootDirection", 0.5, 0.5)
                    .append(
                            new LoggedMechanismLigament2d(
                                    "Direction", 0.1, 0, 0,
                                    new Color8Bit(Color.kWhite))),
            moduleMechanisms[2]
                    .getRoot("RootDirection", 0.5, 0.5)
                    .append(
                            new LoggedMechanismLigament2d(
                                    "Direction", 0.1, 0, 0,
                                    new Color8Bit(Color.kWhite))),
            moduleMechanisms[3]
                    .getRoot("RootDirection", 0.5, 0.5)
                    .append(
                            new LoggedMechanismLigament2d(
                                    "Direction", 0.1, 0, 0,
                                    new Color8Bit(Color.kWhite))),
    };
    /* --------------------------------------------- */

    public SwerveVisualizer(double maxSpeed) {
        this.maxRobotSpeed = maxSpeed;
    }

    public void updateSwerveState(SwerveDriveState state) {
        if (state == null || state.Pose == null || state.ModuleStates == null) {
            return;
        }

        // Update module visualizations
        for (int i = 0; i < 4; ++i) {
            moduleSpeeds[i].setAngle(state.ModuleStates[i].angle);
            moduleDirections[i].setAngle(state.ModuleStates[i].angle);
            moduleSpeeds[i].setLength(state.ModuleStates[i].speedMetersPerSecond / (2 * maxRobotSpeed));
        }
    }

    @Override
    public void updateLog(String standardPrefix, String inputPrefix) {
        for (LoggedMechanism2d mechanism : moduleMechanisms) {
            Logger.recordOutput(standardPrefix + "/DriveViz", mechanism);
        }
    }
}
