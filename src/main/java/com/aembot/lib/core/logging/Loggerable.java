package com.aembot.lib.core.logging;

import com.aembot.lib.constants.generated.BuildConstants;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.RobotBase;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.NT4Publisher;
import org.littletonrobotics.junction.wpilog.WPILOGWriter;

/** Interface for a class that will set up the AKit {@link Logger} */
public interface Loggerable {
  public default void setupLogger() {
    setupMetadata();

    // On real log to logs file, and (if not on FMS) NetworkTables
    if (RobotBase.isReal()) {
      Logger.addDataReceiver(new WPILOGWriter("/U/logs"));
      if (!DriverStation.isFMSAttached()) {
        Logger.addDataReceiver(new NT4Publisher());
      }
    }

    // On sim log to both file & NetworkTables
    else if (RobotBase.isSimulation()) {
      Logger.addDataReceiver(new WPILOGWriter());
      Logger.addDataReceiver(new NT4Publisher());
    }

    // TODO Replay

    Logger.start();
  }

  /** Add metadata such as build constants (gversion) and robot info to log */
  public default void setupMetadata() {
    Logger.recordMetadata("ProjectName", BuildConstants.MAVEN_NAME);
    Logger.recordMetadata("BuildDate", BuildConstants.BUILD_DATE);
    Logger.recordMetadata("GitSHA", BuildConstants.GIT_SHA);
    Logger.recordMetadata("GitDate", BuildConstants.GIT_DATE);
    Logger.recordMetadata("GitBranch", BuildConstants.GIT_BRANCH);
    switch (BuildConstants.DIRTY) {
      case 0:
        Logger.recordMetadata("GitDirty", "All changes committed");
        break;
      case 1:
        Logger.recordMetadata("GitDirty", "Uncommitted changes");
        break;
      default:
        Logger.recordMetadata("GitDirty", "Unknown");
        break;
    }

    // TODO Add robot metadata
  }
}
