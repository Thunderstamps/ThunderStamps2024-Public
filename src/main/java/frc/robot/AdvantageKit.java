package frc.robot;

import org.littletonrobotics.junction.LogFileUtil;
import org.littletonrobotics.junction.LoggedRobot;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.NT4Publisher;
import org.littletonrobotics.junction.wpilog.WPILOGReader;
import org.littletonrobotics.junction.wpilog.WPILOGWriter;

import edu.wpi.first.wpilibj.PowerDistribution;

public class AdvantageKit {

    // When running on the real robot, it will always run the "real" hardware.
    // To run the SIM or REPLAY modes, use WPILib "Simulate Robot Code" and choose with this enum:
    public static final SetMode setMode = SetMode.SIM;
    
    private static enum SetMode {

        /** Run a physics simulator. */
        SIM,

        /** Run a physics simulator, and log to a log file. */
        SIM_LOGGED,

        /** Replay from a log file. */
        REPLAY
    }

    public static enum CurrentMode {

        /** Running on the real robot */
        REAL,

        /** Running a physics simulator. */
        SIM,

        /** Running a physics simulator, and logging to a log file. */
        SIM_LOGGED,

        /** Replaying from a log file. */
        REPLAY

    }

    public static CurrentMode getCurrentMode() {
        if(Robot.isReal()) {
            return CurrentMode.REAL;
        }

        switch (setMode) {
            case SIM:
                return CurrentMode.SIM;
                
            case SIM_LOGGED:
                return CurrentMode.SIM_LOGGED;
        
            default:
                return CurrentMode.REPLAY;
        }
    }

    public static void initializeLogging(LoggedRobot robot) {
        // Record metadata
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
            Logger.recordMetadata("GitDirty", "Uncomitted changes");
            break;
        default:
            Logger.recordMetadata("GitDirty", "Unknown");
            break;
        }

        // Set up data receivers & replay source
        switch (getCurrentMode()) {
            case REAL:
                // Running on a real robot, log to a USB stick ("/U/logs")
                Logger.addDataReceiver(new WPILOGWriter());
                Logger.addDataReceiver(new NT4Publisher());
                // AdvantageKit patches the PowerDistribution object to add logging, but we need to instantiate it at least once
                new PowerDistribution(); // should auto-detect CTRE PDP or Rev PDH
                break;

            case SIM:
                // Running a physics simulator, log to NT
                Logger.addDataReceiver(new NT4Publisher());
                break;

            case SIM_LOGGED:
                // Running a physics simulator, log to file and NT
                Logger.addDataReceiver(new WPILOGWriter());
                Logger.addDataReceiver(new NT4Publisher());
                break;

            case REPLAY:
                // Replaying a log, set up replay source
                robot.setUseTiming(false); // Run as fast as possible
                String logPath = LogFileUtil.findReplayLog();
                Logger.setReplaySource(new WPILOGReader(logPath));
                Logger.addDataReceiver(new WPILOGWriter(LogFileUtil.addPathSuffix(logPath, "_sim")));
                break;
        }
        

        // See http://bit.ly/3YIzFZ6 for more information on timestamps in AdvantageKit.
        // Logger.disableDeterministicTimestamps()

        // Start AdvantageKit logger
        Logger.start();
    }
}
