package frc.robot.note;

import org.littletonrobotics.junction.AutoLog;

public interface NoteDetectorIO {
    
    @AutoLog
    public static class NoteDetectorIOInputs {
        public boolean hasTargets;
        public double yaw_deg;
    }

    public default void updateInputs(NoteDetectorIOInputs inputs) {}

    public default void setDriverMode(boolean driverMode) {}
}
