package frc.robot.utilities.canivore;

import org.littletonrobotics.junction.AutoLog;

public interface CanivoreIO {
    
    @AutoLog
    public static class CanivoreIOInputs {
        public float busUtilization;
        public int receiveErrorCounter;
        public int transmitErrorCounter;
    }

    public default void updateInputs(CanivoreIOInputs inputs) {}
}
