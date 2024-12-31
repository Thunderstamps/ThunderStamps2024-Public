package frc.robot.subsystems.wallclimb;

import org.littletonrobotics.junction.AutoLog;

public interface WallClimbIO {
    
    @AutoLog
    public static class WallClimbIOInputs {
        public double velocity_rpm;
        public double outputCurrent_A;
        public double appliedOutput;
    }

    public default void updateInputs(WallClimbIOInputs inputs) {}

    public default void setDutyCycle(double dutyCycle) {}

    public default void setSpeed(double speed_rpm) {}
}
