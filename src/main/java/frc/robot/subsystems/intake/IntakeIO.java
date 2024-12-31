package frc.robot.subsystems.intake;

import org.littletonrobotics.junction.AutoLog;

public interface IntakeIO {
    
    @AutoLog
    public static class IntakeIOInputs {
        public double velocity_rpm;
        public double outputCurrent_A;
        public double appliedOutput;
    }

    public default void updateInputs(IntakeIOInputs inputs) {}

    public default void setSpeed(double speed_rpm) {}
}
