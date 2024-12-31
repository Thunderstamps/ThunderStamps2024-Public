package frc.robot.subsystems.tramp;

import org.littletonrobotics.junction.AutoLog;

import com.revrobotics.REVLibError;

public interface TrampIO {
    
    @AutoLog
    public static class TrampIOInputs {
        public double velocity_rpm;
        public double position_rot;
        public double outputCurrent_A;
        public double appliedOutput;
    }

    public default void updateInputs(TrampIOInputs inputs) {}

    public default void setDutyCycle(double dutyCycle) {}

    public default void zeroPosition() {}

    public default void setSpeed(double speed_rpm) {}

    public default REVLibError setHighCurrentLimit() { return REVLibError.kOk;}
    public default REVLibError setLowCurrentLimit() { return REVLibError.kOk;}
}
