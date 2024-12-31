package frc.robot.subsystems.shooter;

import org.littletonrobotics.junction.AutoLog;

import com.revrobotics.REVLibError;

public interface ShooterSideIO {

    @AutoLog
    public static class ShooterSideIOInputs {
        public double velocity_rpm;
        public double outputCurrent_A;
        public double appliedOutput;
        public double busVoltage;
    }

    public default void updateInputs(ShooterSideIOInputs inputs) {}

    public default REVLibError configure(boolean invert, double feedForwardGain) {
        return REVLibError.kOk;
    }

    public default void setVelocity(double velocity_rpm) {}
    public default void coast() {}
}
