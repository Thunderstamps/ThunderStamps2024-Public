package frc.robot.subsystems.angle;

import org.littletonrobotics.junction.AutoLog;

public interface AngleIO {
    
    @AutoLog
    public static class AngleIOInputs {
        public boolean atHome;
        public double motorAbsolutePosition;
        public double motorPosition_revs;
        public double motorVelocity_rps;
        public double leftMotorStatorCurrent_A;
        public double leftMotorSupplyCurrent_A;
        public double leftMotorTemperature_C;
        public double rightMotorStatorCurrent_A;
        public double rightMotorSupplyCurrent_A;
        public double rightMotorTemperature_C;
    }

    public default void updateInputs(AngleIOInputs inputs) {}

    public default boolean setSpeed(double speed) {
        return true;
    }

    public default void goToPositionMotionProfile(double target_revs, double feedforward_A) {}
    public default void goToPositionPid(double target_revs, double feedforward_A) {}
    public default void stop() {}
    public default void follow() {}
}
