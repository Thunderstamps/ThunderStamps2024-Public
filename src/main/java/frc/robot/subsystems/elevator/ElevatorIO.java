package frc.robot.subsystems.elevator;

import org.littletonrobotics.junction.AutoLog;

public interface ElevatorIO {
    
    @AutoLog
    public static class ElevatorIOInputs {
        public double motorPosition_revs;
        public double motorVelocity_rps;
        public double motorStatorCurrent_A;
        public double motorSupplyCurrent_A;
        public double motorTemperature_C;
    }

    public default void updateInputs(ElevatorIOInputs inputs) {}

    public default void setDutyCycle(double dutyCycle) {}
    public default void goToPositionMotionProfile(double target_revs, double feedforward_A) {}
    public default void goToPositionPid(double target_revs, double feedforward_A) {}
}
