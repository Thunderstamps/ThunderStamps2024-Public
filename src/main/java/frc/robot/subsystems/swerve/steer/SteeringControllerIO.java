package frc.robot.subsystems.swerve.steer;

import org.littletonrobotics.junction.AutoLog;

public interface SteeringControllerIO {
    
    @AutoLog
    public static class SteeringControllerIOInputs {
        public double homeSensorPosition_revs;
        public int homeSensorPosition_counts;
        public double motorPosition_revs;
        public double motorVelocity_rps;
        public double motorStatorCurrent_A;
        public double motorSupplyCurrent_A;
        public double motorTemperature_C;
    }

    public default void updateInputs(SteeringControllerIOInputs inputs) {}

    public default void setDutyCycle(double dutyCycle) {}
    public default void goToPositionPid(double target_revs) {}
    
}
