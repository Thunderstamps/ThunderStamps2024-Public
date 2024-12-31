package frc.robot.subsystems.swerve.drive;

import org.littletonrobotics.junction.AutoLog;

public interface DriveControllerIO {
    
    @AutoLog
    public static class DriveControllerIOInputs {
        public double motorPosition_revs;
        public double absolutePosition_revs;
        public double motorVelocity_rps;
        public double motorStatorCurrent_A;
        public double motorSupplyCurrent_A;
        public double motorTemperature_C;
    }

    public default void updateInputs(DriveControllerIOInputs inputs) {}

    public default void setDutyCycle(double dutyCycle) {}

    public default void setVelocity(double rotations_per_sec) {}
}
