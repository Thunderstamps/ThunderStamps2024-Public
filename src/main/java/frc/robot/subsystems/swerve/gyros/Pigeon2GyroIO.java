package frc.robot.subsystems.swerve.gyros;

import org.littletonrobotics.junction.AutoLog;

public interface Pigeon2GyroIO {
    
    @AutoLog
    public static class Pigeon2GyroIOInputs {
        public double rotation2d_rad;
        public double rotationRate_rad;
        public double worldLinearAccelZ_G;
        public double pitch_deg;
        public double roll_deg;
    }

    public default void updateInputs(Pigeon2GyroIOInputs inputs) {}

    public default void zeroYaw() {}
}
