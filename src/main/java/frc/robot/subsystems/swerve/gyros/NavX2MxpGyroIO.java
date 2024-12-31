package frc.robot.subsystems.swerve.gyros;

import org.littletonrobotics.junction.AutoLog;

public interface NavX2MxpGyroIO {
    
    @AutoLog
    public static class NavX2MxpGyroIOInputs {
        public double rotation2d_rad;
        public double rotationRate_rad;
        public double worldLinearAccelZ_G;
        public double pitch_deg;
        public double roll_deg;
    }

    public default void updateInputs(NavX2MxpGyroIOInputs inputs) {}

    public default void reset() {}
    public default void zeroYaw() {}
}
