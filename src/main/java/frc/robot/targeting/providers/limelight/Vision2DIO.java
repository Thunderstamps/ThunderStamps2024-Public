package frc.robot.targeting.providers.limelight;

import org.littletonrobotics.junction.AutoLog;

public interface Vision2DIO {
    
    @AutoLog
    public static class Vision2DIOInputs {
        public boolean limelight_tv;
        public double limelight_tx;
        public double limelight_ty;
    }

    public default void updateInputs(Vision2DIOInputs inputs) {}
}
