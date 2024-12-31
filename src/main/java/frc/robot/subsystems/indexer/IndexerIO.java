package frc.robot.subsystems.indexer;

import org.littletonrobotics.junction.AutoLog;

public interface IndexerIO {
    
    @AutoLog
    public static class IndexerIOInputs {
        public boolean sensorDetectsNote;
        public double motorPosition_revs;
        public double motorVelocity_rps;
        public double motorStatorCurrent_A;
        public double motorSupplyCurrent_A;
        public double motorTemperature_C;
    }

    public default void updateInputs(IndexerIOInputs inputs) {}

    public default void setDutyCycle(double dutyCycle) {}

    public default void zeroPosition() {}

    public default void setVelocity(double rotations_per_sec) {}
}
