package frc.robot.subsystems.swerve.gyros;

import org.littletonrobotics.junction.Logger;

import frc.robot.subsystems.swerve.util.SwerveUtil;

public class Pigeon2Gyro implements IGyro {

    private boolean enabled = true;
    private double offsetToZero_rad;

    private final Pigeon2GyroIO io;
    private final Pigeon2GyroIOInputsAutoLogged inputs = new Pigeon2GyroIOInputsAutoLogged();

    public Pigeon2Gyro(
            Pigeon2GyroIO io) {
        this.io = io;
    }

    @Override
    public void readInputs() {
        io.updateInputs(inputs);
        Logger.processInputs("Pigeon2Gyro", inputs);
    }

    @Override
    public double getFieldOrientation_rad() {
        double rawFieldOrientation_rad = inputs.rotation2d_rad - offsetToZero_rad;
        return SwerveUtil.limitAngleFromNegativePItoPI_rad(rawFieldOrientation_rad);
    }

    @Override
    public double getRotationRate_rad_s() {
        return inputs.rotationRate_rad;
    }

    @Override
    public void resetOffsetToZero_rad() {
        io.zeroYaw();
        this.offsetToZero_rad = 0;
    }

    @Override
    public void adjustOffset_rad(double adjustment_rad) {
        this.offsetToZero_rad += adjustment_rad;
    }

    @Override
    public boolean getEnabled() {
        return this.enabled;
    }

    @Override
    public void setEnabled(boolean enabled) {
        this.enabled = enabled;
    }

    @Override
    public double getZAccel_G() {
        return inputs.worldLinearAccelZ_G;
    }

    @Override
    public double getTiltAngle_deg() {
        return Math.abs(inputs.pitch_deg);
    }

    @Override
    public double getRoll_deg() {
        return inputs.roll_deg;
    }

    @Override
    public double getPitch_deg() {
        return inputs.pitch_deg;
    }

    @Override
    public boolean isTilted() {
        return this.getTiltAngle_deg() > 11.0;
    }

    @Override
    public boolean isNotTilted() {
        return this.getTiltAngle_deg() < 10.0;
    }

    @Override
    public void zeroPitch() {
    }
    
}
