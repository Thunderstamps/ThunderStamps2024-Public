package frc.robot.subsystems.swerve.gyros;

import org.littletonrobotics.junction.Logger;

import frc.robot.subsystems.swerve.util.SwerveUtil;

public class NavX2MxpGyro implements IGyro {

    private double scaleFactor;
    private boolean enabled = true;
    private double offsetToZero_rad;
    private double pitchZero_deg = 0.0;

    private final NavX2MxpGyroIO io;
    private final NavX2MxpGyroIOInputsAutoLogged inputs = new NavX2MxpGyroIOInputsAutoLogged();

    public NavX2MxpGyro(double scaleFactor, NavX2MxpGyroIO io) {
        this.scaleFactor = scaleFactor;
        this.io = io;
    }

    @Override
    public void readInputs() {
        io.updateInputs(inputs);
        Logger.processInputs("NavX2MxpGyro", inputs);
    }

    public double getFieldOrientation_rad() {
        double rawFieldOrientation_rad = inputs.rotation2d_rad * this.scaleFactor - offsetToZero_rad;
        return SwerveUtil.limitAngleFromNegativePItoPI_rad(rawFieldOrientation_rad);
    }

    public double getRotationRate_rad_s() {
        return inputs.rotationRate_rad;
    }

    public void resetOffsetToZero_rad() {
        io.reset();
        io.zeroYaw();
        this.offsetToZero_rad = 0;
    }

    public void adjustOffset_rad(double adjustment_rad) {
        this.offsetToZero_rad += adjustment_rad;
    }

    public boolean getEnabled() {
        return this.enabled;
    }

    public void setEnabled(boolean enabled) {
        this.enabled = enabled;
    }

    @Override
    public double getZAccel_G() {
        return inputs.worldLinearAccelZ_G;
    }
    
    public double getTiltAngle_deg() {
        return Math.abs(inputs.pitch_deg);
    }

    public double getRoll_deg() {
        return inputs.roll_deg;
    }

    public void zeroPitch() {
        this.pitchZero_deg = 0.0;
        this.pitchZero_deg = this.getPitch_deg();
    }

    public double getPitch_deg() {
        return inputs.pitch_deg - this.pitchZero_deg;
    }

    public boolean isTilted() {
        return this.getTiltAngle_deg() > 11.0;
    }

    public boolean isNotTilted() {
        return this.getTiltAngle_deg() < 10.0;
    }
}