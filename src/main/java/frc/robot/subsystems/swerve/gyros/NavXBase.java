package frc.robot.subsystems.swerve.gyros;

import frc.robot.NavX.*;

public abstract class NavXBase {

    private double pitchZero_deg = 0.0;


    protected abstract AHRS getGyro();
    
    public double getTiltAngle_deg() {
        var pitch_deg = this.getGyro().getPitch();
        return Math.abs(pitch_deg);
    }

    public double getRoll_deg() {
        var roll_deg = this.getGyro().getRoll();
        return roll_deg;
    }

    public void zeroPitch() {
        this.pitchZero_deg = 0.0;
        this.pitchZero_deg = this.getPitch_deg();
    }

    public double getPitch_deg() {
        var pitch_deg = this.getGyro().getPitch();
        return pitch_deg - this.pitchZero_deg;
    }

    public boolean isTilted() {
        return this.getTiltAngle_deg() > 11.0;
    }

    public boolean isNotTilted() {
        return this.getTiltAngle_deg() < 10.0;
    }
}
