package frc.robot.targeting.providers.fixedpositions;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;

public abstract class FixedShootingPosition {
    
    private double redFieldOriented_rad;
    private double shooterSpeed_rpm;
    private double angle_deg;

    protected FixedShootingPosition(
            double redFieldOriented_deg,
            double shooterSpeed_rpm,
            double angle_deg) {
        this.redFieldOriented_rad = Math.toRadians(redFieldOriented_deg);
        this.shooterSpeed_rpm = shooterSpeed_rpm;
        this.angle_deg = angle_deg;
    }

    public double getFieldOriented_rad() {
        var optionAlliance = DriverStation.getAlliance();
        if(optionAlliance.isPresent()) {
            if(optionAlliance.get() == Alliance.Red) {
                return this.redFieldOriented_rad;
            }
        }
        return -this.redFieldOriented_rad;
    }

    public double getShooterSpeed_rpm() {
        return this.shooterSpeed_rpm;
    }

    public double getAngle_deg() {
        return this.angle_deg;
    }
}
