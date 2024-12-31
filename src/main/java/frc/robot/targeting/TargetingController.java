package frc.robot.targeting;

import java.util.OptionalDouble;

import frc.robot.communications.*;
import frc.robot.subsystems.shooter.*;
import frc.robot.targeting.mode.*;
import frc.robot.targeting.providers.fixedpositions.*;
import frc.robot.targeting.providers.limelight.Vision2D;
import frc.robot.targeting.providers.limelight.Vision2DIO;

// This class provides targeting information to the Shooter, Angle, and Swerve subsystems.
// It allows the operator/driver to choose between various targeting providers.
public class TargetingController implements ITargetingController {
    
    private final ShooterLookupTable shooterLookupTable = new ShooterLookupTable();
    private final ShootingMode shootingMode;
    private final NetworkTableComms nt;
    private final Vision2D vision2d;
    private final SafeShootingPosition safeShootingPosition = new SafeShootingPosition();
    private final LobShootingPosition lobShootingPosition = new LobShootingPosition();
    private final PassShootingPosition passShootingPosition = new PassShootingPosition();
    private double shooterSpeed_rpm = ShooterSubsystem.SHOOTER_RIGHT_RPM;
    private boolean onTargetSwerve = false;
    private boolean onTargetShooter = false;
    private boolean onTargetAngle = false;

    public TargetingController(
            ShootingMode shootingMode,
            NetworkTableComms nt,
            Vision2DIO vision2DIO) {
        this.shootingMode = shootingMode;
        this.nt = nt;
        this.vision2d = new Vision2D(nt, vision2DIO);
    }

    public OptionalDouble getTargetHeading_rad(double currentHeading_rad) {
        switch (shootingMode.GetMode()) {
            case Vision2D:
                return add(this.vision2d.getTheta_rad(), currentHeading_rad);

            case SafePosition:
                return OptionalDouble.of(this.safeShootingPosition.getFieldOriented_rad());

            case LobPosition:
                return OptionalDouble.of(this.lobShootingPosition.getFieldOriented_rad());

            case PassPosition:
                return OptionalDouble.empty(); // let driver control shot direction
                //return OptionalDouble.of(this.passShootingPosition.getFieldOriented_rad());
        
            default:
                return OptionalDouble.empty();
        }
    }

    private static OptionalDouble add(OptionalDouble optionTheta_rad, double currentHeading_rad) {
        if(optionTheta_rad.isPresent()) {
            var vision2dTarget_rad = optionTheta_rad.getAsDouble() + currentHeading_rad;
            return OptionalDouble.of(vision2dTarget_rad);
        }
        return OptionalDouble.empty();
    }

    public double getShooterSpeed_rpm() {
        if(this.shootingMode.IsManual()) {
            return this.nt.getManualShooterSpeed_rpm();
        }

        if(this.shootingMode.IsSafePosition()) {
            return this.safeShootingPosition.getShooterSpeed_rpm();
        }

        if(this.shootingMode.IsLobPosition()) {
            return this.lobShootingPosition.getShooterSpeed_rpm();
        }

        if(this.shootingMode.IsPassPosition()) {
            return this.passShootingPosition.getShooterSpeed_rpm();
        }

        var optionDistance_ft = this.getDistance_ft();
        if(optionDistance_ft.isPresent()) {
            var distance_ft = optionDistance_ft.getAsDouble();
            this.shooterSpeed_rpm = this.shooterLookupTable.GetSpeed_rpm(distance_ft);
            this.nt.setManualShooterSpeed_rpm(this.shooterSpeed_rpm);
        }

        return this.shooterSpeed_rpm;
    }

    public OptionalDouble getShooterAngle_deg() {
        if(this.shootingMode.IsManual()) {
            var shooterAngle_deg = this.nt.getManualAngle_deg();
            return OptionalDouble.of(shooterAngle_deg);
        }

        if(this.shootingMode.IsSafePosition()) {
            var shooterAngle_deg = this.safeShootingPosition.getAngle_deg();
            return OptionalDouble.of(shooterAngle_deg);
        }

        if(this.shootingMode.IsLobPosition()) {
            var shooterAngle_deg = this.lobShootingPosition.getAngle_deg();
            return OptionalDouble.of(shooterAngle_deg);
        }

        if(this.shootingMode.IsPassPosition()) {
            var shooterAngle_deg = this.passShootingPosition.getAngle_deg();
            return OptionalDouble.of(shooterAngle_deg);
        }

        var optionDistance_ft = this.getDistance_ft();
        if(optionDistance_ft.isPresent()) {
            var distance_ft = optionDistance_ft.getAsDouble();
            var shooterAngle_deg = this.shooterLookupTable.GetAngle_deg(distance_ft);
            this.nt.setManualAngle_deg(shooterAngle_deg);
            return OptionalDouble.of(shooterAngle_deg);
        }

        return OptionalDouble.empty();
    }

    private OptionalDouble getDistance_ft() {
        switch (shootingMode.GetMode()) {
            case Vision2D:
                return this.vision2d.getDistance_ft();
        
            default:
                return OptionalDouble.empty();
        }
    }

    @Override
    public void SetTargetStatusSwerve(boolean onTarget) {
        this.onTargetSwerve = onTarget;
        this.nt.setOnTargetSwerve(onTarget);
        this.setLimelightLEDs();
    }

    @Override
    public void SetTargetStatusShooter(boolean onTarget) {
        this.onTargetShooter = onTarget;
        this.nt.setOnTargetShooter(onTarget);
        this.setLimelightLEDs();
    }

    @Override
    public void SetTargetStatusAngle(boolean onTarget) {
        this.onTargetAngle = onTarget;
        this.nt.setOnTargetAngle(onTarget);
        this.setLimelightLEDs();
    }

    private void setLimelightLEDs() {
        if(this.IsOkToShoot()) {
            this.nt.setLimelightLEDsOn();
        }
        else {
            this.nt.setLimelightLEDsOff();
        }
    }

    @Override
    public boolean IsOkToShoot() {
        return this.onTargetSwerve
            && this.onTargetShooter
            && this.onTargetAngle;
    }
}
