package frc.robot.subsystems.shooter;

import com.revrobotics.REVLibError;

import frc.robot.communications.*;
import frc.robot.utilities.*;
import frc.robot.targeting.*;

public class ShooterSubsystem extends ThunderSubsystem {

    public static final double SHOOTER_RIGHT_RPM = 3000; // 3500 is the max sustainable at 7V
    private static final double SHOOTER_LEFT_FACTOR = 0.8;
    private static final double SHOOTER_IDLE_RPM = 1500; // keep it running so a note that touches it still shoots out

    private final ShooterSide left;
    private final ShooterSide right;
    private final NetworkTableComms nt;
    private final ITargetingController targetingController;
    private boolean climbing = false;

    public ShooterSubsystem(
            NetworkTableComms nt,
            ITargetingController targetingController,
            ShooterSideIO leftShooterSideIO,
            ShooterSideIO rightShooterSideIO) {
        this.nt = nt;
        this.targetingController = targetingController;
        this.left = new ShooterSide(true, 0.000181, leftShooterSideIO);
        this.right = new ShooterSide(false, 0.000175, rightShooterSideIO);

        REVLibError leftConfigStatus = REVLibError.kError;
        {
            var tries = 0;
            var success = false;
            while (!success && tries < 10) {
                tries++;
                leftConfigStatus = this.left.configure();
                if(leftConfigStatus == REVLibError.kOk) {
                    success = true;
                }
                
            }
        }
        REVLibError rightConfigStatus = REVLibError.kError;
        {
            var tries = 0;
            var success = false;
            while (!success && tries < 10) {
                tries++;
                rightConfigStatus = this.right.configure();
                if(rightConfigStatus == REVLibError.kOk) {
                    success = true;
                }
                
            }
        }
        this.nt.setShooterConfigStatus(leftConfigStatus, rightConfigStatus);
    }

    @Override
    public void readInputs() {
        this.left.readInputs("Left");
        this.right.readInputs("Right");
    }

    @Override
    public void recordOutputs() {
        this.nt.setShooterSpeedRight(this.GetRightRpm());
        this.nt.setShooterSpeedLeft(this.GetLeftRpm());
        this.left.recordOutputs("Left");
        this.right.recordOutputs("Right");
    }

    public void RunShooterToTarget() {
        var right_rpm = this.targetingController.getShooterSpeed_rpm();
        this.RunShooter(right_rpm);
    }

    public void RunShooter(double right_rpm) {
        var left_rpm = right_rpm * SHOOTER_LEFT_FACTOR;
        this.left.SetVelocity(left_rpm);
        this.right.SetVelocity(right_rpm);
        this.nt.setShooterSpeedTarget(right_rpm);

        var diffLeft_rpm = left_rpm - this.GetLeftRpm();
        var diffRight_rpm = right_rpm - this.GetRightRpm();
        var onTargetLeft = Math.abs(diffLeft_rpm) < 25;
        var onTargetRight = Math.abs(diffRight_rpm) < 25;
        var onTarget = onTargetLeft && onTargetRight;
        this.targetingController.SetTargetStatusShooter(onTarget);
    }

    public void StopShooter() {
        this.left.SetVelocity(0);
        this.right.SetVelocity(0);
        this.nt.setShooterSpeedTarget(0);
        this.targetingController.SetTargetStatusShooter(false);
    }

    public boolean IsNotClimbing() {
        return !this.climbing;
    }

    public void SetClimbing(boolean value) {
        this.climbing = value;
    }

    // Puts the motors into coast mode
    public void CoastShooter() {
        this.left.Coast();
        this.right.Coast();
        this.targetingController.SetTargetStatusShooter(false);
    }

    public void IdleShooter() {
        var right_rpm = SHOOTER_IDLE_RPM;
        var left_rpm = right_rpm * SHOOTER_LEFT_FACTOR;
        this.left.SetVelocity(left_rpm);
        this.right.SetVelocity(right_rpm);
        this.nt.setShooterSpeedTarget(right_rpm);
        this.targetingController.SetTargetStatusShooter(false);
    }

    public double GetRightRpm() {
        return this.right.GetVelocityRpm();
    }

    public double GetLeftRpm() {
        return this.left.GetVelocityRpm();
    }
}
