package frc.robot.targeting;

import java.util.OptionalDouble;

public interface ITargetingController {

    // Angle that the robot must turn, in radians, to be lined up to the target.
    // Note that positive should be counter-clockwise.
    OptionalDouble getTargetHeading_rad(double currentHeading_rad);

    double getShooterSpeed_rpm();

    OptionalDouble getShooterAngle_deg();

    void SetTargetStatusSwerve(boolean onTarget);
    void SetTargetStatusShooter(boolean onTarget);
    void SetTargetStatusAngle(boolean onTarget);

    boolean IsOkToShoot();
}
