package frc.robot.targeting.providers.fixedpositions;

import frc.robot.subsystems.angle.AngleSubsystem;

public class PassShootingPosition extends FixedShootingPosition {
    
    public PassShootingPosition() {
        super(20.0, 2300, AngleSubsystem.MIN_ANGLE_DEG + 0.1);
    }
    
}
