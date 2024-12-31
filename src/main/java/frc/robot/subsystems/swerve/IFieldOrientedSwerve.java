package frc.robot.subsystems.swerve;

import frc.robot.subsystems.swerve.util.IVector2D;

public interface IFieldOrientedSwerve {

    // Call this to execute the swerve drive in velocity mode, 
    // providing robot- and field-oriented translation vectors and 
    // a rotation rate. 
    void execute(
        IVector2D fieldOrientedTranslationCommand_in_s_rad,
        double fieldOrientedHeadingCommand_rad,
        IVector2D robotOrientedTranslationCommand_in_s_rad,
        double targetRotationRate_rad_s);

    void setRotationP(double rotationP);
}