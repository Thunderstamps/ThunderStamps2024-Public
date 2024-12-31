package frc.robot.subsystems.swerve.commands;

import edu.wpi.first.wpilibj2.command.*;
import frc.robot.Constants;
import frc.robot.subsystems.swerve.SwerveSubsystem;

public class WheelsStraightCommand extends Command {
    
    private SwerveSubsystem swerveSubsystem;

    public WheelsStraightCommand(
            SwerveSubsystem swerveSubsystem) {

        this.swerveSubsystem = swerveSubsystem;
    }

    @Override
    public void initialize() {
        this.swerveSubsystem.setRotationP(0);
    }

    @Override
    public void end(boolean interrupted) {
        this.swerveSubsystem.setRotationP(Constants.ROTATION_P);
    }
}
