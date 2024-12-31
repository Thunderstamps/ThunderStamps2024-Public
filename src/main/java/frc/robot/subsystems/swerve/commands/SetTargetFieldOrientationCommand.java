package frc.robot.subsystems.swerve.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.swerve.SwerveSubsystem;

public class SetTargetFieldOrientationCommand extends Command {
    
    private final SwerveSubsystem swerveSubsystem;
    private final double targetFieldOrientation_rad;

    public SetTargetFieldOrientationCommand(
            SwerveSubsystem swerveSubsystem,
            double targetFieldOrientation_rad) {
        this.swerveSubsystem = swerveSubsystem;
        this.targetFieldOrientation_rad = targetFieldOrientation_rad;

    }

    @Override
    public void initialize() {
        this.swerveSubsystem.setTargetFieldOrientation_rad(this.targetFieldOrientation_rad);
    }

    @Override
    public boolean isFinished() {
        return true;
    }
}
