package frc.robot.subsystems.swerve.commands;

import edu.wpi.first.wpilibj2.command.*;
import frc.robot.subsystems.swerve.*;
import frc.robot.subsystems.swerve.util.Vector2D;

public class MoveXDirectionAtSpeedCommand extends Command  {
    
    private SwerveSubsystem swerveSubsystem;
    private double speed_in_s;

    public MoveXDirectionAtSpeedCommand(
        SwerveSubsystem swerveSubsystem,
        double speedX_in_s) {
            this.swerveSubsystem = swerveSubsystem;
            this.speed_in_s = speedX_in_s;

        this.addRequirements(swerveSubsystem);
    }

    @Override
    public void execute() {
        this.swerveSubsystem.executeAutonomousControl(
            Vector2D.FromXY(0, 0), 
            Vector2D.FromXY(this.speed_in_s, 0),
            0.0);
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}
