package frc.robot.subsystems.shooter.commands;

import edu.wpi.first.wpilibj2.command.*;
import frc.robot.subsystems.shooter.ShooterSubsystem;

public class ShooterRunToTargetCommand extends Command {
    
    private final ShooterSubsystem shooterSubsystem;

    public ShooterRunToTargetCommand(
            ShooterSubsystem shooterSubsystem) {
        this.shooterSubsystem = shooterSubsystem;
        this.addRequirements(shooterSubsystem);
    }

    @Override
    public void initialize() {
        this.shooterSubsystem.RunShooterToTarget();
    }

    @Override
    public void execute() {
        this.shooterSubsystem.RunShooterToTarget();
    }

    @Override
    public boolean isFinished() {
        return false;
    }

    @Override
    public void end(boolean interrupted) {
        this.shooterSubsystem.StopShooter();
    }
}
