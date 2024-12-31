package frc.robot.subsystems.shooter.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.shooter.ShooterSubsystem;

public class ShooterIdleCommand extends Command {
    
    private ShooterSubsystem shooterSubsystem;

    public ShooterIdleCommand(ShooterSubsystem shooterSubsystem) {
        this.shooterSubsystem = shooterSubsystem;
        this.addRequirements(shooterSubsystem);
    }

    @Override
    public void initialize() {
        this.shooterSubsystem.IdleShooter();
    }
}
