package frc.robot.subsystems.shooter.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.shooter.ShooterSubsystem;

public class ShooterOffCommand extends Command {
    private final ShooterSubsystem shooterSubsystem;



    public ShooterOffCommand(
            ShooterSubsystem shooterSubsystem
    ) {
        this.shooterSubsystem = shooterSubsystem;
        this.addRequirements((shooterSubsystem));
    }

    public void initialize(){
        this.shooterSubsystem.CoastShooter();
    }
    
    @Override
    public boolean isFinished() {
        return false;
    }
}
