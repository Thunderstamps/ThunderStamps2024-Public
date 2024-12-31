package frc.robot.subsystems.commands;

import edu.wpi.first.wpilibj2.command.*;
import frc.robot.subsystems.angle.*;
import frc.robot.subsystems.indexer.IndexerSubsystem;
import frc.robot.subsystems.shooter.*;

public class PrepareToClimbCommand extends Command {
    
    private final AngleSubsystem angleSubsystem;
    private final IndexerSubsystem indexerSubsystem;
    private final ShooterSubsystem shooterSubsystem;
    private boolean isReadyToClimb = false;

    public PrepareToClimbCommand(
            AngleSubsystem angleSubsystem, 
            IndexerSubsystem indexerSubsystem,
            ShooterSubsystem shooterSubsystem) {
        this.angleSubsystem = angleSubsystem;
        this.indexerSubsystem = indexerSubsystem;
        this.shooterSubsystem = shooterSubsystem;
        addRequirements(angleSubsystem);
        addRequirements(indexerSubsystem);
        addRequirements(shooterSubsystem);
    }

    @Override
    public void initialize() {
        this.isReadyToClimb = false;
        this.angleSubsystem.GoToPrepareToClimbPosition();
        this.indexerSubsystem.SetModeClimb();
        this.shooterSubsystem.SetClimbing(true);
        this.shooterSubsystem.CoastShooter();
    }

    @Override
    public void execute() {
        this.isReadyToClimb = this.angleSubsystem.AtReadyToClimbPosition();
    }

    public boolean IsReadyToClimb() {
        return this.isReadyToClimb;
    }

    @Override
    public void end(boolean interrupted) {
        this.isReadyToClimb = false;
        this.indexerSubsystem.SetModeDefault();
        this.shooterSubsystem.SetClimbing(false);
    }
}
