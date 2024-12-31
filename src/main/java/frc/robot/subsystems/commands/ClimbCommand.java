package frc.robot.subsystems.commands;

import edu.wpi.first.wpilibj2.command.*;
import frc.robot.subsystems.angle.*;
import frc.robot.subsystems.elevator.*;
import frc.robot.subsystems.indexer.IndexerSubsystem;
import frc.robot.subsystems.shooter.*;
import frc.robot.subsystems.swerve.SwerveSubsystem;
import frc.robot.subsystems.tramp.*;
import frc.robot.subsystems.wallclimb.*;

public class ClimbCommand extends Command {
    
    private final ElevatorSubsystem elevatorSubsystem;
    private final WallClimbSubsystem wallClimbSubsystem;
    private final AngleSubsystem angleSubsystem;
    private final IndexerSubsystem indexerSubsystem;
    private final ShooterSubsystem shooterSubsystem;
    private final TrampSubsystem trampSubsystem;
    private final SwerveSubsystem swerveSubsystem;
    private boolean scoreStarted = false;

    public ClimbCommand(
            ElevatorSubsystem elevatorSubsystem,
            WallClimbSubsystem wallClimbSubsystem,
            AngleSubsystem angleSubsystem,
            IndexerSubsystem indexerSubsystem,
            ShooterSubsystem shooterSubsystem,
            TrampSubsystem trampSubsystem,
            SwerveSubsystem swerveSubsystem) {
        this.elevatorSubsystem = elevatorSubsystem;
        this.wallClimbSubsystem = wallClimbSubsystem;
        this.angleSubsystem = angleSubsystem;
        this.indexerSubsystem = indexerSubsystem;
        this.shooterSubsystem = shooterSubsystem;
        this.trampSubsystem = trampSubsystem;
        this.swerveSubsystem = swerveSubsystem;
        addRequirements(elevatorSubsystem);
        addRequirements(wallClimbSubsystem);
        addRequirements(angleSubsystem);
        addRequirements(indexerSubsystem);
        addRequirements(shooterSubsystem);
        addRequirements(trampSubsystem);
        addRequirements(swerveSubsystem);
    }

    @Override
    public void initialize() {
        this.scoreStarted = false;
        this.swerveSubsystem.zeroPitch();
        this.indexerSubsystem.SetModeClimb();
        this.shooterSubsystem.SetClimbing(true);
        this.shooterSubsystem.CoastShooter();
        this.elevatorSubsystem.GoToPositionUp();
        this.angleSubsystem.GoToPrepareToClimbPosition();
        this.wallClimbSubsystem.RunForward(4000);
    }

    @Override
    public void execute() {
        if(this.elevatorSubsystem.AtPositionUp()) {
            this.angleSubsystem.GoToClimbPosition(); // climb
            this.swerveSubsystem.SetDriveTowardsWallDuringClimb(true);

            if(this.angleSubsystem.AtPositionClimb()) { // stop climbing and score
                this.wallClimbSubsystem.Stop();
                if(!this.scoreStarted) {
                    this.trampSubsystem.SetHighCurrentLimit();
                }
                this.trampSubsystem.SetDutyCycle(1.0);
                this.scoreStarted = true;
            }
        }
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}
