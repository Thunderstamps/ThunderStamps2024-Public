package frc.robot.subsystems.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.elevator.ElevatorSubsystem;
import frc.robot.subsystems.tramp.TrampSubsystem;
import frc.robot.subsystems.wallclimb.WallClimbSubsystem;

// Raises the elevator to Amp score height and then runs the Tramp scorer to score in the Amp, and finally lowers the elevator
public class AmpScoreCommand extends Command {
    private final ElevatorSubsystem elevatorSubsystem;
    private final TrampSubsystem trampSubsystem;
    private final WallClimbSubsystem wallClimbSubsystem;
    private double startTime_sec;
    private boolean isFinished;
    private boolean scoreStarted = false;

    public AmpScoreCommand (
            ElevatorSubsystem elevatorSubsystem, 
            TrampSubsystem trampSubsystem, 
            WallClimbSubsystem wallclimbSubsystem){
        this.elevatorSubsystem = elevatorSubsystem;
        this.trampSubsystem = trampSubsystem;
        this.wallClimbSubsystem = wallclimbSubsystem;
        addRequirements(elevatorSubsystem);
        addRequirements(trampSubsystem);
        addRequirements(wallclimbSubsystem);
    }

    @Override
    public void initialize() {
        this.startTime_sec = Timer.getFPGATimestamp();
        this.isFinished = false;
        this.scoreStarted = false;
        this.elevatorSubsystem.GoToPositionAmpScore();

    }

    @Override
    public void execute() {
        var duration_sec = Timer.getFPGATimestamp() - this.startTime_sec;
        if (duration_sec > 0.5) {
            if(!this.scoreStarted) {
                this.trampSubsystem.SetHighCurrentLimit();
            }
            this.trampSubsystem.SetDutyCycle(1.0);
            this.wallClimbSubsystem.RunForward(2500);
            this.scoreStarted = true;
        }
        if (duration_sec > 2.5) {
            isFinished = true;
        }
    }

    @Override
    public boolean isFinished() {
        return this.isFinished;
    }
    
    @Override
    public void end(boolean interrupted) {
        this.elevatorSubsystem.GoToPositionDown();
        this.trampSubsystem.Stop();
        this.trampSubsystem.SetLowCurrentLimit();
        this.wallClimbSubsystem.Stop();
        if(!interrupted) {
            this.trampSubsystem.SetNotePresent(false);
        }
    }
}
