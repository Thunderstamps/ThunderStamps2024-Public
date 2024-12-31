package frc.robot.subsystems.intake.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.intake.IntakeSubsystem;

public class UnjamIntakeCommand extends Command {

    private static final double MILLISECONDS = 300;
    private IntakeSubsystem intakeSubsystem;
    private double startTime_sec;
    private boolean complete;

    public UnjamIntakeCommand(IntakeSubsystem intakeSubsystem) {
        this.intakeSubsystem = intakeSubsystem;
        addRequirements(intakeSubsystem);
    }

    @Override
    public void initialize() {
        this.intakeSubsystem.RunReverse();
        this.startTime_sec = Timer.getFPGATimestamp();
        this.complete = false;
    }

    @Override
    public void execute() {
        var duration_sec = Timer.getFPGATimestamp() - this.startTime_sec;
        if(duration_sec * 1000.0 >= MILLISECONDS) {
            this.complete = true;
        }
    }

    @Override
    public boolean isFinished() {
        return this.complete;
    }

    @Override
    public void end(boolean interrupted) {
        this.intakeSubsystem.IntakeNote();
    }
}
