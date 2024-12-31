package frc.robot.subsystems.intake.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.indexer.IndexerSubsystem;
import frc.robot.subsystems.intake.IntakeSubsystem;

public class RunIntakeUntilNote extends Command {
    
    private IntakeSubsystem intakeSubsystem;
    private IndexerSubsystem indexerSubsystem;

    public RunIntakeUntilNote(
            IntakeSubsystem intakeSubsystem, 
            IndexerSubsystem indexerSubsystem) {
        this.intakeSubsystem = intakeSubsystem;
        this.indexerSubsystem = indexerSubsystem;
        addRequirements(intakeSubsystem);
        // doesn't require indexer subsystem
    }

    @Override
    public void initialize() {
        if(!this.indexerSubsystem.GetNotePresent()) {
            this.intakeSubsystem.IntakeNote();;
            System.out.println("Start intake run (until note)");
        }
    }

    @Override
    public boolean isFinished() {
        return this.indexerSubsystem.GetNotePresent();
    }

    @Override
    public void end(boolean interrupted) {
        this.intakeSubsystem.Stop();
    }
}
