package frc.robot.subsystems.indexer.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.indexer.IndexerSubsystem;

public class IndexerWaitForNoteCommand extends Command {
    
    private IndexerSubsystem indexerSubsystem;

    public IndexerWaitForNoteCommand(IndexerSubsystem indexerSubsystem) {
        this.indexerSubsystem = indexerSubsystem;
        // doesn't need to add requirement for indexer subsystem because it doesn't tell the indexer subsystem to do anything
    }

    @Override
    public void initialize() {
        System.out.println("Waiting for note.");
    }

    @Override
    public boolean isFinished() {
        return this.indexerSubsystem.GetNotePresent();
    }

    @Override
    public void end(boolean interrupted) {
        System.out.println("Got note.");
    }
}
