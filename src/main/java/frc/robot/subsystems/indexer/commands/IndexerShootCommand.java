package frc.robot.subsystems.indexer.commands;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.indexer.IndexerSubsystem;

public class IndexerShootCommand extends Command {
    
    private IndexerSubsystem indexerSubsystem;


    public IndexerShootCommand(
            IndexerSubsystem indexerSubsystem) {
        this.indexerSubsystem = indexerSubsystem;
        addRequirements(indexerSubsystem);
    }
    
    @Override
    public void initialize() {
        this.indexerSubsystem.SetModeShoot();
    }

    @Override
    public boolean isFinished() {
        return true;
    }
}
