package frc.robot.subsystems.commands;

import edu.wpi.first.wpilibj2.command.*;
import frc.robot.subsystems.indexer.*;
import frc.robot.subsystems.indexer.commands.*;
import frc.robot.subsystems.intake.IntakeSubsystem;
import frc.robot.subsystems.swerve.*;
import frc.robot.subsystems.swerve.commands.*;
import frc.robot.targeting.ITargetingController;

public class ShootStationaryCommand extends SequentialCommandGroup {

    public ShootStationaryCommand(
            SwerveSubsystem swerveSubsystem, 
            IndexerSubsystem indexerSubsystem,
            IntakeSubsystem intakeSubsystem,
            ITargetingController targetingController) {
        
        addCommands(
            new ParallelDeadlineGroup(
                new SequentialCommandGroup(
                    new IndexerWaitForNoteCommand(indexerSubsystem),
                    new WaitUntilCommand(targetingController::IsOkToShoot),
                    new ParallelDeadlineGroup(
                        new IndexerShootCommand(indexerSubsystem), 
                        new StartEndCommand(
                            () -> intakeSubsystem.RunReverse(),
                            () -> intakeSubsystem.Stop(),
                            intakeSubsystem
                        )),
                    new IndexerWaitForEmptyCommand(indexerSubsystem)
                ), 
                new StopCommand(swerveSubsystem))
        );
    }
}
