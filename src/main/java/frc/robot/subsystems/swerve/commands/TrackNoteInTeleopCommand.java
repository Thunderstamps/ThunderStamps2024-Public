package frc.robot.subsystems.swerve.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.note.NoteDetector;
import frc.robot.subsystems.indexer.IndexerSubsystem;
import frc.robot.subsystems.intake.IntakeSubsystem;
import frc.robot.subsystems.swerve.SwerveSubsystem;
import frc.robot.subsystems.tramp.TrampSubsystem;

public class TrackNoteInTeleopCommand extends Command {
    
    private static final double DECEL_IN_S = 2.0;

    private final SwerveSubsystem swerveSubsystem;
    private final IndexerSubsystem indexerSubsystem;
    private final TrampSubsystem trampSubsystem;
    private final IntakeSubsystem intakeSubsystem;
    private final NoteDetector noteDetector;
    private double lastAssist = 0.0;

    public TrackNoteInTeleopCommand(
            SwerveSubsystem swerveSubsystem,
            IndexerSubsystem indexerSubsystem,
            TrampSubsystem trampSubsystem,
            IntakeSubsystem intakeSubsystem,
            NoteDetector noteDetector) {
        this.swerveSubsystem = swerveSubsystem;
        this.indexerSubsystem = indexerSubsystem;
        this.trampSubsystem = trampSubsystem;
        this.intakeSubsystem = intakeSubsystem;
        this.noteDetector = noteDetector;

    }

    @Override
    public void initialize() {
        this.lastAssist = 0.0;
    }

    @Override
    public void execute() {

        var assist_in_sec = 0.0;

        var shouldAssist = this.noteDetector.GetAssistModeOn()
            && !this.indexerSubsystem.GetNotePresent() 
            && !this.trampSubsystem.GetNotePresent()
            && intakeSubsystem.IsIntaking();

        if(shouldAssist) {
            var optionTheta = this.noteDetector.GetTheta_rad();
            if(optionTheta.isPresent()) {
                assist_in_sec = optionTheta.getAsDouble() * 50.0;
            }
            else {
                if(this.lastAssist > 0 && this.lastAssist < DECEL_IN_S) {
                    assist_in_sec = this.lastAssist - DECEL_IN_S;
                }
                else if(this.lastAssist < 0 && this.lastAssist > -DECEL_IN_S) {
                    assist_in_sec = this.lastAssist + DECEL_IN_S;
                }
            }
        }
        
        this.swerveSubsystem.SetAssist(assist_in_sec);
        this.lastAssist = assist_in_sec;
    }

    @Override
    public void end(boolean interrupted) {
        this.swerveSubsystem.SetAssist(0.0);
    }
}
