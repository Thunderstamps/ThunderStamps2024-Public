package frc.robot.note.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.note.NoteDetector;

public class DisableAssistModeCommand extends Command {

    private final NoteDetector noteDetector;

    public DisableAssistModeCommand(NoteDetector noteDetector) {
        this.noteDetector = noteDetector;
        this.setName("Disable Assist Mode");
    }

    @Override
    public void initialize() {
        this.noteDetector.SetAssistMode(false);
    }

    @Override
    public boolean isFinished() {
        return true;
    }

    @Override
    public boolean runsWhenDisabled() {
        return true;
    }
}
