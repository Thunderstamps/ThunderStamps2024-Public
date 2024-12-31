package frc.robot.note.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.note.NoteDetector;

public class EnableAssistModeCommand extends Command {
    
    private final NoteDetector noteDetector;

    public EnableAssistModeCommand(NoteDetector noteDetector) {
        this.noteDetector = noteDetector;
        this.setName("Enable Assist Mode");
    }

    @Override
    public void initialize() {
        this.noteDetector.SetAssistMode(true);
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
