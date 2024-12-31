package frc.robot.subsystems.tramp.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.tramp.TrampSubsystem;

public class TrampDefaultCommand extends Command {
    
    private TrampSubsystem trampSubsystem;

    public TrampDefaultCommand(
            TrampSubsystem trampSubsystem) {
        this.trampSubsystem = trampSubsystem;
        addRequirements(trampSubsystem);
    }

    @Override
    public void execute() {
        this.trampSubsystem.Stop();
    }
}
