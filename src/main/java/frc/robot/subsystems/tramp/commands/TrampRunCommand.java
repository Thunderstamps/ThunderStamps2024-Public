package frc.robot.subsystems.tramp.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.tramp.TrampSubsystem;

public class TrampRunCommand extends Command {
    
    private TrampSubsystem trampSubsystem;

    public TrampRunCommand(
            TrampSubsystem trampSubsystem) {
        this.trampSubsystem = trampSubsystem;
        addRequirements(trampSubsystem);
    }

    @Override
    public void initialize() {
        this.trampSubsystem.SetVelocity(3000);
    }
}
