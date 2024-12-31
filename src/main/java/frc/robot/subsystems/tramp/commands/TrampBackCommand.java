package frc.robot.subsystems.tramp.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.tramp.TrampSubsystem;

public class TrampBackCommand extends Command {
    
    private final TrampSubsystem trampSubsystem;

    public TrampBackCommand(TrampSubsystem trampSubsystem) {
        this.trampSubsystem = trampSubsystem;
        addRequirements(trampSubsystem);
    }

    @Override
    public void initialize() {
        this.trampSubsystem.SetDutyCycle(-0.06);
    }

    @Override
    public void end(boolean interrupted) {
        this.trampSubsystem.SetDutyCycle(0);
    }
}
