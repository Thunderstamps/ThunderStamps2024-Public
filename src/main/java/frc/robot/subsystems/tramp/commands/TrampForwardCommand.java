package frc.robot.subsystems.tramp.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.tramp.TrampSubsystem;

public class TrampForwardCommand extends Command {
    
    private final TrampSubsystem trampSubsystem;

    public TrampForwardCommand(TrampSubsystem trampSubsystem) {
        this.trampSubsystem = trampSubsystem;
        addRequirements(trampSubsystem);
    }

    @Override
    public void initialize() {
        this.trampSubsystem.SetVelocity(1000);
        //this.trampSubsystem.SetDutyCycle(0.25);
    }

    @Override
    public void end(boolean interrupted) {
        this.trampSubsystem.SetDutyCycle(0);
    }
}
