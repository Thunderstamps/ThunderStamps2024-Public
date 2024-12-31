package frc.robot.subsystems.angle.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.angle.AngleSubsystem;

public class AngleTransferCommand extends Command {

    private final AngleSubsystem angleSubsystem;

    public AngleTransferCommand(AngleSubsystem angleSubsystem) {
        this.angleSubsystem = angleSubsystem;
        this.addRequirements(angleSubsystem);
    }

    @Override
    public void execute() {
        // transfer to amp scorer position
        this.angleSubsystem.SetPosition(55); 
    }
}
