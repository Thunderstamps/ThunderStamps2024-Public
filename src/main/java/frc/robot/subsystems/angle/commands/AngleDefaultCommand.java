package frc.robot.subsystems.angle.commands;

import edu.wpi.first.wpilibj2.command.*;
import frc.robot.subsystems.angle.*;

public class AngleDefaultCommand extends Command {
    
    private final AngleSubsystem angleSubsystem;

    public AngleDefaultCommand(AngleSubsystem angleSubsystem) {
        this.angleSubsystem = angleSubsystem;
        this.addRequirements(angleSubsystem);
    }

    @Override
    public void execute() {
        // intake position (high enough that note touches indexer roller)
        this.angleSubsystem.GoToIntakePosition();
    }
}
