package frc.robot.subsystems.wallclimb.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.wallclimb.WallClimbSubsystem;

public class WallClimbDefaultCommand extends Command {
    
    private WallClimbSubsystem wallClimbSubsystem;

    public WallClimbDefaultCommand(WallClimbSubsystem wallClimbSubsystem) {
        this.wallClimbSubsystem = wallClimbSubsystem;
        addRequirements(wallClimbSubsystem);
    }

    @Override
    public void initialize() {
        this.wallClimbSubsystem.RunForward(0);
    }
}
