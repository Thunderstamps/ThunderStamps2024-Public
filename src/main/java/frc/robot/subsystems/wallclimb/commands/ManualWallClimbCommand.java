package frc.robot.subsystems.wallclimb.commands;

import edu.wpi.first.wpilibj2.command.*;
import frc.robot.communications.NetworkTableComms;
import frc.robot.subsystems.wallclimb.WallClimbSubsystem;

public class ManualWallClimbCommand extends Command {

    private WallClimbSubsystem wallClimbSubsystem;
    private NetworkTableComms nt;

    public ManualWallClimbCommand(
        WallClimbSubsystem wallClimbSubsystem,
        NetworkTableComms nt)
    {
        this.wallClimbSubsystem = wallClimbSubsystem;
        this.nt = nt;
        addRequirements(wallClimbSubsystem);
    }

    @Override
    public void execute() {
        var speed_rpm = this.nt.getManualWallClimb_rpm();
        this.wallClimbSubsystem.RunForward(speed_rpm);
    }
}
