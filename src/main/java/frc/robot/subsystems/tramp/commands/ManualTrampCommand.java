package frc.robot.subsystems.tramp.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.communications.NetworkTableComms;
import frc.robot.subsystems.tramp.TrampSubsystem;

public class ManualTrampCommand extends Command {

    private TrampSubsystem trampSubsystem;
    private NetworkTableComms nt;

    public ManualTrampCommand(
        TrampSubsystem trampSubsystem,
        NetworkTableComms nt
    )
    {
        this.trampSubsystem = trampSubsystem;
        this.nt = nt;
        addRequirements(trampSubsystem);


    }
    @Override
    public void execute() {
        var speed_rpm = this.nt.getManualTramp_rpm();
        this.trampSubsystem.SetVelocity(speed_rpm);
    }
}
