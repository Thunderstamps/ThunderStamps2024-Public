package frc.robot.subsystems.swerve.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.swerve.SwerveSubsystem;

public class PitchZeroCommand extends Command {

    private final SwerveSubsystem swerveSubsystem;

    public PitchZeroCommand(
        SwerveSubsystem swerveSubsystem) {
        this.swerveSubsystem = swerveSubsystem;
        this.setName("Reset (Zero) Pitch");
    }

    @Override
    public void initialize() {
        this.swerveSubsystem.zeroPitch();
    }

    @Override
    public boolean isFinished() {
        return true;
    }
}
