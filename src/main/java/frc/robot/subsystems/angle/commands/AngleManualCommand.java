package frc.robot.subsystems.angle.commands;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.angle.AngleSubsystem;

public class AngleManualCommand extends Command {
    
    private final AngleSubsystem angleSubsystem;
    private final Joystick joystick;
    private static final double MID_DEG = (AngleSubsystem.MAX_ANGLE_DEG + AngleSubsystem.MIN_ANGLE_DEG) / 2.0;
    private static final double PLUSMINUS_DEG = (AngleSubsystem.MAX_ANGLE_DEG - AngleSubsystem.MIN_ANGLE_DEG) / 2.0;

    public AngleManualCommand(
            AngleSubsystem angleSubsystem,
            Joystick joystick) {
        this.angleSubsystem = angleSubsystem;
        this.joystick = joystick;
        this.addRequirements(angleSubsystem);
    }

    @Override
    public void execute() {
        var t = -this.joystick.getThrottle();
        var position_deg = MID_DEG + PLUSMINUS_DEG * t;
        this.angleSubsystem.SetPosition(position_deg);
    }
}
