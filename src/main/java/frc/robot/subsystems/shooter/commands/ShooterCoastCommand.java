package frc.robot.subsystems.shooter.commands;

import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.shooter.ShooterSubsystem;

public class ShooterCoastCommand extends Command {
    private final ShooterSubsystem shooterSubsystem;
    private final double untilVoltageAbove;

    public ShooterCoastCommand(
            ShooterSubsystem shooterSubsystem,
            double untilVoltageAbove) {
        this.shooterSubsystem = shooterSubsystem;
        this.untilVoltageAbove = untilVoltageAbove;
        this.addRequirements(shooterSubsystem);
    }

    @Override
    public void initialize() {
        this.shooterSubsystem.CoastShooter();
    }

    @Override
    public boolean isFinished() {
        return RobotController.getInputVoltage() >= this.untilVoltageAbove;
    }
}
