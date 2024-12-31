package frc.robot.subsystems.angle.commands;

import edu.wpi.first.wpilibj2.command.*;
import frc.robot.subsystems.angle.*;
import frc.robot.targeting.*;
import frc.robot.utilities.OnDelayTimer;

public class AngleTargetCommand extends Command {
    
    private final AngleSubsystem angleSubsystem;
    private final ITargetingController targetingController;
    private final OnDelayTimer onTargetTimer = new OnDelayTimer(100);

    public AngleTargetCommand(
            AngleSubsystem angleSubsystem,
            ITargetingController targetingController) {
        this.angleSubsystem = angleSubsystem;
        this.targetingController = targetingController;
        this.addRequirements(angleSubsystem);
    }

    @Override
    public void initialize() {
        this.onTargetTimer.execute(false);
        this.targetingController.SetTargetStatusAngle(false);
    }

    @Override
    public void execute() {
        var optionAngle_deg = this.targetingController.getShooterAngle_deg();
        double position_deg;
        if(optionAngle_deg.isPresent()) {
            position_deg = optionAngle_deg.getAsDouble();

            var diff_deg = position_deg - this.angleSubsystem.GetAngleMotor_deg();
            var onTarget = Math.abs(diff_deg) < 0.25;
            this.onTargetTimer.execute(onTarget);
        }
        else {
            position_deg = 20.0;
            this.onTargetTimer.execute(false);
        }
        this.angleSubsystem.SetPosition(position_deg);
        this.targetingController.SetTargetStatusAngle(this.onTargetTimer.getOutput());
    }

    @Override
    public void end(boolean interrupted) {
        this.targetingController.SetTargetStatusAngle(false);
    }
}
