package frc.robot.subsystems.intake.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.intake.IntakeSubsystem;

public class RunIntakeCommand extends Command {
    
    private IntakeSubsystem intakeSubsystem;

    public RunIntakeCommand(IntakeSubsystem intakeSubsystem) {
        this.intakeSubsystem = intakeSubsystem;
        addRequirements(intakeSubsystem);
    }

    @Override
    public void initialize() {
        this.intakeSubsystem.IntakeNote();;
        System.out.println("Start intake run");
    }

    @Override
    public boolean isFinished() {
        return false;
    }

    @Override
    public void end(boolean interrupted) {
        this.intakeSubsystem.Stop();
        System.out.println("End intake run");
    }
}
