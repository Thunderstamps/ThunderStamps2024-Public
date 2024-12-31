package frc.robot.subsystems.elevator.commands;

import edu.wpi.first.wpilibj2.command.*;
import frc.robot.subsystems.elevator.ElevatorSubsystem;

public class ElevatorDefaultCommand extends Command {
    
    private final ElevatorSubsystem elevatorSubsystem;

    public ElevatorDefaultCommand(
            ElevatorSubsystem elevatorSubsystem) {
        this.elevatorSubsystem = elevatorSubsystem;
        this.addRequirements(elevatorSubsystem);
    }

    @Override
    public void execute() {
        this.elevatorSubsystem.GoToPositionDown();
    }
}
