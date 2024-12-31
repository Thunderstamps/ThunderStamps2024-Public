package frc.robot.subsystems.elevator.commands;

import edu.wpi.first.wpilibj.*;
import edu.wpi.first.wpilibj2.command.*;
import edu.wpi.first.wpilibj2.command.button.CommandJoystick;
import frc.robot.subsystems.elevator.*;

public class ElevatorManualCommand extends Command {
    private ElevatorSubsystem elevatorSubsystem;
    private Joystick joystick;

    public ElevatorManualCommand(
            ElevatorSubsystem elevatorSubsystem,
            CommandJoystick joystick) {
        this.elevatorSubsystem = elevatorSubsystem;
        this.joystick = joystick.getHID();
        this.addRequirements(elevatorSubsystem);
    }

    @Override
    public void initialize() {
        this.elevatorSubsystem.SetDutyCycle(0.0);
    }

    @Override
    public void execute() {
        var throttle = -this.joystick.getThrottle(); // axis is backwards
        var mid_in = (ElevatorSubsystem.MAX_POSITION_IN + ElevatorSubsystem.MIN_POSITION_IN) / 2.0;
        var range_in = (ElevatorSubsystem.MAX_POSITION_IN - ElevatorSubsystem.MIN_POSITION_IN) / 2.0;
        var position_in = throttle * range_in + mid_in;
        this.elevatorSubsystem.SetPosition(position_in);
    }

    @Override
    public void end(boolean interrupted) {
        this.elevatorSubsystem.SetDutyCycle(0.0);
    }
}
