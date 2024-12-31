package frc.robot.subsystems.wallclimb;

import org.littletonrobotics.junction.Logger;

import frc.robot.communications.*;
import frc.robot.utilities.*;

public class WallClimbSubsystem extends ThunderSubsystem {

    private final NetworkTableComms nt;

    private double setpointSpeed_rpm;
    
    private final WallClimbIO io;
    private final WallClimbIOInputsAutoLogged inputs = new WallClimbIOInputsAutoLogged();

    public WallClimbSubsystem(
        NetworkTableComms nt,
        WallClimbIO io)
    {
        this.nt = nt;
        this.io = io;
    }

    @Override
    public void readInputs() {
        io.updateInputs(inputs);
        Logger.processInputs("WallClimbSubsystem", inputs);
    }

    @Override
    public void recordOutputs() {
        this.nt.setWallClimbSpeed(this.GetVelocityRpm());
        Logger.recordOutput("WallClimbSubsystem/SetpointSpeed_rpm", this.setpointSpeed_rpm);
    }

    public void RunForward(double speed_rpm) {
        this.setpointSpeed_rpm = speed_rpm;
        io.setSpeed(this.setpointSpeed_rpm);
    }

    public void Stop() {
        this.setpointSpeed_rpm = 0;
        io.setDutyCycle(0);
    }

    public double GetVelocityRpm() {
        return inputs.velocity_rpm;
    }

    public double GetCurrentA() {
        return inputs.outputCurrent_A;
    }
}
