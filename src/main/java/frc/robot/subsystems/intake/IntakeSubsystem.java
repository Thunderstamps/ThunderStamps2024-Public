package frc.robot.subsystems.intake;

import org.littletonrobotics.junction.Logger;

import frc.robot.communications.NetworkTableComms;
import frc.robot.utilities.*;

public class IntakeSubsystem extends ThunderSubsystem {
    
    private boolean intaking = false;
    private double setpointSpeed_rpm;

    private final IntakeIO io;
    private final IntakeIOInputsAutoLogged inputs = new IntakeIOInputsAutoLogged();
    
    public IntakeSubsystem(NetworkTableComms nt, IntakeIO io) {
        this.io = io;
    }

    @Override
    public void readInputs() {
        this.io.updateInputs(inputs);
        Logger.processInputs("IntakeSubsystem", inputs);
    }

    @Override
    public void recordOutputs() {
        Logger.recordOutput("IntakeSubsystem/IsIntaking", this.IsIntaking());
        Logger.recordOutput("IntakeSubsystem/SetpointRPM", this.setpointSpeed_rpm);
    }

    public boolean IsIntaking() {
        return this.intaking;
    }

    public void IntakeNote() {
        this.setSpeed(3000.0);
        this.intaking = true;
    }

    public void TransferNote() {
        this.setSpeed(500.0);
        this.intaking = false;
    }

    public void Stop() {
        this.setSpeed(0.0);
        this.intaking = false;
    }

    public void RunReverse() {
        this.setSpeed(-2000.0);
        this.intaking = false;
    }

    private void setSpeed(double speed_rpm) {
        this.setpointSpeed_rpm = speed_rpm;
        io.setSpeed(this.setpointSpeed_rpm);
    }

    public double GetVelocityRpm() {
        return inputs.velocity_rpm;
    }

    public double GetCurrentA() {
        return inputs.outputCurrent_A;
    }

    public double GetDutyCycle() {
        return inputs.appliedOutput;
    }
}
