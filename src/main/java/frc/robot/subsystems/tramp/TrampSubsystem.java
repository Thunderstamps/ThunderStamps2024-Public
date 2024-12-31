package frc.robot.subsystems.tramp;

import org.littletonrobotics.junction.Logger;

import com.revrobotics.REVLibError;

import frc.robot.communications.*;
import frc.robot.utilities.*;

public class TrampSubsystem extends ThunderSubsystem {
    private final NetworkTableComms nt;
    private boolean notePresent = false;
    private double setpoint_rpm = 0;

    private final TrampIO io;
    private final TrampIOInputsAutoLogged inputs = new TrampIOInputsAutoLogged();
    
    public TrampSubsystem(
            NetworkTableComms nt,
            TrampIO io) {
        this.nt = nt;
        this.io = io;
    }

    @Override
    public void readInputs() {
        io.updateInputs(inputs);
        Logger.processInputs("TrampSubsystem", inputs);
    }

    @Override
    public void recordOutputs() {
        this.nt.setHasNoteInTramp(this.GetNotePresent());
        Logger.recordOutput("TrampSubsystem/NotePresent", this.GetNotePresent());
        Logger.recordOutput("TrampSubsystem/SetpointRPM", this.setpoint_rpm);
    }

    public REVLibError SetHighCurrentLimit() {
        return io.setHighCurrentLimit();
    }

    public REVLibError SetLowCurrentLimit() {
        return io.setLowCurrentLimit();
    }

    public void SetVelocity(double motorRpm) {
        this.setpoint_rpm = motorRpm;
        io.setSpeed(this.setpoint_rpm);
    }

    public void SetDutyCycle(double dutyCycle){
        this.setpoint_rpm = 0;
        io.setDutyCycle(dutyCycle);
    }
    public void Stop(){
        this.setpoint_rpm = 0;
        io.setDutyCycle(0);
    }

    public double GetVelocityRpm() {
        return inputs.velocity_rpm;
    }

    public double GetStatorCurrent() {
        return inputs.outputCurrent_A;
    }

    public boolean GetNotePresent() {
        return this.notePresent;
    }

    public void SetNotePresent(boolean value) {
        this.notePresent = value;
    }

    public double GetPosition_revs() {
        return this.inputs.position_rot;
    }

    public void ZeroPosition() {
        io.zeroPosition();
    }
}
