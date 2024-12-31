package frc.robot.subsystems.shooter;

import org.littletonrobotics.junction.Logger;

import com.revrobotics.REVLibError;

public class ShooterSide {

    private final boolean invert;
    private final double feedForwardGain;
    
    private final ShooterSideIO io;
    private final ShooterSideIOInputsAutoLogged inputs = new ShooterSideIOInputsAutoLogged();

    private double setpointRpm;
    
    public ShooterSide(boolean invert, double feedForwardGain, ShooterSideIO io) {
        this.invert = invert;
        this.feedForwardGain = feedForwardGain;
        this.io = io;
    }

    public void readInputs(String leftOrRight) {
        io.updateInputs(inputs);
        Logger.processInputs("ShooterSubsystem/" + leftOrRight, inputs);
    }

    public void recordOutputs(String leftOrRight) {
        Logger.recordOutput("ShooterSubsystem/" + leftOrRight + "/SetpointRPM", this.setpointRpm);
    }

    public void SetVelocity(double motorRpm) {
        io.setVelocity(motorRpm);
        this.setpointRpm = motorRpm;
    }

    public void Coast() {
        io.coast();
        this.setpointRpm = 0;
    }

    public double GetVelocityRpm() {
        return inputs.velocity_rpm;
    }

    public double GetStatorCurrent() {
        return inputs.outputCurrent_A;
    }

    public double GetAppliedVoltage() {
        return inputs.busVoltage
            * this.GetDutyCycle();
    }

    public double GetDutyCycle() {
        return inputs.appliedOutput;
    }

    public REVLibError configure() {
        return io.configure(this.invert, this.feedForwardGain);
    }
}
