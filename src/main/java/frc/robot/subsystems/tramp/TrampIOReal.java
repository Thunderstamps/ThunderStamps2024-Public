package frc.robot.subsystems.tramp;

import com.revrobotics.CANSparkBase.ControlType;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.*;
import com.revrobotics.CANSparkLowLevel.*;

import frc.robot.Constants;
import frc.robot.communications.NetworkTableComms;

public class TrampIOReal implements TrampIO {
    
    private final CANSparkFlex motor = new CANSparkFlex(Constants.TrampCanbus, MotorType.kBrushless);
    private final RelativeEncoder encoder;
    private final SparkPIDController pid;
    
    public TrampIOReal(NetworkTableComms nt) {

        this.encoder = this.motor.getEncoder();
        this.pid = this.motor.getPIDController();

        var err = configure();
        nt.setTrampConfigStatus(err);
    }

    @Override
    public void updateInputs(TrampIOInputs inputs) {
        inputs.velocity_rpm = this.encoder.getVelocity();
        inputs.position_rot = this.encoder.getPosition();
        inputs.outputCurrent_A = this.motor.getOutputCurrent();
        inputs.appliedOutput = this.motor.getAppliedOutput();
    }

    private REVLibError configure() {
        REVLibError result;

        result = this.motor.enableVoltageCompensation(7);
        printErrorIfAny(result, "enableVoltageCompensation");
        if(result != REVLibError.kOk) return result;

        this.motor.setInverted(false);

        result = this.motor.setIdleMode(IdleMode.kBrake);
        printErrorIfAny(result, "setIdleMode");
        if(result != REVLibError.kOk) return result;

        result = this.setPid();
        if(result != REVLibError.kOk) return result;

        result = this.setLowCurrentLimit();
        if(result != REVLibError.kOk) return result;

        result = this.motor.burnFlash();
        printErrorIfAny(result, "burnFlash");
        if(result != REVLibError.kOk) return result;

        return result;
    }

    private REVLibError setPid() {
        REVLibError result;

        result = this.pid.setFF(0.0001);
        printErrorIfAny(result, "setFF");
        if(result != REVLibError.kOk) return result;

        result = this.pid.setP(0.000155); 
        printErrorIfAny(result, "setP");
        if(result != REVLibError.kOk) return result;

        return result;
    }

    @Override
    public void setDutyCycle(double dutyCycle) {
        this.motor.set(dutyCycle);
    }

    @Override
    public void zeroPosition() {
        this.encoder.setPosition(0);
    }

    @Override
    public void setSpeed(double speed_rpm) {
        var err = this.pid.setReference(speed_rpm, ControlType.kVelocity);
        printErrorIfAny(err, "SetVelocity");
    }

    @Override
    public REVLibError setHighCurrentLimit() {
        REVLibError result;

        result = this.motor.setSmartCurrentLimit(150);
        printErrorIfAny(result, "setSmartCurrentLimit-high");
        if(result != REVLibError.kOk) return result;

        return result;
    }

    @Override
    public REVLibError setLowCurrentLimit() {
        REVLibError result;

        result = this.motor.setSecondaryCurrentLimit(30);
        printErrorIfAny(result, "setSecondaryCurrentLimit-low");
        if(result != REVLibError.kOk) return result;

        result = this.motor.setSmartCurrentLimit(20);
        printErrorIfAny(result, "setSmartCurrentLimit-low");
        if(result != REVLibError.kOk) return result;

        return result;
    }

    private void printErrorIfAny(REVLibError err, String label) {
        if(err != REVLibError.kOk) {
            System.out.println("Tramp (" + label + ") " + err.name());
        }
    }
}
