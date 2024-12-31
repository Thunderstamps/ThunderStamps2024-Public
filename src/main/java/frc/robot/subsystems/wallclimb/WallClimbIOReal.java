package frc.robot.subsystems.wallclimb;

import com.revrobotics.CANSparkBase.ControlType;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;
import com.revrobotics.REVLibError;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkPIDController;

import frc.robot.Constants;
import frc.robot.communications.NetworkTableComms;

public class WallClimbIOReal implements WallClimbIO {
    
    private final CANSparkMax motor = new CANSparkMax(Constants.WallClimbCanbus, MotorType.kBrushless);
    private final SparkPIDController pidController;
    private final RelativeEncoder motorEncoder;
    
    public WallClimbIOReal(NetworkTableComms nt) {
        
        this.pidController = motor.getPIDController();
        this.motorEncoder = motor.getEncoder();
        
        var err = this.configure();
        nt.setWallClimbConfigStatus(err);

    }

    @Override
    public void updateInputs(WallClimbIOInputs inputs) {
        inputs.velocity_rpm = this.motorEncoder.getVelocity();
        inputs.outputCurrent_A = this.motor.getOutputCurrent();
        inputs.appliedOutput = this.motor.getAppliedOutput();
    }

    private REVLibError configure() {
        REVLibError result;

        result = motor.setSmartCurrentLimit(160); 
        printErrorIfAny(result, "setSmartCurrentLimit");
        if(result != REVLibError.kOk) return result;

        result = motor.enableVoltageCompensation(12);
        printErrorIfAny(result, "enableVoltageCompensation");
        if(result != REVLibError.kOk) return result;

        motor.setInverted(true);
        
        result = motor.setIdleMode(IdleMode.kBrake);
        printErrorIfAny(result, "setIdleMode");
        if(result != REVLibError.kOk) return result;

        result = this.setPid();
        if(result != REVLibError.kOk) return result;

        result = this.motor.burnFlash();
        printErrorIfAny(result, "burnFlash");
        if(result != REVLibError.kOk) return result;

        return result;
    }

    REVLibError setPid() {
        REVLibError result;

        result = pidController.setP(0.00003);
        printErrorIfAny(result, "setFF");
        if(result != REVLibError.kOk) return result;

        result = pidController.setFF(0.000163);
        printErrorIfAny(result, "setP");
        if(result != REVLibError.kOk) return result;

        return result;
    }

    @Override
    public void setDutyCycle(double dutyCycle) {
        var err = this.pidController.setReference(dutyCycle, ControlType.kDutyCycle);
        printErrorIfAny(err, "setDutyCycle");
    }

    @Override
    public void setSpeed(double speed_rpm) {
        var err = this.pidController.setReference(speed_rpm, ControlType.kVelocity);
        printErrorIfAny(err, "setSpeed");
    }

    private void printErrorIfAny(REVLibError err, String label) {
        if(err != REVLibError.kOk) {
            System.out.println("WallClimb (" + label + ") " + err.name());
        }
    }
}
