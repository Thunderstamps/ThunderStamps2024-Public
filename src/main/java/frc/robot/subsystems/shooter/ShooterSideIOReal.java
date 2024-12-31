package frc.robot.subsystems.shooter;

import com.revrobotics.CANSparkBase.ControlType;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkFlex;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.REVLibError;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkLimitSwitch;
import com.revrobotics.SparkPIDController;

public class ShooterSideIOReal implements ShooterSideIO {

    private final int canbusId;

    private final CANSparkFlex motor;
    private final RelativeEncoder encoder;
    private final SparkPIDController pid;
    
    public ShooterSideIOReal(int canbusId) {
        this.canbusId = canbusId;
        this.motor = new CANSparkFlex(canbusId, MotorType.kBrushless);
        this.encoder = this.motor.getEncoder();
        this.pid = this.motor.getPIDController();
    }

    @Override
    public void updateInputs(ShooterSideIOInputs inputs) {
        inputs.velocity_rpm = this.encoder.getVelocity();
        inputs.outputCurrent_A = this.motor.getOutputCurrent();
        inputs.appliedOutput = this.motor.getAppliedOutput();
        inputs.busVoltage = this.motor.getBusVoltage();
    }

    public REVLibError configure(boolean invert, double feedForwardGain) {
        REVLibError result;

        result = this.motor.restoreFactoryDefaults();
        printErrorIfAny(result, "restoreFactoryDefaults");
        if(result != REVLibError.kOk) return result;

        result = this.motor.setSmartCurrentLimit(65);
        printErrorIfAny(result, "setSmartCurrentLimit");
        if(result != REVLibError.kOk) return result;

        this.motor.setInverted(invert);

        result = this.motor.setIdleMode(IdleMode.kCoast);
        printErrorIfAny(result, "setInverted");
        if(result != REVLibError.kOk) return result;

        var forwardLimit = this.motor.getForwardLimitSwitch(SparkLimitSwitch.Type.kNormallyClosed);
        result = forwardLimit.enableLimitSwitch(false);
        printErrorIfAny(result, "disableLimitSwitch-forward");
        if(result != REVLibError.kOk) return result;

        var reverseLimit = this.motor.getReverseLimitSwitch(SparkLimitSwitch.Type.kNormallyClosed);
        result = reverseLimit.enableLimitSwitch(false);
        printErrorIfAny(result, "disableLimitSwitch-reverse");
        if(result != REVLibError.kOk) return result;

        result = this.setPid(feedForwardGain);
        if(result != REVLibError.kOk) return result;

        result = this.motor.burnFlash();
        printErrorIfAny(result, "burnFlash");
        return result;
    }

    private REVLibError setPid(double feedForwardGain) {
        REVLibError result;
        result = this.pid.setFF(feedForwardGain);
        printErrorIfAny(result, "setFF");
        if(result != REVLibError.kOk) return result;

        result = this.pid.setP(0.0020); // 0.0020
        printErrorIfAny(result, "setP");
        return result;
    }

    private void printErrorIfAny(REVLibError err, String label) {
        if(err != REVLibError.kOk) {
            System.out.println("Shooter " + String.valueOf(this.canbusId) + " (" + label + ") " + err.name());
        }
    }

    @Override
    public void setVelocity(double velocity_rpm) {
        var err = this.pid.setReference(velocity_rpm, ControlType.kVelocity);
        printErrorIfAny(err, "SetVelocity");
    }

    @Override
    public void coast() {
        this.motor.set(0);
    }
}
