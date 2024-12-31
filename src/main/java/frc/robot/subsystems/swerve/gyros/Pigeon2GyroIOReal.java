package frc.robot.subsystems.swerve.gyros;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.Pigeon2Configuration;
import com.ctre.phoenix6.hardware.Pigeon2;

import frc.robot.Constants;
import frc.robot.utilities.CTREUtil;
import frc.robot.utilities.ThunderSubsystem;

public class Pigeon2GyroIOReal implements Pigeon2GyroIO {
    
    private final Pigeon2 pigeon2 = new Pigeon2(Constants.Pigeon2Canbus, Constants.CANBUS_NAME);

    private final StatusSignal<Double> yaw = pigeon2.getYaw();
    private final StatusSignal<Double> rotationRate = pigeon2.getAngularVelocityZWorld();
    private final StatusSignal<Double> accelerationZ = pigeon2.getAccelerationZ();
    private final StatusSignal<Double> pitch = pigeon2.getPitch();
    private final StatusSignal<Double> roll = pigeon2.getRoll();
    
    public Pigeon2GyroIOReal() {
        BaseStatusSignal.setUpdateFrequencyForAll(
            200.0, 
            yaw,
            rotationRate,
            accelerationZ,
            pitch,
            roll);
        CTREUtil.optimizeBusUtilization(pigeon2);

        var pigeon2Configuration = new Pigeon2Configuration();
        pigeon2Configuration.Pigeon2Features.EnableCompass = false;
        CTREUtil.applyConfiguration(pigeon2, pigeon2Configuration);
    }

    @Override
    public void updateInputs(Pigeon2GyroIOInputs inputs) {
        BaseStatusSignal.refreshAll(
            yaw,
            rotationRate,
            accelerationZ,
            pitch,
            roll);

        var yaw_deg = BaseStatusSignal.getLatencyCompensatedValue(yaw, rotationRate);
        inputs.rotation2d_rad = Math.toRadians(yaw_deg);

        var rotationRate_deg = rotationRate.getValueAsDouble();
        inputs.rotationRate_rad = Math.toRadians(rotationRate_deg);

        inputs.worldLinearAccelZ_G = accelerationZ.getValueAsDouble();
        inputs.pitch_deg = pitch.getValueAsDouble();
        inputs.roll_deg = roll.getValueAsDouble();
    }

    @Override
    public void zeroYaw() {
        var statusCode = CTREUtil.tryUntilOK(() -> 
            pigeon2.setYaw(0), 
            pigeon2.getDeviceID());
        ThunderSubsystem.printStatusIfBad(statusCode, "Pigeon2 Gyro (40) ZeroYaw: ");
    }
}
