package frc.robot.subsystems.swerve.gyros;

import edu.wpi.first.wpilibj.SPI;
import frc.robot.NavX.AHRS;

public class NavX2MxpGyroIOReal implements NavX2MxpGyroIO {
    
    private final AHRS gyro;

    public NavX2MxpGyroIOReal() {
        this.gyro = new AHRS(SPI.Port.kMXP, (byte)50);
        this.gyro.enableLogging(true);
    }

    @Override
    public void updateInputs(NavX2MxpGyroIOInputs inputs) {
        inputs.rotation2d_rad = gyro.getRotation2d().getRadians();
        inputs.rotationRate_rad = Math.toRadians(-gyro.getRate());
        inputs.worldLinearAccelZ_G = gyro.getWorldLinearAccelZ();
        inputs.pitch_deg = gyro.getPitch();
        inputs.roll_deg = gyro.getRoll();
    }

    @Override
    public void reset() {
        this.gyro.reset();
    }

    @Override
    public void zeroYaw() {
        this.gyro.zeroYaw();
    }
}
