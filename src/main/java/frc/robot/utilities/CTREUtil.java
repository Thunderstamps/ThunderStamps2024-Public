package frc.robot.utilities;

import java.util.function.Supplier;

import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.Pigeon2Configuration;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.ParentDevice;
import com.ctre.phoenix6.hardware.Pigeon2;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.wpilibj.DriverStation;

public class CTREUtil {

    private static final int MAX_RETRIES = 10;

    public static StatusCode tryUntilOK(Supplier<StatusCode> function, int deviceId) {
        StatusCode statusCode = StatusCode.OK;
        for (int i = 0; i < MAX_RETRIES; ++i) {
            statusCode = function.get();
            if (statusCode == StatusCode.OK)
                break;
        }
        if (statusCode != StatusCode.OK) {
            DriverStation.reportError(
                    "Error calling " + function + " on ctre device id " + deviceId + ": " + statusCode, true);
        }
        return statusCode;
    }

    public static StatusCode applyConfiguration(TalonFX motor, TalonFXConfiguration config) {
        return tryUntilOK(() -> motor.getConfigurator().apply(config), motor.getDeviceID());
    }

    public static StatusCode applyConfiguration(TalonFX motor, CurrentLimitsConfigs config) {
        return tryUntilOK(() -> motor.getConfigurator().apply(config), motor.getDeviceID());
    }

    public static StatusCode applyConfiguration(TalonFX motor, MotionMagicConfigs config) {
        return tryUntilOK(() -> motor.getConfigurator().apply(config), motor.getDeviceID());
    }

    public static StatusCode applyConfiguration(CANcoder cancoder, CANcoderConfiguration config) {
        return tryUntilOK(() -> cancoder.getConfigurator().apply(config), cancoder.getDeviceID());
    }

    public static StatusCode applyConfiguration(Pigeon2 pigeon2, Pigeon2Configuration config) {
        return tryUntilOK(() -> pigeon2.getConfigurator().apply(config), pigeon2.getDeviceID());
    }

    public static StatusCode refreshConfiguration(TalonFX motor, TalonFXConfiguration config) {
        return tryUntilOK(() -> motor.getConfigurator().refresh(config), motor.getDeviceID());
    }

    public static StatusCode optimizeBusUtilization(ParentDevice device) {
        return tryUntilOK(() -> device.optimizeBusUtilization(), device.getDeviceID());
    }
}
