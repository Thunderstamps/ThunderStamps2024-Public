package frc.robot.subsystems.swerve.drive;

import com.ctre.phoenix6.*;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.*;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.*;

import frc.robot.Constants;
import frc.robot.subsystems.swerve.util.SwerveModuleConstants;
import frc.robot.utilities.CTREUtil;

public class DriveControllerIOReal implements DriveControllerIO {

    private final static double MAX_STATOR_CURRENT_A = DriveControllerTalonFXv6.MAX_STATOR_CURRENT_A;
    private final static double MAX_INPUT_CURRENT_A = DriveControllerTalonFXv6.MAX_INPUT_CURRENT_A;
    
    private final TalonFX talon;
    
    private final StatusSignal<Double> motorPosition;
    private final StatusSignal<Double> absolutePosition;
    private final StatusSignal<Double> motorVelocity;
    private final StatusSignal<Double> statorCurrent;
    private final StatusSignal<Double> supplyCurrent;
    private final StatusSignal<Double> temperature;

    private final DutyCycleOut percentControl = new DutyCycleOut(0);
    private final VelocityVoltage velocityControl = new VelocityVoltage(0)
        .withSlot(0);

    public DriveControllerIOReal(SwerveModuleConstants swerveModuleConstants) {

        talon = new TalonFX(swerveModuleConstants.DriveCanbus, Constants.CANBUS_NAME);
        motorPosition = talon.getPosition();
        absolutePosition = talon.getRotorPosition();
        motorVelocity = talon.getVelocity();
        statorCurrent = talon.getStatorCurrent();
        supplyCurrent = talon.getSupplyCurrent();
        temperature = talon.getDeviceTemp();

        BaseStatusSignal.setUpdateFrequencyForAll(
            200.0,
            motorPosition,
            absolutePosition,
            motorVelocity,
            statorCurrent,
            supplyCurrent,
            temperature);
        CTREUtil.optimizeBusUtilization(talon);
        
        var talonConfiguration = new TalonFXConfiguration();

        // neutral behavior
        var motorOutputConfigs = talonConfiguration.MotorOutput;
        motorOutputConfigs.NeutralMode = NeutralModeValue.Brake;
        motorOutputConfigs.Inverted = InvertedValue.CounterClockwise_Positive; // counter-clockwise should be positive;

        // stop it
        talon.setControl(this.percentControl.withOutput(0));

        var currentLimitsConfigs = talonConfiguration.CurrentLimits;
        currentLimitsConfigs.SupplyCurrentLimit = MAX_INPUT_CURRENT_A;
        currentLimitsConfigs.SupplyCurrentThreshold = 0;
        currentLimitsConfigs.SupplyTimeThreshold = 0;
        currentLimitsConfigs.SupplyCurrentLimitEnable = true; // note: doesn't work in Field Oriented Control mode
        currentLimitsConfigs.StatorCurrentLimit = MAX_STATOR_CURRENT_A;
        currentLimitsConfigs.StatorCurrentLimitEnable = true; // note: doesn't work in Field Oriented Control mode

        var torqueCurrentConfig = talonConfiguration.TorqueCurrent;
        torqueCurrentConfig.PeakForwardTorqueCurrent = MAX_STATOR_CURRENT_A;
        torqueCurrentConfig.PeakReverseTorqueCurrent = -MAX_STATOR_CURRENT_A;
        
        var slot0Configs = talonConfiguration.Slot0;
        slot0Configs.kV = 12.0 / 88.2142857143;
        slot0Configs.kP = 0.35; 
        slot0Configs.kI = 0;
        slot0Configs.kD = 0;

        var openLoopRamps = talonConfiguration.OpenLoopRamps;
        openLoopRamps.DutyCycleOpenLoopRampPeriod = 2;
        
        var closeLoopRamps = talonConfiguration.ClosedLoopRamps;
        closeLoopRamps.DutyCycleClosedLoopRampPeriod = Constants.SCAN_TIME_S / 2.0;
        closeLoopRamps.TorqueClosedLoopRampPeriod = Constants.SCAN_TIME_S / 2.0;
        closeLoopRamps.VoltageClosedLoopRampPeriod = Constants.SCAN_TIME_S / 2.0;

        CTREUtil.applyConfiguration(talon, talonConfiguration);
    }

    @Override
    public void updateInputs(DriveControllerIOInputs inputs) {
        BaseStatusSignal.refreshAll(
            motorPosition,
            absolutePosition,
            motorVelocity,
            statorCurrent,
            supplyCurrent,
            temperature);
        inputs.motorPosition_revs = BaseStatusSignal.getLatencyCompensatedValue(motorPosition, motorVelocity);
        inputs.absolutePosition_revs = absolutePosition.getValueAsDouble();
        inputs.motorVelocity_rps = motorVelocity.getValueAsDouble();
        inputs.motorStatorCurrent_A = statorCurrent.getValueAsDouble();
        inputs.motorSupplyCurrent_A = supplyCurrent.getValueAsDouble();
        inputs.motorTemperature_C = temperature.getValueAsDouble();
    }

    @Override
    public void setDutyCycle(double dutyCycle) {
        CTREUtil.tryUntilOK(() -> 
            talon.setControl(percentControl.withOutput(dutyCycle)), 
            talon.getDeviceID());
    }

    @Override
    public void setVelocity(double rotations_per_sec) {
        CTREUtil.tryUntilOK(() -> 
            talon.setControl(velocityControl.withVelocity(rotations_per_sec)), 
            talon.getDeviceID());
    }
}
