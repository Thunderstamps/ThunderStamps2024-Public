package frc.robot.subsystems.swerve.steer;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.PositionTorqueCurrentFOC;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import frc.robot.Constants;
import frc.robot.subsystems.swerve.util.SwerveModuleConstants;
import frc.robot.utilities.CTREUtil;

public class SteeringControllerIOReal implements SteeringControllerIO {

    private final static double MAX_STATOR_CURRENT_A = SteeringControllerTalonFXv6.MAX_STATOR_CURRENT_A;
    
    private final TalonFX talon;
    
    private final StatusSignal<Double> motorPosition;
    private final StatusSignal<Double> motorVelocity;
    private final StatusSignal<Double> statorCurrent;
    private final StatusSignal<Double> supplyCurrent;
    private final StatusSignal<Double> temperature;

    private final DutyCycleOut percentControl = new DutyCycleOut(0);
    private final PositionTorqueCurrentFOC positionPidControl = new PositionTorqueCurrentFOC(0).withSlot(0);

    private final CANcoder homeSensor;

    private final StatusSignal<Double> homeSensorPosition;
    
    public SteeringControllerIOReal(SwerveModuleConstants swerveModuleConstants) {

        this.talon = new TalonFX(swerveModuleConstants.SteerCanbus, Constants.CANBUS_NAME);
        this.motorPosition = talon.getPosition();
        this.motorVelocity = talon.getVelocity();
        this.statorCurrent = talon.getStatorCurrent();
        this.supplyCurrent = talon.getSupplyCurrent();
        this.temperature = talon.getDeviceTemp();

        this.homeSensor = new CANcoder(swerveModuleConstants.EncoderCanbus, Constants.CANBUS_NAME);
        this.homeSensorPosition = this.homeSensor.getAbsolutePosition();

        BaseStatusSignal.setUpdateFrequencyForAll(
            200.0,
            motorPosition,
            motorVelocity,
            statorCurrent,
            supplyCurrent,
            temperature);
        CTREUtil.optimizeBusUtilization(talon);

        BaseStatusSignal.setUpdateFrequencyForAll(
            50.0,
            homeSensorPosition);
        CTREUtil.optimizeBusUtilization(homeSensor);
        
        // configure the CANcoder to give us raw counts (0 to 4095)
        var cancoderFactoryDefaults = new CANcoderConfiguration();
        CTREUtil.applyConfiguration(homeSensor, cancoderFactoryDefaults);
        
        // stop it
        CTREUtil.tryUntilOK(() -> 
            talon.setControl(this.percentControl.withOutput(0)), 
            talon.getDeviceID());

        var talonConfiguration = new TalonFXConfiguration();
        
        // neutral behavior
        var motorOutputConfigs = talonConfiguration.MotorOutput;
        motorOutputConfigs.NeutralMode = NeutralModeValue.Brake;
        motorOutputConfigs.Inverted = InvertedValue.CounterClockwise_Positive; // counter-clockwise should be positive;

        // current limits
        var currentLimitsConfigs = talonConfiguration.CurrentLimits;
        currentLimitsConfigs.SupplyCurrentLimit = 40;
        currentLimitsConfigs.SupplyCurrentThreshold = 40;
        currentLimitsConfigs.SupplyCurrentLimitEnable = false;
        currentLimitsConfigs.StatorCurrentLimit = MAX_STATOR_CURRENT_A;
        currentLimitsConfigs.StatorCurrentLimitEnable = true;

        var torqueCurrentConfig = talonConfiguration.TorqueCurrent;
        torqueCurrentConfig.PeakForwardTorqueCurrent = MAX_STATOR_CURRENT_A;
        torqueCurrentConfig.PeakReverseTorqueCurrent = -MAX_STATOR_CURRENT_A;

        var slot0Configs = talonConfiguration.Slot0;
        slot0Configs.kV = 0; // in phoenix 5 this was 0.1443 = 0.0289
        slot0Configs.kP = 300 * Constants.SWERVE_MOTOR_REVS_PER_STEERING_REV; // unit: amps per rotation of error. Max we should have to move is 3.2 motor rotations
        slot0Configs.kI = 0;
        slot0Configs.kD = 5 * Constants.SWERVE_MOTOR_REVS_PER_STEERING_REV;
        slot0Configs.kS = 0; // how many amps to get it moving

        var openLoopRamps = talonConfiguration.OpenLoopRamps;
        openLoopRamps.DutyCycleOpenLoopRampPeriod = 0;
        
        var closeLoopRamps = talonConfiguration.ClosedLoopRamps;
        closeLoopRamps.DutyCycleClosedLoopRampPeriod = 0;
        closeLoopRamps.TorqueClosedLoopRampPeriod = Constants.SCAN_TIME_S / 2.0;
        closeLoopRamps.VoltageClosedLoopRampPeriod = 0;

        // Enable fused CANCoder
        var feedback = talonConfiguration.Feedback;
        feedback.FeedbackRemoteSensorID = this.homeSensor.getDeviceID();
        feedback.FeedbackSensorSource = FeedbackSensorSourceValue.FusedCANcoder;
        feedback.SensorToMechanismRatio = 1.0;
        feedback.RotorToSensorRatio = Constants.SWERVE_MOTOR_REVS_PER_STEERING_REV;
        
        CTREUtil.applyConfiguration(talon, talonConfiguration);
    }

    @Override
    public void updateInputs(SteeringControllerIOInputs inputs) {
        BaseStatusSignal.refreshAll(
            motorPosition,
            motorVelocity,
            statorCurrent,
            supplyCurrent,
            homeSensorPosition,
            temperature);
        inputs.motorPosition_revs = BaseStatusSignal.getLatencyCompensatedValue(motorPosition, motorVelocity);
        inputs.motorVelocity_rps = motorVelocity.getValueAsDouble();
        inputs.motorStatorCurrent_A = statorCurrent.getValueAsDouble();
        inputs.motorSupplyCurrent_A = supplyCurrent.getValueAsDouble();
        inputs.homeSensorPosition_revs = homeSensorPosition.getValueAsDouble();
        inputs.homeSensorPosition_counts = (int)(inputs.homeSensorPosition_revs * SteeringControllerTalonFXv6.CANCODER_COUNTS);
        inputs.motorTemperature_C = temperature.getValueAsDouble();
    }

    @Override
    public void setDutyCycle(double dutyCycle) {
        CTREUtil.tryUntilOK(() -> 
            talon.setControl(percentControl.withOutput(dutyCycle)), 
            talon.getDeviceID());
    }

    @Override
    public void goToPositionPid(double target_revs) {
        CTREUtil.tryUntilOK(() -> 
            talon.setControl(positionPidControl.withPosition(target_revs)), 
            talon.getDeviceID());
    }
}
