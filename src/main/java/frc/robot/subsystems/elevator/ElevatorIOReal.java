package frc.robot.subsystems.elevator;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.MotionMagicTorqueCurrentFOC;
import com.ctre.phoenix6.controls.PositionTorqueCurrentFOC;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import frc.robot.Constants;
import frc.robot.communications.NetworkTableComms;
import frc.robot.utilities.CTREUtil;
import frc.robot.utilities.ThunderSubsystem;

public class ElevatorIOReal implements ElevatorIO {

    private final static double MAX_STATOR_CURRENT_A = ElevatorSubsystem.MAX_STATOR_CURRENT_A;
    private final static double REVERSE_LIMIT_ROTATIONS = ElevatorSubsystem.REVERSE_LIMIT_ROTATIONS;
    private final static double FORWARD_LIMIT_ROTATIONS = ElevatorSubsystem.FORWARD_LIMIT_ROTATIONS;

    private final TalonFX motor = new TalonFX(Constants.ElevatorCanbus, Constants.CANBUS_NAME);
    
    private final StatusSignal<Double> motorPosition = motor.getPosition();
    private final StatusSignal<Double> motorVelocity = motor.getVelocity();
    private final StatusSignal<Double> statorCurrent = motor.getStatorCurrent();
    private final StatusSignal<Double> supplyCurrent = motor.getSupplyCurrent();
    private final StatusSignal<Double> temperature = motor.getDeviceTemp();

    private final DutyCycleOut dutyCycleOut = new DutyCycleOut(0);
    private final PositionTorqueCurrentFOC positionPidControl = new PositionTorqueCurrentFOC(0).withSlot(0);
    private final MotionMagicTorqueCurrentFOC positionMotionControl = new MotionMagicTorqueCurrentFOC(0).withSlot(1);
    
    public ElevatorIOReal(NetworkTableComms nt) {

        // stop it
        var statusCode = CTREUtil.tryUntilOK(() ->
            motor.setControl(this.dutyCycleOut.withOutput(0)), 
            motor.getDeviceID());
        var configStatusCode = this.configureMotor(motor, InvertedValue.CounterClockwise_Positive); // positive is angle up
        nt.setElevatorConfigStatus(configStatusCode);

        // assume it's at the bottom and zero it
        statusCode = CTREUtil.tryUntilOK(() ->
            motor.setPosition(0.0), 
            motor.getDeviceID());
        ThunderSubsystem.printStatusIfBad(statusCode, "Elevator (60) setPosition: ");

        BaseStatusSignal.setUpdateFrequencyForAll(
            50.0,
            motorPosition,
            motorVelocity,
            statorCurrent,
            supplyCurrent,
            temperature);
        CTREUtil.optimizeBusUtilization(motor);
    }

    @Override
    public void updateInputs(ElevatorIOInputs inputs) {
        BaseStatusSignal.refreshAll(
            motorPosition,
            motorVelocity,
            statorCurrent,
            supplyCurrent,
            temperature);
        inputs.motorPosition_revs = BaseStatusSignal.getLatencyCompensatedValue(motorPosition, motorVelocity);
        inputs.motorVelocity_rps = motorVelocity.getValueAsDouble();
        inputs.motorStatorCurrent_A = statorCurrent.getValueAsDouble();
        inputs.motorSupplyCurrent_A = supplyCurrent.getValueAsDouble();
        inputs.motorTemperature_C = temperature.getValueAsDouble();
    }

    private StatusCode configureMotor(TalonFX talonFX, InvertedValue invertedValue) {
        var talonConfiguration = new TalonFXConfiguration();

        // neutral behavior
        var motorOutputConfigs = talonConfiguration.MotorOutput;
        motorOutputConfigs.NeutralMode = NeutralModeValue.Brake;
        motorOutputConfigs.Inverted = invertedValue;

        var currentLimitsConfigs = talonConfiguration.CurrentLimits;
        currentLimitsConfigs.SupplyCurrentLimit = 40;
        currentLimitsConfigs.SupplyCurrentThreshold = 40;
        currentLimitsConfigs.SupplyCurrentLimitEnable = false; // doesn't work in field oriented control mode
        currentLimitsConfigs.StatorCurrentLimit = MAX_STATOR_CURRENT_A;
        currentLimitsConfigs.StatorCurrentLimitEnable = true;

        var torqueCurrentConfig = talonConfiguration.TorqueCurrent;
        torqueCurrentConfig.PeakForwardTorqueCurrent = MAX_STATOR_CURRENT_A;
        torqueCurrentConfig.PeakReverseTorqueCurrent = -MAX_STATOR_CURRENT_A;
        
        // these are the PID position control parameters (holding precise position)
        var slot0Configs = talonConfiguration.Slot0;
        slot0Configs.kV = 0.0;
        slot0Configs.kP = 300.0; // unit is amps of current per rotations per second of error
        slot0Configs.kI = 3.0;
        slot0Configs.kD = 10.0;

        // these are the PID velocity control parameters for motion control mode (long moves)
        var slot1Configs = talonConfiguration.Slot1;
        slot1Configs.kV = 0.1;
        slot1Configs.kP = 80.0; // unit is amps of current per rotations per second of error
        slot1Configs.kI = 0.0;
        slot1Configs.kD = 5.0;

        var motionMagicConfigs = talonConfiguration.MotionMagic;
        motionMagicConfigs.MotionMagicAcceleration = 300.0;
        motionMagicConfigs.MotionMagicCruiseVelocity = 500.0;
        motionMagicConfigs.MotionMagicJerk = 30000.0;

        var openLoopRamps = talonConfiguration.OpenLoopRamps;
        openLoopRamps.DutyCycleOpenLoopRampPeriod = 2;
        
        var closeLoopRamps = talonConfiguration.ClosedLoopRamps;
        closeLoopRamps.DutyCycleClosedLoopRampPeriod = 0;

        var softwareLimitSwitchConfigs = talonConfiguration.SoftwareLimitSwitch;
        softwareLimitSwitchConfigs.ReverseSoftLimitThreshold = REVERSE_LIMIT_ROTATIONS;
        softwareLimitSwitchConfigs.ReverseSoftLimitEnable = true;
        softwareLimitSwitchConfigs.ForwardSoftLimitThreshold = FORWARD_LIMIT_ROTATIONS;
        softwareLimitSwitchConfigs.ForwardSoftLimitEnable = true;

        var statusCode = CTREUtil.applyConfiguration(talonFX, talonConfiguration);
        return statusCode;
    }

    @Override
    public void setDutyCycle(double dutyCycle) {
        var statusCode = CTREUtil.tryUntilOK(() ->
            motor.setControl(this.dutyCycleOut.withOutput(dutyCycle)), 
            motor.getDeviceID());
        ThunderSubsystem.printStatusIfBad(statusCode, "Elevator (60) DutyCycle: ");
    }

    @Override
    public void goToPositionMotionProfile(double target_revs, double feedforward_A) {
        var statusCode = CTREUtil.tryUntilOK(() ->
            motor.setControl(
                positionMotionControl
                    .withPosition(target_revs)
                    .withFeedForward(feedforward_A)), 
            motor.getDeviceID());
        ThunderSubsystem.printStatusIfBad(statusCode, "Elevator (60) MotionMagic: ");
    }

    @Override
    public void goToPositionPid(double target_revs, double feedforward_A) {
        var statusCode = CTREUtil.tryUntilOK(() ->
            motor.setControl(
                positionPidControl
                    .withPosition(target_revs)
                    .withFeedForward(feedforward_A)), 
            motor.getDeviceID());
        ThunderSubsystem.printStatusIfBad(statusCode, "Elevator (60) PID: ");
    }
}
