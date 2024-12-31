package frc.robot.subsystems.angle;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.MotionMagicTorqueCurrentFOC;
import com.ctre.phoenix6.controls.PositionTorqueCurrentFOC;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.wpilibj.DigitalInput;
import frc.robot.Constants;
import frc.robot.communications.NetworkTableComms;
import frc.robot.utilities.CTREUtil;
import frc.robot.utilities.ThunderSubsystem;

public class AngleIOReal implements AngleIO {

    public static final double MAX_STATOR_CURRENT_A = AngleSubsystem.MAX_STATOR_CURRENT_A;
    private static final double FAST_VELOCITY_RPS = AngleSubsystem.FAST_VELOCITY_RPS;

    private static final double ACCELERATION_RPS2 = AngleSubsystem.ACCELERATION_RPS2;
    private static final double JERK_RPS3 = AngleSubsystem.JERK_RPS3;
    
    private final TalonFX leftMotor = new TalonFX(Constants.AngleLeftCanbus, Constants.CANBUS_NAME);
    private final TalonFX rightMotor = new TalonFX(Constants.AngleRightCanbus, Constants.CANBUS_NAME);

    private final StatusSignal<Double> motorAbsolute = leftMotor.getRotorPosition();
    private final StatusSignal<Double> motorPosition = leftMotor.getPosition();
    private final StatusSignal<Double> motorVelocity = leftMotor.getVelocity();
    private final StatusSignal<Double> leftStatorCurrent = leftMotor.getStatorCurrent();
    private final StatusSignal<Double> leftSupplyCurrent = leftMotor.getSupplyCurrent();
    private final StatusSignal<Double> leftTemperature = leftMotor.getDeviceTemp();
    private final StatusSignal<Double> rightStatorCurrent = rightMotor.getStatorCurrent();
    private final StatusSignal<Double> rightSupplyCurrent = rightMotor.getSupplyCurrent();
    private final StatusSignal<Double> rightTemperature = rightMotor.getDeviceTemp();

    private final DutyCycleOut dutyCycleOut = new DutyCycleOut(0);
    private final PositionTorqueCurrentFOC positionPidControl = new PositionTorqueCurrentFOC(0).withSlot(0);
    private final MotionMagicTorqueCurrentFOC positionMotionControl = new MotionMagicTorqueCurrentFOC(0).withSlot(1);
    private final Follower followLeftMotor;
    private final DigitalInput homeSensor = new DigitalInput(1);

    public AngleIOReal(NetworkTableComms nt) {

        // stop it
        this.leftMotor.setControl(this.dutyCycleOut.withOutput(0));
        this.rightMotor.setControl(this.dutyCycleOut.withOutput(0));

        var leftStatusCode = configureMotor(leftMotor, InvertedValue.Clockwise_Positive); // positive is angle up
        var rightStatusCode = configureMotor(rightMotor, InvertedValue.CounterClockwise_Positive);
        nt.setLeftAngleConfigStatus(leftStatusCode);
        nt.setRightAngleConfigStatus(rightStatusCode);

        this.followLeftMotor = new Follower(
            this.leftMotor.getDeviceID(),
            true);

        BaseStatusSignal.setUpdateFrequencyForAll(
            50.0,
            motorAbsolute,
            motorPosition,
            motorVelocity,
            leftStatorCurrent,
            leftSupplyCurrent,
            leftTemperature,
            rightStatorCurrent,
            rightSupplyCurrent,
            rightTemperature);
        CTREUtil.optimizeBusUtilization(leftMotor);
        CTREUtil.optimizeBusUtilization(rightMotor);
    }

    @Override
    public void updateInputs(AngleIOInputs inputs) {
        inputs.atHome = !this.homeSensor.get(); // is Normally Closed contact, so invert it

        BaseStatusSignal.refreshAll(
            motorAbsolute,
            motorPosition,
            motorVelocity,
            leftStatorCurrent,
            leftSupplyCurrent,
            leftTemperature,
            rightStatorCurrent,
            rightSupplyCurrent,
            rightTemperature);
        inputs.motorAbsolutePosition = this.motorAbsolute.getValueAsDouble();
        inputs.motorPosition_revs = BaseStatusSignal.getLatencyCompensatedValue(motorPosition, motorVelocity);
        inputs.motorVelocity_rps = this.motorVelocity.getValueAsDouble();
        inputs.leftMotorStatorCurrent_A = this.leftStatorCurrent.getValueAsDouble();
        inputs.leftMotorSupplyCurrent_A = this.leftSupplyCurrent.getValueAsDouble();
        inputs.leftMotorTemperature_C = this.leftTemperature.getValueAsDouble();
        inputs.rightMotorStatorCurrent_A = this.rightStatorCurrent.getValueAsDouble();
        inputs.rightMotorSupplyCurrent_A = this.rightSupplyCurrent.getValueAsDouble();
        inputs.rightMotorTemperature_C = this.rightTemperature.getValueAsDouble();
    }

    private static StatusCode configureMotor(TalonFX talonFX, InvertedValue invertedValue) {
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
        slot0Configs.kP = 200.0; // unit is amps of current per rotations per second of error
        slot0Configs.kI = 2.0;
        slot0Configs.kD = 10.0;

        // these are the PID velocity control parameters for motion control mode (long moves)
        var slot1Configs = talonConfiguration.Slot1;
        slot1Configs.kV = 0.1;
        slot1Configs.kP = 60.0; // unit is amps of current per rotations per second of error
        slot1Configs.kI = 0.0;
        slot1Configs.kD = 5.0;

        var motionMagicConfigs = talonConfiguration.MotionMagic;
        motionMagicConfigs.MotionMagicAcceleration = ACCELERATION_RPS2;
        motionMagicConfigs.MotionMagicCruiseVelocity = FAST_VELOCITY_RPS;
        motionMagicConfigs.MotionMagicJerk = JERK_RPS3;

        var openLoopRamps = talonConfiguration.OpenLoopRamps;
        openLoopRamps.DutyCycleOpenLoopRampPeriod = 2;
        
        var closeLoopRamps = talonConfiguration.ClosedLoopRamps;
        closeLoopRamps.DutyCycleClosedLoopRampPeriod = 0;

        StatusCode statusCode = CTREUtil.applyConfiguration(talonFX, talonConfiguration);
        return statusCode;
    }

    @Override
    public boolean setSpeed(double speed) {
        var motionMagicConfigs = new MotionMagicConfigs();
        motionMagicConfigs.MotionMagicCruiseVelocity = speed;
        motionMagicConfigs.MotionMagicAcceleration = ACCELERATION_RPS2;
        motionMagicConfigs.MotionMagicJerk = JERK_RPS3;
        var statusCode = CTREUtil.applyConfiguration(leftMotor, motionMagicConfigs);
        ThunderSubsystem.printStatusIfBad(statusCode, "Angle Left (51) Set Speed: ");
        return statusCode.isOK();
    }

    @Override
    public void goToPositionMotionProfile(double target_revs, double feedforward_A) {
        var statusCode = CTREUtil.tryUntilOK(() ->
            leftMotor.setControl(
            positionMotionControl
                .withPosition(target_revs)
                .withFeedForward(feedforward_A)), 
            leftMotor.getDeviceID());
        ThunderSubsystem.printStatusIfBad(statusCode, "Left Angle Motor (51) - MotionMagic setControl: ");
    }

    @Override
    public void goToPositionPid(double target_revs, double feedforward_A) {
        var statusCode = CTREUtil.tryUntilOK(() ->
            this.leftMotor.setControl(
            positionPidControl
                .withPosition(target_revs)
                .withFeedForward(feedforward_A)), 
            leftMotor.getDeviceID());
        ThunderSubsystem.printStatusIfBad(statusCode, "Left Angle Motor (51) - Position PID setControl: ");
    }

    @Override
    public void stop() {
        this.leftMotor.set(0.0);
    }

    @Override
    public void follow() {
        var statusCode = CTREUtil.tryUntilOK(() ->
            rightMotor.setControl(this.followLeftMotor), 
            rightMotor.getDeviceID());
        ThunderSubsystem.printStatusIfBad(statusCode, "Right Angle Motor (52) - Follow setControl: ");
    }
}
