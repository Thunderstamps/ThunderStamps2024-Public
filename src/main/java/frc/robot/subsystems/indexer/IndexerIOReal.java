package frc.robot.subsystems.indexer;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.VelocityTorqueCurrentFOC;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.wpilibj.DigitalInput;
import frc.robot.Constants;
import frc.robot.communications.NetworkTableComms;
import frc.robot.utilities.CTREUtil;
import frc.robot.utilities.ThunderSubsystem;

public class IndexerIOReal implements IndexerIO {

    private final static double MAX_STATOR_CURRENT_A = IndexerSubsystem.MAX_STATOR_CURRENT_A;
    
    private final DigitalInput noteSensor = new DigitalInput(0);

    private final TalonFX leftMotor = new TalonFX(Constants.IndexerCanbus, Constants.CANBUS_NAME);
    
    private final StatusSignal<Double> motorPosition = leftMotor.getPosition();
    private final StatusSignal<Double> motorVelocity = leftMotor.getVelocity();
    private final StatusSignal<Double> statorCurrent = leftMotor.getStatorCurrent();
    private final StatusSignal<Double> supplyCurrent = leftMotor.getSupplyCurrent();
    private final StatusSignal<Double> temperature = leftMotor.getDeviceTemp();

    private final DutyCycleOut dutyCycleOut = new DutyCycleOut(0);
    private final VelocityTorqueCurrentFOC velocityControl = new VelocityTorqueCurrentFOC(0)
        .withSlot(0);

    public IndexerIOReal(NetworkTableComms nt) {
        // stop it
        this.leftMotor.setControl(this.dutyCycleOut.withOutput(0));

        var statusCode = configureMotor(this.leftMotor);
        nt.setIndexerConfigStatus(statusCode);

        BaseStatusSignal.setUpdateFrequencyForAll(
            50.0,
            motorPosition,
            motorVelocity,
            statorCurrent,
            supplyCurrent,
            temperature);
        CTREUtil.optimizeBusUtilization(leftMotor);
    }
    
    @Override
    public void updateInputs(IndexerIOInputs inputs) {
        inputs.sensorDetectsNote = !this.noteSensor.get();

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

    private StatusCode configureMotor(TalonFX motor) {
        
        var talonConfiguration = new TalonFXConfiguration();

        // neutral behavior
        var motorOutputConfigs = talonConfiguration.MotorOutput;
        motorOutputConfigs.NeutralMode = NeutralModeValue.Brake;
        motorOutputConfigs.Inverted = InvertedValue.CounterClockwise_Positive; 

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
        slot0Configs.kV = 0.09;
        slot0Configs.kP = 4.0; // unit is amps of current per rotations per second of error
        slot0Configs.kI = 0;
        slot0Configs.kD = 0;

        var openLoopRamps = talonConfiguration.OpenLoopRamps;
        openLoopRamps.DutyCycleOpenLoopRampPeriod = 2;
        
        var closeLoopRamps = talonConfiguration.ClosedLoopRamps;
        closeLoopRamps.DutyCycleClosedLoopRampPeriod = 0;

        var statusCode = CTREUtil.applyConfiguration(motor, talonConfiguration);
        return statusCode;
    }

    @Override
    public void setDutyCycle(double dutyCycle) {
        var statusCode = CTREUtil.tryUntilOK(() ->
            leftMotor.setControl(this.dutyCycleOut.withOutput(dutyCycle)), 
            leftMotor.getDeviceID());
        ThunderSubsystem.printStatusIfBad(statusCode, "Indexer (50) SetDutyCycle: ");
    }

    @Override
    public void zeroPosition() {
        var statusCode = CTREUtil.tryUntilOK(() ->
            leftMotor.setPosition(0.0), 
            leftMotor.getDeviceID());
        ThunderSubsystem.printStatusIfBad(statusCode, "Indexer (50) ZeroPosition: ");
    }

    @Override
    public void setVelocity(double rotations_per_sec) {
        var statusCode = CTREUtil.tryUntilOK(() ->
            leftMotor.setControl(velocityControl.withVelocity(rotations_per_sec)), 
            leftMotor.getDeviceID());
        ThunderSubsystem.printStatusIfBad(statusCode, "Indexer (50) Velocity: ");
    }
}
