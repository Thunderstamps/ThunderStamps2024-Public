package frc.robot.subsystems.indexer;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.wpilibj.Timer;
import frc.robot.communications.*;
import frc.robot.utilities.*;

public class IndexerSubsystem extends ThunderSubsystem {
    
    private final static double MAX_MOTOR_RPM = 6000.0;
    public final static double MAX_STATOR_CURRENT_A = 80.0;

    private final NetworkTableComms nt;

    private boolean notePresent = false;
    private IndexerMode mode = IndexerMode.Default;
    
    private double indexerSpeed_rpm;
    private double indexerPosition_revs;
    
    private double shootStartTime_sec;
    private boolean shootComplete;
    private double shootCompleteTime_sec;

    private double setpointSpeed_rpm;

    private final IndexerIO io;
    private final IndexerIOInputsAutoLogged inputs = new IndexerIOInputsAutoLogged();

    public IndexerSubsystem(
            NetworkTableComms nt,
            IndexerIO io) {
        this.nt = nt;
        this.io = io;
    }

    @Override
    public void readInputs() {
        io.updateInputs(inputs);
        Logger.processInputs("IndexerSubsystem", inputs);
    }

    @Override
    public void periodic() {

        this.indexerSpeed_rpm = this.getMotorRpm();
        this.indexerPosition_revs = this.getMotorRevs();
        this.nt.setIndexerSpeed(this.indexerSpeed_rpm);
        
        if(this.IsInitialized()) {
            switch (this.mode) {
                case Climb:
                    this.Stop();
                    break;

                case Transfer:
                    this.SetVelocity(-1000);
                    break;

                case Stop:
                    this.SetVelocity(0);
                    break;

                case Shoot:
                    var duration_sec = Timer.getFPGATimestamp() - this.shootStartTime_sec;
                    if(!this.shootComplete && duration_sec >= 0.080 && this.GetPositionRotations() >= 2.5) {
                        this.shootComplete = true;
                        this.shootCompleteTime_sec = Timer.getFPGATimestamp();
                    }
            
                    if(!this.shootComplete) {
                        this.SetVelocity(1000);
                    }
                    else {
                        this.Stop();
                        this.SetNotePresent(false);
                    }

                    if(this.shootModeDone()) {
                        this.SetModeDefault();
                    }
                    break;
            
                default: // Default mode
                    if(!this.GetNotePresent() && this.SensorDetectsNote()) {
                        this.SetNotePresent(true);
                    }
                    if(this.GetNotePresent()) {
                        this.Stop();
                    }
                    else {
                        this.SetVelocity(1700);
                    }
                    break;
            }
        }
        else {
            this.SetDutyCycle(0.0);
        }
    }

    @Override
    public void recordOutputs() {
        this.nt.setHasNoteInShooter(this.GetNotePresent());
        Logger.recordOutput("IndexerSubsystem/HasNoteInShooter", this.GetNotePresent());
        Logger.recordOutput("IndexerSubsystem/SetpointRPM", this.setpointSpeed_rpm);
        var efficiency = KrakenX60.CalculateEfficiencyFOC(
            inputs.motorStatorCurrent_A, 
            inputs.motorSupplyCurrent_A, 
            inputs.motorVelocity_rps);
        Logger.recordOutput("IndexerSubsystem/Efficiency", efficiency);
    }

    private boolean shootModeDone() {
        var now = Timer.getFPGATimestamp();
        if(this.shootComplete) {
            var sinceShootComplete_sec = now - this.shootCompleteTime_sec;
            if(sinceShootComplete_sec >= 0.025) {
                return true;
            }
        }
        var duration_sec = now - shootStartTime_sec;
        return duration_sec >= 3.0;
    }

    public IndexerMode GetMode() {
        return this.mode;
    }

    public void SetModeDefault() {
        this.mode = IndexerMode.Default;
        System.out.println("Indexer in Default Mode");
    }

    public void SetModeStop() {
        this.mode = IndexerMode.Stop;
        System.out.println("Indexer in Stop Mode");
    }

    public void SetModeShoot() {
        this.shootStartTime_sec = Timer.getFPGATimestamp();
        this.shootComplete = false;
        this.ZeroPosition();
        this.mode = IndexerMode.Shoot;
        System.out.println("Indexer in Shoot Mode");
    }

    public void SetModeTransfer() {
        this.mode = IndexerMode.Transfer;
        System.out.println("Indexer in Transfer Mode");
    }

    public void SetModeClimb() {
        this.mode = IndexerMode.Climb;
        System.out.println("Indexer in Climb Mode");
    }

    public boolean IsStopped() {
        return Math.abs(this.indexerSpeed_rpm) < 25.0;
    }

    public void Stop() {
        this.SetVelocity(0);
    }

    public void SetDutyCycle(double dutyCycle) {
        this.setpointSpeed_rpm = 0;
        io.setDutyCycle(dutyCycle);
    }

    public void SetVelocity(double targetMotorRpmInput) {
        double targetMotorRPM = targetMotorRpmInput;

        // enforce sane limits
        if(targetMotorRPM > MAX_MOTOR_RPM) {
            targetMotorRPM = MAX_MOTOR_RPM;
        }
        if(targetMotorRPM < -MAX_MOTOR_RPM) {
            targetMotorRPM = -MAX_MOTOR_RPM;
        }

        this.setpointSpeed_rpm = targetMotorRPM;

        var rotations_per_sec = targetMotorRPM / 60.0;
        io.setVelocity(rotations_per_sec);
    }

    public void ZeroPosition() {
        io.zeroPosition();
        this.indexerPosition_revs = 0.0;
    }

    public double GetPositionRotations() {
        return this.indexerPosition_revs;
    }

    public boolean GetNotePresent() {
        return this.notePresent;
    }

    public void SetNotePresent(boolean value) {
        this.notePresent = value;
    }

    public boolean SensorDetectsNote() {
        return inputs.sensorDetectsNote;
    }

    public void PrintVelocityAndCurrent() {
        System.out.printf("%.0f RPM, %.2f A, Has Gamepiece: %b%n", 
            this.indexerSpeed_rpm,
            this.getMotorStatorCurrent(),
            this.GetNotePresent());
    }

    private double getMotorRpm() {
        return 60.0 * inputs.motorVelocity_rps;
    }

    private double getMotorRevs() {
        return inputs.motorPosition_revs;
    }

    private double getMotorStatorCurrent() {
        return inputs.motorStatorCurrent_A;
    }

    public double getMotorSupplyCurrent() {
        return inputs.motorSupplyCurrent_A;
    }
}
