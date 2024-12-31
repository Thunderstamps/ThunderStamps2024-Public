package frc.robot.subsystems.elevator;

import org.littletonrobotics.junction.Logger;
import frc.robot.utilities.*;

public class ElevatorSubsystem extends ThunderSubsystem {

    public final static double MAX_STATOR_CURRENT_A = 100.0; // Kraken X60 is 0.019 Nm/A
    private final static double COUNTERBALANCE_CURRENT_A = 18.0; // stator current to overcome gravity
    private final static double TENSION_CURRENT_A = 6.0; // stator current to keep string tensioned at bottom
    public static final double MIN_POSITION_IN = 0.0;
    public static final double MAX_POSITION_IN = 16.0;
    private static final double UP_POSITION_IN = MAX_POSITION_IN;
    private static final double MOTOR_ROTATIONS_PER_INCH = 0.8875;
    private static final double INCHES_PER_MOTOR_ROTATION = 1.0 / MOTOR_ROTATIONS_PER_INCH;
    public final static double REVERSE_LIMIT_ROTATIONS = MIN_POSITION_IN * MOTOR_ROTATIONS_PER_INCH;
    public final static double FORWARD_LIMIT_ROTATIONS = MAX_POSITION_IN * MOTOR_ROTATIONS_PER_INCH; // hard stop was 14.2 full rot
    
    private double target_in = 0.0;
    private double target_revs = 0.0;

    private ElevatorIO io;
    private final ElevatorIOInputsAutoLogged inputs = new ElevatorIOInputsAutoLogged();
    
    public ElevatorSubsystem(
            ElevatorIO io) {
        this.io = io;
    }

    @Override
    public void readInputs() {
        io.updateInputs(inputs);
        Logger.processInputs("ElevatorSubsystem", inputs);
    }

    @Override
    public void recordOutputs() {
        Logger.recordOutput("ElevatorSubsystem/Target_in", this.target_in);
        Logger.recordOutput("ElevatorSubsystem/Target_revs", this.target_revs);
        Logger.recordOutput("ElevatorSubsystem/AtPositionUp", this.AtPositionUp());
        var efficiency = KrakenX60.CalculateEfficiencyFOC(
            inputs.motorStatorCurrent_A, 
            inputs.motorSupplyCurrent_A, 
            inputs.motorVelocity_rps);
        Logger.recordOutput("ElevatorSubsystem/Efficiency", efficiency);
    }

    public void SetDutyCycle(double dutyCycle) {
        io.setDutyCycle(dutyCycle);
    }

    public void GoToPositionUp() {
        this.SetPosition(UP_POSITION_IN);
    }

    public boolean AtPositionUp() {
        return this.getPosition_in() > UP_POSITION_IN - 1.0;
    }

    public void GoToPositionAmpScore() {
        this.SetPosition(7.5);
    }

    public void GoToPositionDown() {
        this.SetPosition(MIN_POSITION_IN);
    }

    public void SetPosition(double position_in) {

        this.target_in = limitPosition_in(position_in);
        
        this.target_revs = this.target_in * MOTOR_ROTATIONS_PER_INCH;

        var currentPosition_revs = this.getPosition_rot();
        var feedforward_A = COUNTERBALANCE_CURRENT_A;
        if(currentPosition_revs < 0.5 && position_in < 0.5) {
            feedforward_A = TENSION_CURRENT_A; // don't want to use too much current when holding at the bottom
        }

        var distanceToTarget_revs = Math.abs(this.target_revs - currentPosition_revs);
        if(distanceToTarget_revs > 0.5) { // about 1 degrees = 0.2 motor revs
            // Motion control makes nicer long moves if we're further from the target, but doesn't hold the final position well
            io.goToPositionMotionProfile(this.target_revs, feedforward_A);
        }
        else {
            // PID position control is better at holding a precise position, but is overly aggressive for long moves
            io.goToPositionPid(this.target_revs, feedforward_A);
        }
    }

    private static double limitPosition_in(double position_in) {
        var result = position_in;
        if(position_in < MIN_POSITION_IN) {
            result = MIN_POSITION_IN;
        }
        else if(position_in > MAX_POSITION_IN) {
            result = MAX_POSITION_IN;
        }
        return result;
    }

    public double getPosition_in() {
        return this.getPosition_rot() * INCHES_PER_MOTOR_ROTATION;
    }

    public double getPosition_rot() {
        return inputs.motorPosition_revs;
    }

    public double getMotorStatorCurrent() {
        return inputs.motorStatorCurrent_A;
    }

    public double getMotorSupplyCurrent() {
        return inputs.motorSupplyCurrent_A;
    }
}
