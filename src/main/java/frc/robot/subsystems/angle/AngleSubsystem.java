package frc.robot.subsystems.angle;

import org.littletonrobotics.junction.Logger;
import frc.robot.communications.*;
import frc.robot.utilities.*;
import frc.robot.subsystems.swerve.gyros.IGyro;

public class AngleSubsystem extends ThunderSubsystem {
    
    public final static double MAX_STATOR_CURRENT_A = 300.0; // needs to be 250+ for start of climb
    private final static double COUNTERBALANCE_CURRENT_A = 7.0; // per motor stator current to overcome gravity at zero deg
    public final static double MAX_ANGLE_DEG = 74.8;
    public final static double MIN_ANGLE_DEG = 13.5;
    //private final static double GEAR_RATIO = 67.5; // 3:1 belt ratio and 225:10 spur to large gear ratio = 67.5
    private final static double FIRST_STAGE_GEAR_RATIO = 74.0/24.0;
    private final static double SECOND_STAGE_GEAR_RATIO = 68.0/22.0;
    private final static double SPUR_TO_LARGE_GEAR_RATIO = 225.0/10.0;
    private final static double GEAR_RATIO = FIRST_STAGE_GEAR_RATIO * SECOND_STAGE_GEAR_RATIO * SPUR_TO_LARGE_GEAR_RATIO; 
    private final static double CIRCLE_DEG = 360.0;

    private final static double TRANSFER_LOW_POS_DEG = 15.0;
    private final static double TRANSFER_HIGH_POS_DEG = 43.0;

    // put it all the way to the top, let it relax, read motor absolute position
    private static final double TOP_ROTOR_REVS = 0.374;
    private static final double TOP_ANGLE_DEG = 74.2; // originally 74.7 based on Cancoder, changed to 74.2 at Mississauga after motor change

    public static final double FAST_VELOCITY_RPS = 200.0;
    public static final double CLIMB_VELOCITY_RPS = 19.5;

    public static final double ACCELERATION_RPS2 = 100.0;
    public static final double JERK_RPS3 = 30000.0;

    private final NetworkTableComms nt;
    private final IGyro gyro;

    private boolean homed = false;
    private double motorHomeOffset_revs = 0;

    private double target_deg = 45.0;
    private double target_revs = 0;

    private static final double CLIMB_TARGET_DEG = MIN_ANGLE_DEG + 0.1;
    private double climbAngle_deg = MAX_ANGLE_DEG;

    private double lastSpeed = 0;
    private double pitch_deg = 0;

    private final AngleIO io;
    private final AngleIOInputsAutoLogged inputs = new AngleIOInputsAutoLogged();

    public AngleSubsystem(
            NetworkTableComms nt,
            IGyro gyro,
            AngleIO io) {
        this.nt = nt;
        this.gyro = gyro;
        this.io = io;
        
        this.SetSpeedFast();

    }

    @Override
    public void readInputs() {
        io.updateInputs(inputs);
        Logger.processInputs("AngleSubsystem", inputs);
    }

    @Override
    public void periodic() {
        if(!this.IsHomed()) {
            this.tryToHome();
        }
    }

    @Override
    public void recordOutputs() {
        this.pitch_deg = this.gyro.getPitch_deg();
        this.nt.setAngleMotor(this.GetAngleMotor_deg());
        this.nt.setAngleIsHomed(this.IsHomed());
        this.nt.setAngleMotorAbsolute(this.getMotorAbsolute());
        this.nt.setAngleHomeSensor(this.GetHomeSensorAtHome());
        this.nt.setGyroPitch(this.pitch_deg);
        Logger.recordOutput("AngleSubsystem/Target_deg", this.target_deg);
        Logger.recordOutput("AngleSubsystem/Target_revs", this.target_revs);
        Logger.recordOutput("AngleSubsystem/Speed_rps", this.lastSpeed);
        var leftEfficiency = KrakenX60.CalculateEfficiencyFOC(
            inputs.leftMotorStatorCurrent_A, 
            inputs.leftMotorSupplyCurrent_A, 
            inputs.motorVelocity_rps);
        Logger.recordOutput("AngleSubsystem/LeftEfficiency", leftEfficiency);
        var rightEfficiency = KrakenX60.CalculateEfficiencyFOC(
            inputs.leftMotorStatorCurrent_A, 
            inputs.leftMotorSupplyCurrent_A, 
            inputs.motorVelocity_rps);
        Logger.recordOutput("AngleSubsystem/RightEfficiency", rightEfficiency);
    }

    public void SetSpeedFast() {
        this.setSpeed(FAST_VELOCITY_RPS);
    }

    public void SetSpeedSlow() {
        this.setSpeed(CLIMB_VELOCITY_RPS);
    }

    private void setSpeed(double speed) {
        if(!isEqual(speed, this.lastSpeed)) {
            var isOK = this.io.setSpeed(speed);
            if(isOK) {
                this.lastSpeed = speed;
            }
        }
    }

    private static boolean isEqual(double val1, double val2) {
        return Math.abs(val2 - val1) < 0.01;
    }

    private boolean SlowSpeedIsSet() {
        return this.lastSpeed < (FAST_VELOCITY_RPS + CLIMB_VELOCITY_RPS) / 2.0;
    }

    public boolean IsHomed() {
        return this.homed;
    }

    private void tryToHome() {
        if(this.GetHomeSensorAtHome()) {
            var rotorPosition_revs = this.getMotorAbsolute();
            var diff_revs = rotorPosition_revs - TOP_ROTOR_REVS; // how far from the top position
            var motorFromZero_revs = diff_revs + (GEAR_RATIO * TOP_ANGLE_DEG / CIRCLE_DEG);
            var motorPosition_revs = inputs.motorPosition_revs;
            this.motorHomeOffset_revs = motorFromZero_revs - motorPosition_revs;
            this.target_deg = TOP_ANGLE_DEG;
            this.homed = true;
        }
    }

    private static double limitPosition_deg(double position_deg) {
        var result = position_deg;
        if(position_deg < MIN_ANGLE_DEG) {
            result = MIN_ANGLE_DEG;
        }
        else if(position_deg > MAX_ANGLE_DEG) {
            result = MAX_ANGLE_DEG;
        }
        return result;
    }

    public void GoToPrepareToClimbPosition() {
        this.SetPosition(MAX_ANGLE_DEG);
    }

    public boolean AtReadyToClimbPosition() {
        return this.GetAngleMotor_deg() > MAX_ANGLE_DEG - 3.0;
    }

    public void GoToClimbPosition() {
        this.SetSpeedSlow();
        if(this.SlowSpeedIsSet() && this.pitch_deg < 20.0) {
            this.SetPosition(CLIMB_TARGET_DEG);
            this.nt.setClimbTarget(CLIMB_TARGET_DEG);
            this.climbAngle_deg = Math.max(
                this.GetAngleMotor_deg() - 2.0, 
                CLIMB_TARGET_DEG);
        }
        else {
            this.SetPosition(this.climbAngle_deg); // stop climbing if we're tipped too far
            this.nt.setClimbTarget(this.climbAngle_deg);
        }
    }

    public boolean AtPositionClimb() {
        return this.GetAngleMotor_deg() < MIN_ANGLE_DEG + 1.6;
    }

    public void GoToIntakePosition() {
        this.SetPosition(30.0);
    }

    public void GoToLowTransferPosition() {
        this.SetPosition(TRANSFER_LOW_POS_DEG);
    }

    public boolean AtLowTransferPosition() {
        var pos_deg = this.GetAngleMotor_deg();
        return pos_deg >= TRANSFER_LOW_POS_DEG - 2.0 && pos_deg <= TRANSFER_LOW_POS_DEG + 2.0;
    }

    public void GoToHighTransferPosition() {
        this.SetPosition(TRANSFER_HIGH_POS_DEG);
    }

    public boolean AtHighTransferPosition() {
        var pos_deg = this.GetAngleMotor_deg();
        return pos_deg >= TRANSFER_HIGH_POS_DEG - 2.0 && pos_deg <= TRANSFER_HIGH_POS_DEG + 2.0;
    }

    public void SetPosition(double position_deg) {

        if(this.IsHomed()) {
            this.target_deg = limitPosition_deg(position_deg);
        
            this.target_revs = this.target_deg * GEAR_RATIO / CIRCLE_DEG - this.motorHomeOffset_revs;

            var currentPosition_rad = this.GetAngleMotor_rad();
            var feedforward_A = COUNTERBALANCE_CURRENT_A * Math.cos(currentPosition_rad); // current to counteract gravity

            var currentPosition_revs = this.GetMotorPositionRaw_revs();
            var distanceToTarget_revs = Math.abs(this.target_revs - currentPosition_revs);
            if(distanceToTarget_revs > 0.5) { // about 1 degrees = 0.2 motor revs
                // Motion control makes nicer long moves if we're further from the target, but doesn't hold the final position well
                io.goToPositionMotionProfile(this.target_revs, feedforward_A);
            }
            else {
                // PID position control is better at holding a precise angle, but is overly aggressive for long moves
                io.goToPositionPid(this.target_revs, feedforward_A);
            }
        }
        else {
            io.stop(); // brake if we're not homed
        }

        io.follow();
    }

    // This returns the angle in radians from the imaginary zero degrees (horizontal) using the reference motor encoder
    public double GetAngleMotor_rad() {
        return Math.toRadians(this.GetAngleMotor_deg());
    }

    // This returns the angle in degrees from the imaginary zero degrees (horizontal) using the reference motor encoder
    public double GetAngleMotor_deg() {
        var motorPositionWithOffset_revs = GetMotorPositionWithOffset_revs();
        return motorPositionWithOffset_revs * CIRCLE_DEG / GEAR_RATIO;
    }

    // This returns the number of motor revolutions from the imaginary zero degrees (horizontal)
    public double GetMotorPositionWithOffset_revs() {
        var motorPositionWithOffset_revs = 
            this.GetMotorPositionRaw_revs()
                + this.motorHomeOffset_revs;
        return motorPositionWithOffset_revs;
    }

    // This returns the number of motor revolutions from the zero position where it started up
    public double GetMotorPositionRaw_revs() {
        return inputs.motorPosition_revs;
    }

    public double getMotorSupplyCurrent() {
        return inputs.leftMotorSupplyCurrent_A
            + inputs.rightMotorSupplyCurrent_A;
    }

    public double getMotorAbsolute() {
        var result = inputs.motorAbsolutePosition;
        while (result < 0) {
            result += 1.0;
        }
        while (result >= 1.0) {
            result -= 1.0;
        }
        return result;
    }

    public boolean GetHomeSensorAtHome() {
        return inputs.atHome;
    }
}
