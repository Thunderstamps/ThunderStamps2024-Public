package frc.robot.communications;

import java.util.OptionalDouble;

import org.littletonrobotics.junction.networktables.LoggedDashboardNumber;

import com.ctre.phoenix6.StatusCode;
import com.revrobotics.REVLibError;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.networktables.*;
import edu.wpi.first.wpilibj.smartdashboard.*;
import edu.wpi.first.wpilibj2.command.*;
import frc.robot.note.commands.DisableAssistModeCommand;
import frc.robot.note.commands.EnableAssistModeCommand;
import frc.robot.subsystems.swerve.commands.*;
import frc.robot.targeting.mode.commands.*;

public class NetworkTableComms {

    private final NetworkTableInstance inst = NetworkTableInstance.getDefault();

    private final NetworkTable smartDashboard;
    private final LoggedDashboardNumber manualWallCLimbRPM;
    private final LoggedDashboardNumber manualTrampRPM;
    private final LoggedDashboardNumber manualAngleDeg;
    private final LoggedDashboardNumber manualShooterSpeedRpm;
    private final NetworkTableEntry shooterModeManual;
    private final NetworkTableEntry shooterModeSafePosition;
    private final NetworkTableEntry shooterModeLobPosition;
    private final NetworkTableEntry shooterModePassPosition;
    private final NetworkTableEntry shooterModeVision2D;
    private final NetworkTableEntry shooterModeVision3D;
    private final NetworkTableEntry hasNoteInShooter;
    private final NetworkTableEntry hasNoteInTramp;
    private final NetworkTableEntry indexerSpeed;
    private final NetworkTableEntry wallClimbSpeed;
    private final NetworkTableEntry angleMotor;
    private final NetworkTableEntry angleIsHomed; 
    private final NetworkTableEntry angleMotorAbsolute;
    private final NetworkTableEntry angleHomeSensor;
    private final NetworkTableEntry shooterSpeedRight;
    private final NetworkTableEntry shooterSpeedLeft;
    private final NetworkTableEntry shooterSpeedTarget;
    private final NetworkTableEntry gyroDeg;
    private final NetworkTableEntry gyroRad;
    private final NetworkTableEntry gyroPitch;
    private final NetworkTableEntry gyroEnabled;
    private final NetworkTableEntry climbTarget;
    private final NetworkTableEntry noteDetected;
    private final NetworkTableEntry noteTheta;
    private final NetworkTableEntry assistMode;
    private final NetworkTableEntry leftAngleConfigStatus;
    private final NetworkTableEntry rightAngleConfigStatus;
    private final NetworkTableEntry elevatorConfigStatus;
    private final NetworkTableEntry indexerConfigStatus;
    private final NetworkTableEntry shooterLeftConfigStatus;
    private final NetworkTableEntry shooterRightConfigStatus;
    private final NetworkTableEntry trampConfigStatus;
    private final NetworkTableEntry wallClimbConfigStatus;
    private final NetworkTableEntry intakeConfigStatus;

    private final NetworkTableEntry vision2dTargetValid;
    private final NetworkTableEntry vision2dThetaRad;
    private final NetworkTableEntry vision2dDistanceFeet;
    private final NetworkTableEntry vision2dElapsedSec;
    
    private final NetworkTableEntry onTargetSwerve;    
    private final NetworkTableEntry onTargetShooter;    
    private final NetworkTableEntry onTargetAngle;

    private final NetworkTable limelight;
    private final NetworkTableEntry limelight_tv;
    private final NetworkTableEntry limelight_tx;
    private final NetworkTableEntry limelight_ty;
    private final NetworkTableEntry limelight_ledMode;
    private final NetworkTableEntry limelight_priorityid;

    private final NetworkTable debug;
    private final StructPublisher<Pose2d> pose;


    public NetworkTableComms() {
        this.smartDashboard = inst.getTable("SmartDashboard");
        this.manualWallCLimbRPM = new LoggedDashboardNumber("Manual Wall Climb (RPM)", 2500);
        this.manualTrampRPM = new LoggedDashboardNumber("Manual Tramp (RPM)", 500);
        this.manualAngleDeg = new LoggedDashboardNumber("Manual Angle (deg)", 20);
        this.manualShooterSpeedRpm = new LoggedDashboardNumber("Manual Shooter Speed (RPM)", 2500);
        this.shooterModeManual = smartDashboard.getEntry(" Manual Selected");
        this.shooterModeSafePosition = smartDashboard.getEntry(" Safe Position Selected");
        this.shooterModeLobPosition = smartDashboard.getEntry(" Lob Position Selected");
        this.shooterModePassPosition = smartDashboard.getEntry(" Pass Position Selected");
        this.shooterModeVision2D = smartDashboard.getEntry(" Vision 2D Selected");
        this.shooterModeVision3D = smartDashboard.getEntry(" Vision 3D Selected");
        this.hasNoteInShooter = smartDashboard.getEntry(" Has NOTE in Shooter");
        this.setHasNoteInShooter(false);
        this.hasNoteInTramp = smartDashboard.getEntry(" Has NOTE in Tramp");
        this.setHasNoteInTramp(false);
        
        this.indexerSpeed = smartDashboard.getEntry("Indexer Speed (RPM)");
        this.wallClimbSpeed = smartDashboard.getEntry("Wall Climb Speed (RPM)");
        this.angleMotor = smartDashboard.getEntry("Angle Motor (deg)");
        this.angleIsHomed = smartDashboard.getEntry(" Angle is Homed");
        this.angleMotorAbsolute = smartDashboard.getEntry("Angle Motor (abs)");
        this.angleHomeSensor = smartDashboard.getEntry(" Angle Home Sensor");
        this.shooterSpeedRight = smartDashboard.getEntry("Shooter Speed Right (RPM)");
        this.shooterSpeedLeft = smartDashboard.getEntry("Shooter Speed Left (RPM)");
        this.shooterSpeedTarget = smartDashboard.getEntry("Shooter Speed Target (RPM)");
        this.gyroDeg = smartDashboard.getEntry("Gyro (deg)");
        this.gyroRad = smartDashboard.getEntry("Gyro (rad)");
        this.gyroPitch = smartDashboard.getEntry("Gyro Pitch (deg)");
        this.gyroEnabled = smartDashboard.getEntry(" Gyro Enabled");
        this.climbTarget = smartDashboard.getEntry("Climb Target (deg)");
        this.noteDetected = smartDashboard.getEntry(" Note Detected");
        this.noteTheta = smartDashboard.getEntry("Note Theta (rad)");
        this.assistMode = smartDashboard.getEntry(" Assist Mode");
        this.leftAngleConfigStatus = smartDashboard.getEntry("Left Angle Config Status");
        this.rightAngleConfigStatus = smartDashboard.getEntry("Right Angle Config Status");
        this.elevatorConfigStatus = smartDashboard.getEntry("Elevator Config Status");
        this.indexerConfigStatus = smartDashboard.getEntry("Indexer Config Status");
        this.shooterLeftConfigStatus = smartDashboard.getEntry("Shooter (left) Config Status");
        this.shooterRightConfigStatus = smartDashboard.getEntry("Shooter (right) Config Status");
        this.trampConfigStatus = smartDashboard.getEntry("Tramp Config Status");
        this.wallClimbConfigStatus = smartDashboard.getEntry("Wall Climb Config Status");
        this.intakeConfigStatus = smartDashboard.getEntry("Intake Config Status");
        
        this.vision2dTargetValid = smartDashboard.getEntry("Vision 2D Target Valid");
        this.vision2dThetaRad = smartDashboard.getEntry("Vision 2D Theta (rad)");
        this.vision2dDistanceFeet = smartDashboard.getEntry("Vision 2D Distance (ft)");
        this.vision2dElapsedSec = smartDashboard.getEntry("Vision 2D Elapsed (sec)");

        this.onTargetSwerve = smartDashboard.getEntry(" On Target - Swerve");
        this.onTargetShooter = smartDashboard.getEntry(" On Target - Shooter");
        this.onTargetAngle = smartDashboard.getEntry(" On Target - Angle");

        this.limelight = inst.getTable("limelight");
        this.limelight_tv = limelight.getEntry("tv");
        this.limelight_tx = limelight.getEntry("tx");
        this.limelight_ty = limelight.getEntry("ty");
        this.limelight_ledMode = limelight.getEntry("ledMode");
        this.setLimelightLEDsOff();
        this.limelight_priorityid = limelight.getEntry("priorityid");
        
        this.debug = inst.getTable("debug");
        this.pose = this.debug
            .getStructTopic("pose", Pose2d.struct).publish();
    }

    public void initializeSwerveCommands(
            GyroEnableCommand gyroEnableCommand,
            GyroDisableCommand gyroDisableCommand,
            EnableAssistModeCommand enableAssistModeCommand,
            DisableAssistModeCommand disableAssistModeCommand) {
        SmartDashboard.putData(gyroEnableCommand);
        SmartDashboard.putData(gyroDisableCommand);
        SmartDashboard.putData(enableAssistModeCommand);
        SmartDashboard.putData(disableAssistModeCommand);
    }

    public double getManualWallClimb_rpm(){
        return this.manualWallCLimbRPM.get();
    }

    public double getManualTramp_rpm(){
        return this.manualTrampRPM.get();
    }

    public double getManualAngle_deg(){
        return this.manualAngleDeg.get();
    }

    public void setManualAngle_deg(double angle_deg) {
        this.manualAngleDeg.set(angle_deg);;
    }

    public double getManualShooterSpeed_rpm(){
        return this.manualShooterSpeedRpm.get();
    }

    public void setManualShooterSpeed_rpm(double speed_rpm) {
        this.manualShooterSpeedRpm.set(speed_rpm);;
    }

    public void initializeAutoChooser(SendableChooser<Command> autoPathChooser) {
        SmartDashboard.putData(autoPathChooser);
    }

    public void setIndexerSpeed(double speed_rpm) {
        this.indexerSpeed.setDouble(speed_rpm);
    }

    public void setWallClimbSpeed(double speed_rpm) {
        this.wallClimbSpeed.setDouble(speed_rpm);
    }

    public void setAngleMotor(double angle_deg) {
        this.angleMotor.setDouble(angle_deg);
    }

    public void setAngleIsHomed(boolean isHomed) {
        this.angleIsHomed.setBoolean(isHomed);
    }

    public void setAngleMotorAbsolute(double rev) {
        this.angleMotorAbsolute.setDouble(rev);
    }

    public void setAngleHomeSensor(boolean value) {
        this.angleHomeSensor.setBoolean(value);
    }

    public void setShooterSpeedRight(double speed_rpm) {
        this.shooterSpeedRight.setDouble(speed_rpm);
    }

    public void setShooterSpeedLeft(double speed_rpm) {
        this.shooterSpeedLeft.setDouble(speed_rpm);
    }

    public void setShooterSpeedTarget(double speed_rpm) {
        this.shooterSpeedTarget.setDouble(speed_rpm);
    }

    public void setVision2dTargetValid(boolean targetValid) {
        this.vision2dTargetValid.setBoolean(targetValid);
    }

    public void setVision2dThetaRad(double theta_rad) {
        this.vision2dThetaRad.setDouble(theta_rad);
    }

    public void setVision2dDistanceFeet(double distance_ft) {
        this.vision2dDistanceFeet.setDouble(distance_ft);
    }

    public void setVision2dElapsedSec(double seconds) {
        this.vision2dElapsedSec.setDouble(seconds);
    }

    public boolean getLimelight_tv() {
        return this.limelight_tv.getDouble(0.0) > 0.5;
    }

    public double getLimelight_tx(double defaultValue) {
        if(this.getLimelight_tv()) {
            return this.limelight_tx.getDouble(defaultValue);
        }
        return defaultValue;
    }

    public double getLimelight_ty(double defaultValue) {
        if(this.getLimelight_tv()) {
            return this.limelight_ty.getDouble(defaultValue);
        }
        return defaultValue;
    }

    public boolean getHasNoteInShooter() {
        return this.hasNoteInShooter.getBoolean(false);
    }

    public void setHasNoteInShooter(boolean value) {
        this.hasNoteInShooter.setBoolean(value);
    }

    public void setHasNoteInTramp(boolean value) {
        this.hasNoteInTramp.setBoolean(value);
    }

    public void initializeShootingModeCommands(
            ShootingModeManualCommand manualCommand,
            ShootingModeSafePositionCommand safePositionCommand,
            ShootingModeLobPositionCommand lobPositionCommand,
            ShootingModePassPositionCommand passPositionCommand,
            ShootingModeVision2dCommand vision2dCommand,
            ShootingModeVision3dCommand vision3dCommand) {
        SmartDashboard.putData(manualCommand);
        SmartDashboard.putData(safePositionCommand);
        SmartDashboard.putData(lobPositionCommand);
        SmartDashboard.putData(passPositionCommand);
        SmartDashboard.putData(vision2dCommand);
        SmartDashboard.putData(vision3dCommand);
    }

    public void setShooterModeManual() {
        this.clearShooterMode();
        this.shooterModeManual.setBoolean(true);
    }

    public void setShooterModeSafePosition() {
        this.clearShooterMode();
        this.shooterModeSafePosition.setBoolean(true);
    }

    public void setShooterModeLobPosition() {
        this.clearShooterMode();
        this.shooterModeLobPosition.setBoolean(true);
    }

    public void setShooterModePassPosition() {
        this.clearShooterMode();
        this.shooterModePassPosition.setBoolean(true);
    }

    public void setShooterModeVision2D() {
        this.clearShooterMode();
        this.shooterModeVision2D.setBoolean(true);
    }

    public void setShooterModeVision3D() {
        this.clearShooterMode();
        this.shooterModeVision3D.setBoolean(true);
    }

    private void clearShooterMode() {
        this.shooterModeManual.setBoolean(false);
        this.shooterModeSafePosition.setBoolean(false);
        this.shooterModeLobPosition.setBoolean(false);
        this.shooterModePassPosition.setBoolean(false);
        this.shooterModeVision2D.setBoolean(false);
        this.shooterModeVision3D.setBoolean(false);
    }

    public void setLimelightLEDsOff() {
        this.limelight_ledMode.setDouble(1); // force LEDs off
    }

    public void setLimelightLEDsOn() {
        this.limelight_ledMode.setDouble(0); // means let the pipeline control this setting (can change brightness)
    }

    public void setLimelightTargetRed() {
        this.limelight_priorityid.setDouble(4.0);
    }

    public void setLimelightTargetBlue() {
        this.limelight_priorityid.setDouble(7.0);
    }

    public void setOnTargetSwerve(boolean value) {
        this.onTargetSwerve.setBoolean(value);
    }

    public void setOnTargetShooter(boolean value) {
        this.onTargetShooter.setBoolean(value);
    }

    public void setOnTargetAngle(boolean value) {
        this.onTargetAngle.setBoolean(value);
    }

    public void setPose(Pose2d pose2d) {
        this.pose.set(pose2d);
    }

    public void setGyro(double angle_rad) {
        this.gyroRad.setDouble(angle_rad);
        this.gyroDeg.setDouble(Math.toDegrees(angle_rad));
    }

    public void setGyroPitch(double pitch_deg) {
        this.gyroPitch.setDouble(pitch_deg);
    }

    public void setGyroEnabled(boolean enabled) {
        this.gyroEnabled.setBoolean(enabled);
    }

    public void setClimbTarget(double climbTarget_deg) {
        this.climbTarget.setDouble(climbTarget_deg);
    }

    public void setNoteTheta(OptionalDouble value_rad) {
        if(value_rad.isPresent()) {
            this.noteDetected.setBoolean(true);
            this.noteTheta.setDouble(value_rad.getAsDouble());
        }
        else {
            this.noteDetected.setBoolean(false);
        }
    }

    public void setAssistMode(boolean value) {
        this.assistMode.setBoolean(value);
    }

    public void setLeftAngleConfigStatus(StatusCode statusCode) {
        this.leftAngleConfigStatus.setString(statusCode.getDescription());
    }

    public void setRightAngleConfigStatus(StatusCode statusCode) {
        this.rightAngleConfigStatus.setString(statusCode.getDescription());
    }

    public void setElevatorConfigStatus(StatusCode statusCode) {
        this.elevatorConfigStatus.setString(statusCode.getDescription());
    }

    public void setIndexerConfigStatus(StatusCode statusCode) {
        this.indexerConfigStatus.setString(statusCode.getDescription());
    }

    public void setShooterConfigStatus(REVLibError left, REVLibError right) {
        this.shooterLeftConfigStatus.setString(left.name());
        this.shooterRightConfigStatus.setString(right.name());
    }

    public void setTrampConfigStatus(REVLibError err) {
        this.trampConfigStatus.setString(err.name());
    }

    public void setWallClimbConfigStatus(REVLibError err) {
        this.wallClimbConfigStatus.setString(err.name());
    }

    public void setIntakeConfigStatus(REVLibError err) {
        this.intakeConfigStatus.setString(err.name());
    }
}
