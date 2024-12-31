package frc.robot.subsystems.swerve;

import java.util.ArrayList;

import org.littletonrobotics.junction.Logger;

import com.pathplanner.lib.auto.*;
import com.pathplanner.lib.util.*;

import edu.wpi.first.math.estimator.*;
import edu.wpi.first.math.geometry.*;
import edu.wpi.first.math.kinematics.*;
import edu.wpi.first.wpilibj.DriverStation;
import frc.controllers.IXboxController;
import frc.robot.*;
import frc.robot.communications.*;
import frc.robot.utilities.*;
import frc.robot.subsystems.swerve.drive.*;
import frc.robot.subsystems.swerve.gyros.*;
import frc.robot.subsystems.swerve.steer.*;
import frc.robot.subsystems.swerve.util.*;
import frc.robot.targeting.*;

public class SwerveSubsystem extends ThunderSubsystem {

    private static final double HALF_SNAP_ANGLE_deg = Constants.SNAP_ANGLE_deg / 2.0;

    private final IXboxController xbox;
    private final SteeringControllerTalonFXv6 steeringController1, steeringController2, steeringController3, steeringController4;
    private final DriveControllerTalonFXv6 driveController1, driveController2, driveController3, driveController4;
    private final SwerveModule swerveModule1, swerveModule2, swerveModule3, swerveModule4;
    private final RobotOrientedSwerve robotOrientedSwerve;
    private final FieldOrientedSwerve fieldOrientedSwerve;
    ArrayList<ISwerveModule> swerveModules = new ArrayList<ISwerveModule>();
    private final IGyro gyro;
    private final NetworkTableComms nt;
    private final ITargetingController targetingController;
    private double targetFieldOrientation_rad; 
    private final SwerveDriveKinematics swerveDriveKinematics;
    private final SwerveDrivePoseEstimator swerveDrivePoseEstimator;
    private Pose2d pose_m = new Pose2d();
    private boolean homed = false;
    private boolean fineHeading = false;
    private boolean gyroHasBeenZerod = false;
    private boolean driveTowardsWallDuringClimb = false;
    private boolean autoAim = false;
    private double assist_in_sec = 0.0; // positive is robot left
    
    public SwerveSubsystem(
        IXboxController xbox,
        IGyro gyro,
        NetworkTableComms nt,
        ITargetingController targetingController,
        DriveControllerIO driveControllerIO1,
        DriveControllerIO driveControllerIO2,
        DriveControllerIO driveControllerIO3,
        DriveControllerIO driveControllerIO4,
        SteeringControllerIO steeringControllerIO1,
        SteeringControllerIO steeringControllerIO2,
        SteeringControllerIO steeringControllerIO3,
        SteeringControllerIO steeringControllerIO4) {
        this.xbox = xbox;
        this.gyro = gyro;
        this.nt = nt;
        this.targetingController = targetingController;
        steeringController1 = new SteeringControllerTalonFXv6(steeringControllerIO1, Constants.SwerveModule1);
        steeringController2 = new SteeringControllerTalonFXv6(steeringControllerIO2, Constants.SwerveModule2);
        steeringController3 = new SteeringControllerTalonFXv6(steeringControllerIO3, Constants.SwerveModule3);
        steeringController4 = new SteeringControllerTalonFXv6(steeringControllerIO4, Constants.SwerveModule4);

        driveController1 = new DriveControllerTalonFXv6(driveControllerIO1);
        driveController2 = new DriveControllerTalonFXv6(driveControllerIO2);
        driveController3 = new DriveControllerTalonFXv6(driveControllerIO3);
        driveController4 = new DriveControllerTalonFXv6(driveControllerIO4);

        swerveModule1 = new SwerveModule(steeringController1, driveController1, Constants.SwerveModule1);
        swerveModule2 = new SwerveModule(steeringController2, driveController2, Constants.SwerveModule2);
        swerveModule3 = new SwerveModule(steeringController3, driveController3, Constants.SwerveModule3);
        swerveModule4 = new SwerveModule(steeringController4, driveController4, Constants.SwerveModule4);

        swerveModules.add(swerveModule1);
        swerveModules.add(swerveModule2);
        swerveModules.add(swerveModule3);
        swerveModules.add(swerveModule4);

        robotOrientedSwerve = new RobotOrientedSwerve(swerveModules);
        
        fieldOrientedSwerve = new FieldOrientedSwerve(
            robotOrientedSwerve, 
            gyro, 
            Constants.MAX_SPEED_IN_SEC,
            Constants.MAX_ACCELERATION_TELEOP_G * Constants.IN_SEC2_PER_G, 
            Constants.MAX_ROTATION_RATE_RAD_S, 
            Constants.SCAN_TIME_S, 
            Constants.ROTATION_P);
        
        swerveDriveKinematics = new SwerveDriveKinematics(
            swerveModule1.getTranslation2d_m(),
            swerveModule2.getTranslation2d_m(),
            swerveModule3.getTranslation2d_m(),
            swerveModule4.getTranslation2d_m()
        );
        swerveDrivePoseEstimator = new SwerveDrivePoseEstimator(
            swerveDriveKinematics,
            new Rotation2d(), // gyro angle
            this.gSwerveModulePositions_m_s(), 
            new Pose2d(0.0, 0.0, new Rotation2d()));

        // allow PathPlanner auto routines to control the swerve
        AutoBuilder.configureHolonomic(
            this::getPose_m, 
            this::ResetPose_m, 
            this::getSpeeds, 
            this::robotRelativeOutput, 
            this.getConfig(), 
            () -> false,
            this);
    }

    private ChassisSpeeds getSpeeds() {
        return this.swerveDriveKinematics
            .toChassisSpeeds(
                this.swerveModule1.getSwerveModuleState_m_s(),
                this.swerveModule2.getSwerveModuleState_m_s(),
                this.swerveModule3.getSwerveModuleState_m_s(),
                this.swerveModule4.getSwerveModuleState_m_s()
            );
    }

    private void robotRelativeOutput(ChassisSpeeds chassisSpeeds_m_s) {
        var translation_in_s = Vector2D.FromXY(
            chassisSpeeds_m_s.vxMetersPerSecond * 39.3701, 
            chassisSpeeds_m_s.vyMetersPerSecond * 39.3701);
        var rotation_rad_s = chassisSpeeds_m_s.omegaRadiansPerSecond;
        this.executeAutonomousControlRobotOriented(
            translation_in_s,
            rotation_rad_s
        );
    }

    public void ResetPose_m(Pose2d pose2d_m) {
        var gyroRotation = new Rotation2d(this.gyro.getFieldOrientation_rad());
        this.SetRobotFieldPosition_m_s(pose2d_m, gyroRotation);
    }

    public void UpdatePoseWithVision(Pose2d visionPose, Double timestamp){
        this.swerveDrivePoseEstimator.addVisionMeasurement(visionPose, timestamp);
    }
    private HolonomicPathFollowerConfig getConfig() {
        return new HolonomicPathFollowerConfig( // HolonomicPathFollowerConfig, this should likely live in your Constants class
                    new PIDConstants(5.0, 0.0, 0.0), // Translation PID constants
                    new PIDConstants(4.0, 0.0, 0.0), // Rotation PID constants
                    4.5, // Max module speed, in m/s
                    Constants.DRIVE_BASE_RADIUS_m, // Drive base radius in meters. Distance from robot center to furthest module.
                    new ReplanningConfig() // Default path replanning config. See the API for the options here
        );
    }

    @Override
    public void readInputs() {
        this.gyro.readInputs();
        for (ISwerveModule swerveModule : this.swerveModules) {
            swerveModule.readInputs();
        }

        if(!homed) {
            var allHomed = true;
            for(var swerveModule : this.swerveModules) {
                var steeringController = swerveModule.getSteeringController();
                steeringController.tryHoming();
                if(!steeringController.isHomed()) {
                    allHomed = false;
                }
            }

            if(allHomed) {
                this.homed = true;
            }
        }

        if(this.homed) {
            this.runSwerveOdometry();
        }
    }

    @Override
    public void recordOutputs() {
        if(this.homed) {
            var pose = this.getPose_m();
            Logger.recordOutput("SwerveSubsystem/Odometry", pose);
        }
        for (ISwerveModule swerveModule : this.swerveModules) {
            swerveModule.recordOutputs();
        }
        this.nt.setGyro(this.gyro.getFieldOrientation_rad());
        this.nt.setGyroEnabled(this.gyro.getEnabled());
    }

    private void execute(
            Vector2D fieldOrientedTranslationCommand_in_s_rad,
            Vector2D robotOrientedTranslationCommand_in_s_rad,
            double robotOrientedRotationCommand_s_rad) {

        var anyManualCommand = false;

        if (robotOrientedRotationCommand_s_rad !=0.0) {
            var diff_rad = robotOrientedRotationCommand_s_rad * Constants.SCAN_TIME_S;
            this.targetFieldOrientation_rad += diff_rad;
            anyManualCommand = true;
        }

        if (xbox.getLeftBumperPressed()) {
            var prevSnap_deg = nearestSnapAngle_deg(this.targetFieldOrientation_rad);
            var newHeading_rad = Math.toRadians(prevSnap_deg + Constants.SNAP_ANGLE_deg);
            this.targetFieldOrientation_rad = SwerveUtil.limitAngleFromNegativePItoPI_rad(newHeading_rad);
            anyManualCommand = true;
        }

        if (xbox.getRightBumperPressed()) {
            var prevSnap_deg = nearestSnapAngle_deg(this.targetFieldOrientation_rad);
            var newHeading_rad = Math.toRadians(prevSnap_deg - Constants.SNAP_ANGLE_deg);
            this.targetFieldOrientation_rad = SwerveUtil.limitAngleFromNegativePItoPI_rad(newHeading_rad);
            anyManualCommand = true;
        }

        var headingVector_rad = this.getxboxControllerRightStick();
        var headingVectorMagnitide = headingVector_rad.getMagnitude();
        if(headingVectorMagnitide > 0.5) {
            if(this.xbox.getXboxController().getRightStickButton()) {
                this.fineHeading = true; // if we click, remember we're in fine heading control 
            }

            if(this.fineHeading) {
                this.targetFieldOrientation_rad = headingVector_rad.getAngleRadians();
            }
            else {
                var snapHeading_deg = nearest90_deg(headingVector_rad.getAngleRadians());
                this.targetFieldOrientation_rad = Math.toRadians(snapHeading_deg);
            }
            anyManualCommand = true;
        }
        else {
            this.fineHeading = false;
        }

        // auto aim
        var heading_rad = this.gyro.getFieldOrientation_rad();
        var aiming = this.nt.getHasNoteInShooter() &&
            (DriverStation.isTeleopEnabled() || this.autoAim);
        var optionTheta_rad = this.targetingController.getTargetHeading_rad(heading_rad);
        var turnToTarget = aiming && optionTheta_rad.isPresent() && !anyManualCommand;
        if(turnToTarget) {
            var target_rad = optionTheta_rad.getAsDouble();
            var correctedTarget_rad = target_rad + Math.toRadians(Constants.SHOOTER_CORRECTION_DEG);
            this.targetFieldOrientation_rad = correctedTarget_rad;
        }
        
        // determine if we're on target
        var diffToTarget_rad = this.targetFieldOrientation_rad - heading_rad;
        var diffToTarget_deg = Math.toDegrees(diffToTarget_rad);
        var rotationRate_deg_s = Math.toDegrees(this.gyro.getRotationRate_rad_s());
        var holding = Math.abs(rotationRate_deg_s) < 4.0;
        var onTarget = holding && Math.abs(diffToTarget_deg) < 3.0;
        this.targetingController.SetTargetStatusSwerve(onTarget);

        // assist lining up to note
        var assistedRobotOrientedTranslationCommand_in_s_rad =
            Vector2D.FromXY(
                robotOrientedTranslationCommand_in_s_rad.getX(), 
                robotOrientedTranslationCommand_in_s_rad.getY() + this.assist_in_sec); 

        this.fieldOrientedSwerve.execute(
            fieldOrientedTranslationCommand_in_s_rad, 
            targetFieldOrientation_rad,
            assistedRobotOrientedTranslationCommand_in_s_rad, 
            robotOrientedRotationCommand_s_rad);
    }

    private double nearest90_deg(double angle_rad) {
        var limitedAngle_rad = SwerveUtil.limitAngleFromNegativePItoPI_rad(angle_rad);
        var angle_deg = Math.toDegrees(limitedAngle_rad);
        if(angle_deg > 180.0-45.0 || angle_deg < -180.0+45.0) {
            return 180.0;
        }
        if(angle_deg <= 45.0 && angle_deg >= -45.0) {
            return 0.0;
        }
        if(angle_deg > 0) {
            return 90.0;
        }
        return -90.0;
    }

    public double nearestSnapAngle_deg(double angle_rad) {
        var limitedAngle_rad = SwerveUtil.limitAngleFromNegativePItoPI_rad(angle_rad);
        var angle_deg = Math.toDegrees(limitedAngle_rad);

        if(angle_deg >= -HALF_SNAP_ANGLE_deg && angle_deg <= HALF_SNAP_ANGLE_deg) {
            return 0.0;
        }

        for(var a = Constants.SNAP_ANGLE_deg; a < 180.0; a += Constants.SNAP_ANGLE_deg) {
            if(angle_deg >= a-HALF_SNAP_ANGLE_deg && angle_deg <= a+HALF_SNAP_ANGLE_deg) {
                return a;
            }
        }

        for(var a = -Constants.SNAP_ANGLE_deg; a > -180.0; a -= Constants.SNAP_ANGLE_deg) {
            if(angle_deg >= a-HALF_SNAP_ANGLE_deg && angle_deg <= a+HALF_SNAP_ANGLE_deg) {
                return a;
            }
        }

        return 180.0;
    }
    
    private Vector2D getRobotOrientedTranslationCommand_in_s() {
        var robotOrientedTranslation = Vector2D.FromXY(0, 0);
        if(!this.gyro.getEnabled()) {
            robotOrientedTranslation = getxboxControllerLeftStick();
        }
        robotOrientedTranslation.squareMagnitude();
        double newMagnitude = robotOrientedTranslation.getMagnitude() * Constants.MAX_SPEED_IN_SEC;
        robotOrientedTranslation.setMagnitude(newMagnitude);
        if (robotOrientedTranslation.getMagnitude() < 0.5) {
            robotOrientedTranslation.setMagnitude(0.0);
        }
        return robotOrientedTranslation;
    }


    private Vector2D getFieldOrientedTranslationCommand_in_s() {
        Vector2D input;
        double maxSpeed_inSec;
        input = getxboxControllerLeftStick();
        maxSpeed_inSec = Constants.MAX_SPEED_IN_SEC;

        input.squareMagnitude();
        var newMagnitude = input.getMagnitude() * maxSpeed_inSec;
        Vector2D fieldOrientedTranslation = Vector2D.FromPolar(newMagnitude, input.getAngleRadians());
        if(fieldOrientedTranslation.getMagnitude() < 0.2) {
            fieldOrientedTranslation.setMagnitude(0.0);
        }

        return fieldOrientedTranslation;
    }
    private double getRobotOrientedRotationCommand_rad_s() {
        var leftTrigger = xbox.getLeftTriggerAxis();
        var rightTrigger = xbox.getRightTriggerAxis();
        var triggerMagnitude = (leftTrigger - rightTrigger);
        var triggerMagnitudeSquared = triggerMagnitude * Math.abs(triggerMagnitude);

        return triggerMagnitudeSquared * Constants.MAX_ROTATION_RATE_RAD_S * 0.20;
    }
    
    private Vector2D getxboxControllerRightStick() {
        var x = xbox.getRightX();
        var y = xbox.getRightY();
        if(Math.abs(x) < 0.05 && Math.abs(y) < 0.05) {
            x = 0.0;
            y = 0.0;
        }
        return Vector2D.FromXY(-y, -x);
    }

    private Vector2D getxboxControllerLeftStick() {
        var x = xbox.getLeftX();
        var y = xbox.getLeftY();
        if(Math.abs(x) < 0.05 && Math.abs(y) < 0.05) {
            x = 0.0;
            y = 0.0;
        }
        return Vector2D.FromXY(-y, -x);
    }

    public void executeOperatorControl() {
        if(this.gyro.getEnabled()) {
            this.runSwerveFieldOriented();
        }
        else {
            this.runSwerveRobotOriented();
        }
    }

    private void runSwerveFieldOriented() {
        var fieldOrientedTranslationCommand_in_s_rad = getFieldOrientedTranslationCommand_in_s();
        var robotOrientedTranslationCommand_in_s_rad = getRobotOrientedTranslationCommand_in_s();
        var robotOrientedRotationCommand_in_s_rad = getRobotOrientedRotationCommand_rad_s();
        this.execute(
            fieldOrientedTranslationCommand_in_s_rad,
            robotOrientedTranslationCommand_in_s_rad,
            robotOrientedRotationCommand_in_s_rad
        );
    }
    
    private void runSwerveRobotOriented() {
        var robotOrientedTranslationCommand_in_s = getRobotOrientedTranslationCommand_in_s();
        var robotOrientedRotationCommand_rad_s = getRobotOrientedRotationCommand_rad_s();
        robotOrientedSwerve.execute(
            robotOrientedTranslationCommand_in_s, 
            robotOrientedRotationCommand_rad_s);
    }

    // this is for robot swerve disabled
    public void runSwerveRobotOrientedStop() {
        this.gyro.setEnabled(false);
        robotOrientedSwerve.execute(
            Vector2D.FromPolar(0.0, 0.0), 
            0.0);
    }

    // this is for calibration
    public void runSwerveRobotOrientedForwardSlow() {
        this.gyro.setEnabled(false);
        robotOrientedSwerve.execute(
            Vector2D.FromPolar(30.0, 0.0), 
            0.0);
    }

    public void runSwerveRobotOrientedForwardSlowRaw() {
        this.gyro.setEnabled(false);
        for(var swerveModule : this.swerveModules) {
            var driveController = swerveModule.getDriveController();
            driveController.executeVelocityMode(200);
        }
    }

    public void printSteering23Position() {
        var pos = this.steeringController3.getMotorRevCount();
        System.out.println(pos);
    }

    private void runSwerveOdometry() {
      
        var gyroAngle = new Rotation2d(this.gyro.getFieldOrientation_rad());
    
        this.pose_m = 
          this.swerveDrivePoseEstimator.update(
            gyroAngle, 
            this.gSwerveModulePositions_m_s());
        this.nt.setPose(pose_m);
    }

    private SwerveModulePosition[] gSwerveModulePositions_m_s() {
        var numberOfSwerveModules = this.swerveModules.size();
        var result = new SwerveModulePosition[numberOfSwerveModules];
        for(var i = 0; i < numberOfSwerveModules; i++) {
            result[i] = this.swerveModules.get(i).getSwerveModulePosition_m_s();
        }
        return result;
    }


    public SwerveDriveKinematics getKinematics() {
        return this.swerveDriveKinematics;
    }

    public Pose2d getPose_m() {
        return this.pose_m;
    }
    
    public void zeroGyro() {
        gyro.resetOffsetToZero_rad();
        this.targetFieldOrientation_rad = 0.0;
        this.gyroHasBeenZerod = true;
    }
    
    public void zeroPitch() {
        gyro.zeroPitch();
    }

    public void enableGyro() {
        this.gyro.setEnabled(true);
    }

    public void disableGyro() {
        this.gyro.setEnabled(false);
    }

    public void executeAutonomousControl(
        Vector2D fieldOrientedTranslationCommand_in_s,
        Vector2D robotOrientedTranslationCommand_in_s,
        double robotOrientedRotationCommand_rad_s) {

        if(this.gyro.getEnabled()) {
            this.execute(
                fieldOrientedTranslationCommand_in_s, 
                robotOrientedTranslationCommand_in_s,
                robotOrientedRotationCommand_rad_s);
        }
        else {
            this.execute(Vector2D.FromXY(0, 0), Vector2D.FromXY(0, 0), 0.0);    
        }
    }
      
    public void executeAutonomousControlRobotOriented(
        Vector2D robotOrientedTranslationCommand_in_s,
        double robotOrientedRotationCommand_rad_s) {
        robotOrientedSwerve.execute(
            robotOrientedTranslationCommand_in_s, 
            robotOrientedRotationCommand_rad_s);
    }

    public void autonomousInit() {
        this.gyro.setEnabled(true);
        this.zeroGyro();
        this.fieldOrientedSwerve.setMaxAcceleration(Constants.MAX_ACCELERATION_AUTO_G * Constants.IN_SEC2_PER_G);
        this.swerveDrivePoseEstimator.resetPosition(
            new Rotation2d(), 
            this.gSwerveModulePositions_m_s(), 
            new Pose2d(0.0, 0.0, new Rotation2d()));
    }

    public void teleopInit() {
        this.fieldOrientedSwerve.setMaxAcceleration(Constants.MAX_ACCELERATION_TELEOP_G * Constants.IN_SEC2_PER_G);
        this.gyro.setEnabled(true);
        if(!this.gyroHasBeenZerod) { // if we already zero'd in auto, don't zero again, but if we start up and go into teleop, zero it
            this.zeroGyro();
        }
        this.targetFieldOrientation_rad = this.gyro.getFieldOrientation_rad(); // prevents turning when you power up
    }
    
    public double getFieldOrientation_rad() {
        return this.gyro.getFieldOrientation_rad();
    }

    public void setGyroTo(double desiredOrientation_rad) {
        var currentAngle_rad = this.gyro.getFieldOrientation_rad();
        var adjustment_rad = currentAngle_rad - desiredOrientation_rad;
        this.gyro.adjustOffset_rad(adjustment_rad);
        this.targetFieldOrientation_rad = desiredOrientation_rad;
    }

    public void SetRobotFieldPosition_m_s(Pose2d pose_m_s, Rotation2d gyroAngle) {
        this.swerveDrivePoseEstimator.resetPosition(
            gyroAngle,
            this.gSwerveModulePositions_m_s(),
            pose_m_s
        );
        this.runSwerveOdometry(); // update pose field
    }
    public void setTargetFieldOrientation_rad(double targetFieldOrientation_rad) {
        this.targetFieldOrientation_rad = targetFieldOrientation_rad;
    }

    public void setRotationP(double rotationP) {
        this.fieldOrientedSwerve.setRotationP(rotationP);
    }

    public void setMaxRotationRate(double newMaxRotationRate_rad_s) {
        this.fieldOrientedSwerve.setMaxRotationRate(newMaxRotationRate_rad_s);
    }

    public double GetDriveSupplyCurrent() {
        var result = 0;
        for(var swerveModule : this.swerveModules) {
            var driveController = swerveModule.getDriveController();
            result += driveController.getSupplyCurrent();
        }
        return result;
    }

    public boolean GetDriveTowardsWallDuringClimb() {
        return this.driveTowardsWallDuringClimb;
    }

    public void SetDriveTowardsWallDuringClimb(boolean value) {
        this.driveTowardsWallDuringClimb = value;
    }

    public void SetAutoAim(boolean value) {
        this.autoAim = value;
    }

    public void SetAssist(double value_in_s) {
        this.assist_in_sec = value_in_s;
    }
}
