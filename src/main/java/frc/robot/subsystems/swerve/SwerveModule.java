package frc.robot.subsystems.swerve;

import edu.wpi.first.math.geometry.*;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.util.Units;
import frc.robot.subsystems.swerve.drive.IDriveController;
import frc.robot.subsystems.swerve.steer.ISteeringController;
import frc.robot.subsystems.swerve.util.IVector2D;
import frc.robot.subsystems.swerve.util.SwerveModuleConstants;
import frc.robot.subsystems.swerve.util.SwerveUtil;
import frc.robot.subsystems.swerve.util.Vector2D;

public class SwerveModule implements ISwerveModule {

    private final ISteeringController steeringController;
    private final IDriveController driveController;
    // Position of this swerve module relative to the center of mass.
    private final IVector2D modulePosition_in;
    // Location on the robot, such as "RightFront"
    private final String moduleName;
    // Orientation of this module relative to robot forward
    private final double moduleOrientation_rad;

    private final double maxSpeed_in_s;

    public SwerveModule(
            ISteeringController steeringController,
            IDriveController driveController,
            SwerveModuleConstants swerveModuleConstants) {
        this.steeringController = steeringController;
        this.driveController = driveController;
        this.modulePosition_in = swerveModuleConstants.ModulePosition_in;
        this.moduleName = swerveModuleConstants.ModuleName;
        this.moduleOrientation_rad = swerveModuleConstants.ModuleOrientation_rad;
        this.maxSpeed_in_s = calculateWheelSpeed_in_s(this.driveController.getMaxSpeedRpm());
    }

    @Override
    public void readInputs() {
        this.steeringController.readInputs(this.moduleName);
        this.driveController.readInputs(this.moduleName);
    }

    @Override
    public void recordOutputs() {
        this.steeringController.recordOutputs(this.moduleName);
        this.driveController.recordOutputs(this.moduleName);
    }

    @Override
    public ISteeringController getSteeringController() {
        return this.steeringController;
    }

    @Override
    public IDriveController getDriveController() {
        return this.driveController;
    }

    @Override
    public void executeVelocityMode(IVector2D velocityCommand_in_s_rad) {
        double steeringAngleDiff_rad = SwerveUtil.shortestDiffToTargetAngle_rad(
                this.getRobotOrientedSteeringAngle_rad(), velocityCommand_in_s_rad.getAngleRadians());
        double steeringAngleDiffMagnitude_rad = Math.abs(steeringAngleDiff_rad);
        double steeringAngleDiff_direction = steeringAngleDiff_rad >= 0.0 ? 1.0 : -1.0;

        double driveMotorDirection = 1.0;

        if (steeringAngleDiffMagnitude_rad > (Math.PI / 2.0)) {
            steeringAngleDiffMagnitude_rad = Math.PI - steeringAngleDiffMagnitude_rad;
            steeringAngleDiff_direction *= -1.0;
            driveMotorDirection = -1.0;
        }

        steeringAngleDiff_rad = steeringAngleDiffMagnitude_rad * steeringAngleDiff_direction;

        double steeringAngleDiff_revs = steeringAngleDiff_rad / (2.0 * Math.PI);
        double steeringMotorDiff_revs = steeringAngleDiff_revs * this.steeringController.getMotorRevsPerSteeringRev();
        double steeringCommand_revs = this.steeringController.getMotorRevCount() + steeringMotorDiff_revs;

        this.steeringController.setTargetMotorRev(steeringCommand_revs);

        double inchesPerWheelRev = this.driveController.getWheelDiameterInches() * Math.PI;

        if (inchesPerWheelRev <= 0.0) {
            inchesPerWheelRev = 4.0 * Math.PI;
        }

        double commandWheelSpeed_rpm = 60.0 * velocityCommand_in_s_rad.getMagnitude() / inchesPerWheelRev;

        double commandMotorSpeed_rpm = commandWheelSpeed_rpm * this.driveController.getMotorRevsPerWheelRev();

        // If the steering motor is in the wrong direction, use cosine to reduce the speed requested
        // of the drive motor to avoid trying to drive in an orthogonal direction to that desired
        var cosFactor = Math.cos(steeringAngleDiffMagnitude_rad);
        commandMotorSpeed_rpm *= cosFactor;

        this.driveController.executeVelocityMode(commandMotorSpeed_rpm * driveMotorDirection);

    }

    @Override
    public boolean isHomed() {
        return this.steeringController.isHomed();
    }

    @Override
    public IVector2D getModulePos_in() {
        return this.modulePosition_in;
    }

    @Override
    public double getModuleOrientation_rad() {
        return this.moduleOrientation_rad;
    }

    @Override
    public IVector2D getCurrentVelocity_in_s_rad() {

        double driveMotorRpm = this.driveController.getMotorRpm();

        double speed_in_s = calculateWheelSpeed_in_s(driveMotorRpm);

        double positiveSpeed_in_s = Math.abs(speed_in_s);

        double direction_rad = getRobotOrientedSteeringAngle_rad();

        if (speed_in_s < 0.0) {
            direction_rad += Math.PI;
            direction_rad = SwerveUtil.limitAngleFromNegativePItoPI_rad(direction_rad);
        }

        return Vector2D.FromPolar(positiveSpeed_in_s, direction_rad);
    }

    private double calculateWheelSpeed_in_s(double motorRpm) {
        double gearRatio = this.driveController.getMotorRevsPerWheelRev();
        if (gearRatio <= 0.0) {
            return 0;
        }

        double wheelRpm = motorRpm / gearRatio;
        double inchesPerMinute = wheelRpm * this.driveController.getWheelDiameterInches() * Math.PI;
        double inchesPerSecond = inchesPerMinute / 60.0;
        return inchesPerSecond;
    }

    private double getRobotOrientedSteeringAngle_rad() {
        double gearRatio = this.steeringController.getMotorRevsPerSteeringRev();
        if (gearRatio <= 0.0) {
            return 0;
        }
        double steeringMotorRotations = this.steeringController.getMotorRevCount();
        double steeringRotations = steeringMotorRotations / gearRatio;
        double steeringAngle_rad = steeringRotations * 2.0 * Math.PI;
        double robotOrientedSteeringAngle = steeringAngle_rad + this.moduleOrientation_rad;
        return SwerveUtil.limitAngleFromNegativePItoPI_rad(robotOrientedSteeringAngle);

    }

    @Override
    public double getMaxSpeed_in_s() {
        return this.maxSpeed_in_s;
    }

    public Translation2d getTranslation2d_m() {
        return new Translation2d(
            this.modulePosition_in.getX() * 0.0254, 
            this.modulePosition_in.getY() * 0.0254);
    }

    public SwerveModuleState getSwerveModuleState_m_s() {
        var velocity = this.getCurrentVelocity_in_s_rad();
        return new SwerveModuleState(
            velocity.getMagnitude() * 0.0254, // convert to meters/sec
            new Rotation2d(velocity.getAngleRadians())
        );
    }

    public SwerveModulePosition getSwerveModulePosition_m_s() {
        double inchesPerWheelRev = this.driveController.getWheelCircumferenceInches();
        double gearRatio = Math.max(this.driveController.getMotorRevsPerWheelRev(), 0);
        var distance_wheelRevs = this.driveController.getMotorRevCount() / gearRatio;
        var distance_in = distance_wheelRevs * inchesPerWheelRev;
        double direction_rad = getRobotOrientedSteeringAngle_rad();
        return new SwerveModulePosition(
            Units.inchesToMeters(distance_in), 
            new Rotation2d(direction_rad)
        );
    }

}
