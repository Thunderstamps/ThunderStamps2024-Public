package frc.robot.subsystems.swerve.commands;

import edu.wpi.first.wpilibj2.command.*;
import frc.robot.Constants;
import frc.robot.subsystems.swerve.*;
import frc.robot.subsystems.swerve.util.SwerveUtil;
import frc.robot.subsystems.swerve.util.Vector2D;

public class MoveAbsoluteAndRotateCommand extends Command {
    
    private final static double TOLERANCE_RAD = Math.toRadians(15.0);
    private static final double DEFAULT_TOLERANCE_m = 0.35;
    private final double tolerance_m;
    private final SwerveSubsystem swerveSubsystem;
    private final double absoluteX_m;
    private final double absoluteY_m;
    private final double targetFieldOrientation_rad;
    private final double maxSpeed_in_s;
    private final double finalSpeed_in_s;
    private Vector2D distanceToGo_m;
    private boolean rotationComplete;
    
    public MoveAbsoluteAndRotateCommand(
        SwerveSubsystem swerveSubsystem,
        Vector2D absoluteTarget_m,
        double targetFieldOrientation_rad,
        double finalSpeed_in_s) {
            this(swerveSubsystem, absoluteTarget_m, targetFieldOrientation_rad, Constants.MAX_SPEED_IN_SEC, finalSpeed_in_s);

    }
    
    public MoveAbsoluteAndRotateCommand(
        SwerveSubsystem swerveSubsystem,
        Vector2D absoluteTarget_in,
        double targetFieldOrientation_rad,
        double maxSpeed_in_s,
        double finalSpeed_in_s) {
            this(swerveSubsystem, absoluteTarget_in, targetFieldOrientation_rad, maxSpeed_in_s, finalSpeed_in_s, DEFAULT_TOLERANCE_m);
    }
    
    public MoveAbsoluteAndRotateCommand(
        SwerveSubsystem swerveSubsystem,
        Vector2D absoluteTarget_in,
        double targetFieldOrientation_rad,
        double maxSpeed_in_s,
        double finalSpeed_in_s,
        double tolerance_m) {
        this.swerveSubsystem = swerveSubsystem;
        this.addRequirements(swerveSubsystem);
        this.absoluteX_m = absoluteTarget_in.getX();
        this.absoluteY_m = absoluteTarget_in.getY();
        this.targetFieldOrientation_rad = targetFieldOrientation_rad;
        this.maxSpeed_in_s = maxSpeed_in_s;
        this.finalSpeed_in_s = finalSpeed_in_s;
        this.tolerance_m = tolerance_m;
    }

    @Override
    public void initialize() {
        this.rotationComplete = false;
        this.updateDistanceToGo();
        this.swerveSubsystem.setTargetFieldOrientation_rad(this.targetFieldOrientation_rad);
        System.out.println("Started move/rotate to " + this.absoluteX_m + ", " + this.absoluteY_m);
    }

    @Override
    public void execute() {
        this.updateDistanceToGo();
        if(!this.rotationComplete) { // we "remember" it's complete in case we rotate past it a bit
            this.rotationComplete = this.isRotationFinished();
        }
        this.swerveSubsystem.setTargetFieldOrientation_rad(this.targetFieldOrientation_rad);
        var distanceToGo_in = this.distanceToGo_m.getMagnitude() * 39.3701;
        var speed_in_s = distanceToGo_in * 1.5;
        if(speed_in_s < this.finalSpeed_in_s && this.rotationComplete) {
            speed_in_s = this.finalSpeed_in_s; // don't slow down lower than this speed when we get close to the point
        }
        if(speed_in_s > this.maxSpeed_in_s) {
            speed_in_s = maxSpeed_in_s;
        }
        var fieldOrientedTranslationCommand_in_s = Vector2D.FromPolar(
            speed_in_s, 
            this.distanceToGo_m.getAngleRadians());
        this.swerveSubsystem.executeAutonomousControl(
            fieldOrientedTranslationCommand_in_s, 
            Vector2D.FromXY(0, 0),
            0.0);
    }

    @Override
    public boolean isFinished() {
        return this.distanceToGo_m.getMagnitude() <= tolerance_m
            && this.rotationComplete;
    }

    @Override
    public void end(boolean interrupted) {
        System.out.println("Ended move/rotate to " + this.absoluteX_m + ", " + this.absoluteY_m);
    }

    private boolean isRotationFinished() {
        var diff_rad = SwerveUtil.limitAngleFromNegativePItoPI_rad(
            this.targetFieldOrientation_rad - this.swerveSubsystem.getFieldOrientation_rad()
        );
        return Math.abs(diff_rad) < TOLERANCE_RAD;
    }

    private void updateDistanceToGo() {
        var position_m = this.swerveSubsystem.getPose_m().getTranslation();
        this.distanceToGo_m = Vector2D.FromXY(
            this.absoluteX_m - position_m.getX(),
            this.absoluteY_m - position_m.getY()
        );
        // this.counter++;
        // if(this.counter > 4) {
        //     this.counter = 0;
        //     System.out.println(this.distanceToGo);
        // }
        
    }

    //private int counter = 0;
}
