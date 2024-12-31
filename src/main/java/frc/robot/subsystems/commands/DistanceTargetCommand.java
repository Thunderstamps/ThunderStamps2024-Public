package frc.robot.subsystems.commands;

import edu.wpi.first.wpilibj2.command.*;
import frc.robot.subsystems.angle.*;
import frc.robot.subsystems.indexer.IndexerSubsystem;
import frc.robot.subsystems.shooter.*;
import frc.robot.targeting.*;

public class DistanceTargetCommand extends Command {
    
    private final AngleSubsystem angleSubsystem;
    private final ShooterSubsystem shooterSubsystem;
    private final double angle_deg;
    private double speed_rpm;
    private final double distance_ft;
    private final IndexerSubsystem indexerSubsystem;
    private final ITargetingController targetingController;

    public DistanceTargetCommand(
            AngleSubsystem angleSubsystem,
            ShooterSubsystem shooterSubsystem,
            IndexerSubsystem indexerSubsystem,
            ITargetingController targetingController,
            double distance_ft,
            double speed_rpm) {
        this(angleSubsystem, shooterSubsystem, indexerSubsystem, targetingController, distance_ft);
        this.speed_rpm = speed_rpm;
    }

    public DistanceTargetCommand(
            AngleSubsystem angleSubsystem,
            ShooterSubsystem shooterSubsystem,
            IndexerSubsystem indexerSubsystem,
            ITargetingController targetingController,
            double distance_ft) {
        this.angleSubsystem = angleSubsystem;
        this.shooterSubsystem = shooterSubsystem;
        this.indexerSubsystem = indexerSubsystem;
        this.targetingController = targetingController;
        this.distance_ft = distance_ft;
        addRequirements(angleSubsystem);
        addRequirements(shooterSubsystem);
        // doesn't depend on indexer subsystem
        var lookupTable = new ShooterLookupTable();
        this.angle_deg = lookupTable.GetAngle_deg(distance_ft);
        this.speed_rpm = lookupTable.GetSpeed_rpm(distance_ft);
    }
    
    @Override
    public void initialize() {
        this.angleSubsystem.SetSpeedFast();
        this.targetingController.SetTargetStatusAngle(false);
        this.targetingController.SetTargetStatusShooter(false);
        System.out.printf("Start DistanceTarget %.2f ft%n", this.distance_ft);
    }

    @Override
    public void execute() {
        if(this.indexerSubsystem.GetNotePresent()) {
            this.angleSubsystem.SetPosition(this.angle_deg);
            var diff_deg = Math.abs(this.angleSubsystem.GetAngleMotor_deg() - this.angle_deg);
            var onTarget = diff_deg < 0.25;
            this.targetingController.SetTargetStatusAngle(onTarget);
        }
        else {
            this.angleSubsystem.GoToIntakePosition();
            this.targetingController.SetTargetStatusAngle(false);
        }
        this.shooterSubsystem.RunShooter(this.speed_rpm);
    }

    @Override
    public boolean isFinished() {
        return false;
    }

    @Override
    public void end(boolean interrupted) {
        System.out.printf("End DistanceTarget %.2f ft%n", this.distance_ft);
    }
}
