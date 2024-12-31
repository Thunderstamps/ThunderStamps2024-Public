package frc.robot.subsystems.swerve.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.swerve.SwerveSubsystem;
import frc.robot.targeting.providers.photon.Vision3D;

public class UpdatePoseWithVisionCommand extends Command {
    
    private final SwerveSubsystem swerveSubsystem;
    private final Vision3D vision3D;
    private Double lastVisionTimestamp = 0.0;
    private boolean hasUpdated = false;
    private Double maxDistFromLastPose = 1.0;   // in meters

    public UpdatePoseWithVisionCommand(
            SwerveSubsystem swerveSubsystem, 
            Vision3D vision3d) {
        this.swerveSubsystem = swerveSubsystem;
        this.vision3D = vision3d;
        
    }

    @Override
    public void initialize() {
        this.hasUpdated = false;
        this.lastVisionTimestamp=0.0;
    }

    @Override
    public void execute() {
        var visionPose = this.vision3D.getMasterPose();
        var visionTimestamp = this.vision3D.getMasterPoseTimeStamp();
        if ((visionPose!=null)&&(visionTimestamp>this.lastVisionTimestamp)) {
            var pose_m = this.swerveSubsystem.getPose_m();
            Double dist_m = Math.sqrt(Math.pow(visionPose.getX()-pose_m.getX(),2) + 
                                            Math.pow(visionPose.getY()-pose_m.getY(),2));
            if (dist_m<maxDistFromLastPose){
                this.swerveSubsystem.UpdatePoseWithVision(visionPose, visionTimestamp);
                this.hasUpdated = true;
                this.lastVisionTimestamp=visionTimestamp;
            }
        }
    }

    @Override
    public boolean isFinished() {
        return this.hasUpdated;
    }
}