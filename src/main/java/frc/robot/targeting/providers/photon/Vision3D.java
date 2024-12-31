package frc.robot.targeting.providers.photon;

import java.util.ArrayList;
import java.util.List;

import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;
import org.photonvision.targeting.PhotonTrackedTarget;


import edu.wpi.first.math.util.Units;
import edu.wpi.first.networktables.DoublePublisher;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StructPublisher;
import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import frc.robot.communications.NetworkTableComms;

public class Vision3D {
    
    
    private final PhotonCamera arduCam1 = new PhotonCamera("Arducam1");
    private final PhotonCamera arduCam2 = new PhotonCamera("Arducam2");
    private final double CAM_TGT_TIMEOUT = 1000;   // ms before an old target is deemed not good anymore
    private final double CAM_POSE_TIMEOUT = 300;   // ms before we deem a pose stale
    private final double CAM_MAX_DELAY = 80;  // ms max between 2 cameras
    private final double CAMERA_HEIGHT_METERS = 0.18; 
    private final double CAMERA_PITCH_RADIANS = Units.degreesToRadians(21); 
    private final double AMB_THRESHOLD = 0.3;
   
    private final double cam2angle = -20;
    private final double cam1angle = 20;
    private final Transform3d robotToCam1 = new Transform3d(new Translation3d(-0.20, 0.25, CAMERA_HEIGHT_METERS), new Rotation3d(0,-1*CAMERA_PITCH_RADIANS,Units.degreesToRadians(cam1angle))); 
    private final Transform3d robotToCam2 = new Transform3d(new Translation3d(-0.2,-0.25,CAMERA_HEIGHT_METERS), new Rotation3d(0,-1*CAMERA_PITCH_RADIANS,Units.degreesToRadians(cam2angle)));
    private Double ambiguity1 = 1.0;
    private Double ambiguity2 = 1.0;
    private Double targetArea1 = 0.0;
    private Double targetArea2 = 0.0;
    private Pose2d masterPose = new Pose2d();
    private Double masterPoseTime = 0.0;

    private final AprilTagFieldLayout aprilTagFieldLayout = AprilTagFields.k2024Crescendo.loadAprilTagLayoutField();
    private final PhotonPoseEstimator photonPoseEstimator1 = new PhotonPoseEstimator(aprilTagFieldLayout, PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR, arduCam1, robotToCam1);
    private final PhotonPoseEstimator photonPoseEstimator2 = new PhotonPoseEstimator(aprilTagFieldLayout, PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR, arduCam2, robotToCam2);
    private final Timer timeSinceLast3Dpose = new Timer();

    // used for publishing debug values:
    // needs to be updated to use global nt value 
    private final NetworkTableInstance inst = NetworkTableInstance.getDefault();
    private final NetworkTable photonData = inst.getTable("photonData");
    private DoublePublisher amb1 = photonData.getDoubleTopic("amb1").publish();
    private DoublePublisher amb2 = photonData.getDoubleTopic("amb2").publish();
    private DoublePublisher ta1 = photonData.getDoubleTopic("ta1").publish();
    private DoublePublisher ta2 = photonData.getDoubleTopic("ta2").publish();
    private DoublePublisher timeStamp = photonData.getDoubleTopic("timestamp").publish();
    private StructPublisher<Pose2d> masterPosition = photonData.getStructTopic("masterPosition", Pose2d.struct).publish();
    private StructPublisher<Pose2d> arducam1Position = photonData.getStructTopic("Arducam1Position", Pose2d.struct).publish();
    private StructPublisher<Pose2d> arducam2Position = photonData.getStructTopic("Arducam2Position", Pose2d.struct).publish();
  

    public class targetWithTimestamp {
        public PhotonTrackedTarget target;
        public double timeStamp_sec;
    
        public targetWithTimestamp(PhotonTrackedTarget target, double timeStamp_sec){
          this.target = target;
          this.timeStamp_sec = timeStamp_sec;
        }
      }

    private final List<targetWithTimestamp> currentTargetList = new ArrayList<targetWithTimestamp>();
    
    public Vision3D(NetworkTableComms nt) {
    }
    
    public void init() {
        timeSinceLast3Dpose.start();
    }

    public void Periodic() {
        // this.nt.setNoteTheta(this.GetTheta_rad());
        // setup other nt items for debug / etc.
        
        // get best 3D pose from all photonvision
        getVisionPose(photonPoseEstimator1,photonPoseEstimator2);

        // if pose exists publish to NT
        if (masterPose != null){
            masterPosition.set(masterPose);
            timeStamp.set(masterPoseTime);
            //this.swerveDrivePoseEstimator
            // var pose_m = this.swerveSubsystem.getPose_m();

        }
        // update the current target list from photonvision cameras:
        // could be updated to also find limelight targets:
        // enable this when we want 2D targeting
        // updatePhotonTargets();


    }

    public Pose2d getMasterPose() {
      return masterPose;
      // this may be null
    }
    public Double getMasterPoseTimeStamp() {
      return masterPoseTime;
    }
    ////-----------------------------------------
    public void updatePhotonTargets() {
    // this will update the list of valid targets at any time
    // and only include targets from the last CAM_TIMEOUT time
    // 
    // remove all old targets 
    var now = Timer.getFPGATimestamp();
    List<targetWithTimestamp> removeList = new ArrayList<targetWithTimestamp>();

    for (targetWithTimestamp targetResult : currentTargetList) {
      var elapsed_sec = now - targetResult.timeStamp_sec;
      if (elapsed_sec * 1000.0 >CAM_TGT_TIMEOUT){
        removeList.add(targetResult);
      }
    }    
    currentTargetList.removeAll(removeList);

    // find targets from camera 1:
    var result = arduCam1.getLatestResult();
    if (result.hasTargets()) {
        for (PhotonTrackedTarget target : result.getTargets()) {
          SmartDashboard.putNumber("atCam1ID", target.getFiducialId());
          // just need to modify the yaw of the target so that it represents the robot, not sure how else to do this
          targetListUpdate(new PhotonTrackedTarget(target.getYaw()+cam1angle,target.getPitch(),target.getArea(),target.getSkew(),
          target.getFiducialId(),target.getBestCameraToTarget(),target.getAlternateCameraToTarget(),target.getPoseAmbiguity(),
          target.getMinAreaRectCorners(),target.getDetectedCorners())
          ,now);
        }
    }
    // find targets from camera 2
    result = arduCam2.getLatestResult();
    if (result.hasTargets()) {
        for (PhotonTrackedTarget target : result.getTargets()) {
          SmartDashboard.putNumber("atCam2ID", target.getFiducialId());
          // just need to modify the yaw of the target, not sure how else to do this
          targetListUpdate(new PhotonTrackedTarget(target.getYaw()+cam2angle,target.getPitch(),target.getArea(),target.getSkew(),
          target.getFiducialId(),target.getBestCameraToTarget(),target.getAlternateCameraToTarget(),target.getPoseAmbiguity(),
          target.getMinAreaRectCorners(),target.getDetectedCorners())
          ,now);
        }
    }
  }

  ////-----------------------------------------
  public void targetListUpdate(PhotonTrackedTarget target, double timeStamp_sec){
    // takes a target and checks if it should replace existing target in list
    Boolean targetExists = false;
    Boolean targetReplace = false;
    targetWithTimestamp toBeRemoved = new targetWithTimestamp(target, timeStamp_sec);

    for (targetWithTimestamp targetResult : currentTargetList) {
      if (targetResult.target.getFiducialId()==target.getFiducialId()){
        // replace target if it is older and very likely from an old frame
        // or if the current target has a larger area
        targetExists=true;
        var duration_sec = timeStamp_sec - targetResult.timeStamp_sec;
        if ((duration_sec * 1000.0 > CAM_MAX_DELAY) ||
            (targetResult.target.getArea()<target.getArea()))
            {
              targetReplace=true;
              toBeRemoved = targetResult;
              break;
        }
      } 
    }
    if (targetReplace){
      // target needs to be replaced:
      currentTargetList.remove(toBeRemoved);
      currentTargetList.add(new targetWithTimestamp(target, timeStamp_sec));
    }
    // target needs to be added
    if (!targetExists) {
      currentTargetList.add(new targetWithTimestamp(target, timeStamp_sec));
    }
  }
    

  // no list version of Julian's code
  // should update this to return ambiguity too maybe
  public void getVisionPose(PhotonPoseEstimator poseEstimator1, PhotonPoseEstimator poseEstimator2){
    EstimatedRobotPose pose1=null;
    EstimatedRobotPose pose2=null; 
  
      // check if it's been too long since last pose:
      if (timeSinceLast3Dpose.get()>CAM_POSE_TIMEOUT/1000) {
        ambiguity1=1.0;
        ambiguity2=1.0;
        targetArea1 = 0.0;
        targetArea2 = 0.0;
        amb1.set(ambiguity1);  // debug
        amb2.set(ambiguity2);  // debug
        ta1.set(targetArea1);
        ta2.set(targetArea2);
        timeSinceLast3Dpose.reset();
        timeSinceLast3Dpose.start();
        masterPose=null;
      }
      // get latest poses from cameras
      var poseOption1 = poseEstimator1.update();
      if (poseOption1.isPresent()){
        pose1 = poseOption1.get();
      // find min ambiguity out of targets
          List<PhotonTrackedTarget> targets = pose1.targetsUsed;
          ambiguity1=1.0;
          for (PhotonTrackedTarget target : targets){
            if (target.getPoseAmbiguity() < ambiguity1){
              ambiguity1 = target.getPoseAmbiguity();
              targetArea1 = target.getArea();
            }
          }
          amb1.set(ambiguity1);
          ta1.set(targetArea1);
          arducam1Position.set(pose1.estimatedPose.toPose2d());
        }
      var poseOption2 = poseEstimator2.update();
      if (poseOption2.isPresent()){
        pose2 = poseOption2.get();
      // find min ambiguity out of targets
          List<PhotonTrackedTarget> targets = pose2.targetsUsed;
          ambiguity2=1.0;
          for (PhotonTrackedTarget target : targets){
            if (target.getPoseAmbiguity() < ambiguity2){
              ambiguity2 = target.getPoseAmbiguity();
              targetArea2 = target.getArea();
            }
          }
          amb2.set(ambiguity2);
          ta2.set(targetArea2);
          arducam2Position.set(pose2.estimatedPose.toPose2d());
        }
  
      // ambiguity of exactly 0 seems to be bad?
      if (poseOption2.isPresent()&&(ambiguity2<ambiguity1)&&(ambiguity2<AMB_THRESHOLD)&&(ambiguity2!=0.00)) {
        masterPose = pose2.estimatedPose.toPose2d();
        masterPoseTime = pose2.timestampSeconds;
        timeSinceLast3Dpose.reset();
        timeSinceLast3Dpose.start();
      }
      else if (poseOption1.isPresent()&&(ambiguity1<ambiguity2)&&(ambiguity1<AMB_THRESHOLD)&&(ambiguity1!=0.00)) {
        masterPose = pose1.estimatedPose.toPose2d();
        masterPoseTime = pose1.timestampSeconds;
        timeSinceLast3Dpose.reset();
        timeSinceLast3Dpose.start();
      }
      
  
    }
  
}
