package frc.robot.subsystems.swerve.commands;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.note.NoteDetector;
import frc.robot.subsystems.swerve.SwerveSubsystem;

public class TrackNoteInAutoCommand extends Command {
    
    private static final double THRESHOLD_RAD = 0.0;
    private final SwerveSubsystem swerveSubsystem;
    private final NoteDetector noteDetector;

    // field positions of Notes:
    // see https://docs.google.com/spreadsheets/d/1D-0xvXrRpQGKSP0lv0YfV0f6iMKR2s9Lta9ZazbYFI0/edit#gid=1339505864
    // look at tab "field map"
    // 0,0 is blue source corner all measures in m
    // N1,N2,N3 are on the blue side need to be translated for red or make new notes (if we use them)
    // field dim: x 16.58 m, y 8.2 m
    // perhaps this can all go in a separate file??

    //private static final Pose2d note3Pos = new Pose2d(Units.inchesToMeters(114.0),4.1,new Rotation2d(0));
    //private static final Pose2d note2Pos = new Pose2d(Units.inchesToMeters(114.0),4.1+Units.inchesToMeters(57.0),new Rotation2d(0));
    //private static final Pose2d note1Pos = new Pose2d(Units.inchesToMeters(114.0),4.1+Units.inchesToMeters(114.0),new Rotation2d(0));
    private static final Pose2d note8Pos = new Pose2d(8.29,Units.inchesToMeters(29.64),new Rotation2d(0));
    private static final Pose2d note7Pos = new Pose2d(8.29,Units.inchesToMeters(25.64+66),new Rotation2d(0));
    private static final Pose2d note6Pos = new Pose2d(8.29,Units.inchesToMeters(25.64+2*66),new Rotation2d(0));
    private static final Pose2d note5Pos = new Pose2d(8.29,Units.inchesToMeters(25.64+3*66),new Rotation2d(0));
    private static final Pose2d note4Pos = new Pose2d(8.29,Units.inchesToMeters(25.64+4*66),new Rotation2d(0));

    private final Pose2d notePos;
    private boolean hasUpdated = false;

    public TrackNoteInAutoCommand(
            SwerveSubsystem swerveSubsystem, 
            NoteDetector noteDetector,
            NoteNumber noteNumber) {
        this.swerveSubsystem = swerveSubsystem;
        this.noteDetector = noteDetector;
        // does not require either subsystem

        switch (noteNumber) {
            case N4:
                this.notePos = note4Pos;
                break;
            case N5:
                this.notePos = note5Pos;
                break;
            case N6:
                this.notePos = note6Pos;
                break;
            case N7:
                this.notePos = note7Pos;
                break;
            default:
                this.notePos = note8Pos;
                break;
        }
    }

    @Override
    public void initialize() {
        this.hasUpdated = false;
    }

    @Override
    public void execute() {
        var optionTheta = this.noteDetector.GetTheta_rad();

        if(optionTheta.isPresent()) {

            var theta_rad = optionTheta.getAsDouble();
            
            if(Math.abs(theta_rad) > THRESHOLD_RAD) {
                // make correction
                var pose_m = this.swerveSubsystem.getPose_m();
                var pose_yaw = pose_m.getRotation().getRadians();
                // distance from odometry pose to note:
                // if dist return from notecam is deemed accurate, this 
                // could be updated with that
                Double distOdom = Math.sqrt(Math.pow(notePos.getX()-pose_m.getX(),2) + 
                                            Math.pow(notePos.getY()-pose_m.getY(),2));
                Double xo = distOdom*Math.cos(theta_rad+pose_yaw);
                Double yo = distOdom*Math.sin(theta_rad+pose_yaw);

                var newPose_m = new Pose2d(
                    new Translation2d(notePos.getX()-xo, notePos.getY()-yo), 
                    pose_m.getRotation());
                this.swerveSubsystem.ResetPose_m(newPose_m);
                this.hasUpdated = true;
            }
        }
    }

    @Override
    public boolean isFinished() {
        return this.hasUpdated;
    }
}