package frc.robot.note;

import org.photonvision.PhotonCamera;

public class NoteDetectorIOReal implements NoteDetectorIO {
    
    private final PhotonCamera noteCamera = new PhotonCamera("Notecam");

    @Override
    public void updateInputs(NoteDetectorIOInputs inputs) {
        var result = this.noteCamera.getLatestResult();
        inputs.hasTargets = result.hasTargets();
        if(inputs.hasTargets) {
            inputs.yaw_deg = result.getBestTarget().getYaw();
        }
    }

    @Override
    public void setDriverMode(boolean driverMode) {
        this.noteCamera.setDriverMode(driverMode);
    }
}
