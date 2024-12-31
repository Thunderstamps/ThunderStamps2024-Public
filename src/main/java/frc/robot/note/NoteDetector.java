package frc.robot.note;

import java.util.OptionalDouble;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.wpilibj.DriverStation;
import frc.robot.communications.NetworkTableComms;
import frc.robot.utilities.ThunderSubsystem;

public class NoteDetector extends ThunderSubsystem {

    private final NetworkTableComms nt;
    private final NoteDetectorIO io;
    private final NoteDetectorIOInputsAutoLogged inputs = new NoteDetectorIOInputsAutoLogged();

    private boolean assistModeOn = true;

    public NoteDetector(NetworkTableComms nt, NoteDetectorIO io) {
        this.nt = nt;
        this.io = io;
    }

    @Override
    public void readInputs() {
        io.updateInputs(inputs);
        Logger.processInputs("NoteDetector", inputs);
    }

    @Override
    public void recordOutputs() {

        this.nt.setNoteTheta(this.GetTheta_rad());
        var assistMode = this.GetAssistModeOn();
        this.nt.setAssistMode(assistMode);

        var driverMode = DriverStation.isTeleopEnabled() && !assistMode;
        io.setDriverMode(driverMode);

        Logger.recordOutput("NoteDetector/AssistModeOn", this.GetAssistModeOn());
        Logger.recordOutput("NoteDetector/DriveMode", driverMode);
        
    }

    public OptionalDouble GetTheta_rad() {
        if(inputs.hasTargets) {
            var yaw_deg = inputs.yaw_deg;
            var theta_rad = -Math.toRadians(yaw_deg);
            return OptionalDouble.of(theta_rad);
        }
        return OptionalDouble.empty();
    }

    public boolean GetAssistModeOn() {
        return this.assistModeOn;
    }

    public void SetAssistMode(boolean value) {
        this.assistModeOn = value;
    }
}
