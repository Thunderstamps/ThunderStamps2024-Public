package frc.robot.targeting.providers.limelight;

import java.util.OptionalDouble;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.*;
import frc.robot.communications.*;
import frc.robot.targeting.providers.ITargetProvider;

public class Vision2D extends SubsystemBase implements ITargetProvider {

    private static final double DEFAULT_DISTANCE_FT = 16.0;
    private static final double TIMEOUT_MS = 3000.0; 

    private final NetworkTableComms nt;
    private final LimelightDistanceLookup limelightDistanceLookup = new LimelightDistanceLookup();
    private double tx = 0.0;
    private OptionalDouble optionTheta_rad = OptionalDouble.empty();
    private double ty = 0.0;
    private double distance_ft = DEFAULT_DISTANCE_FT;
    private boolean targetValid = false;
    private double lastValidTargetTime_sec = Timer.getFPGATimestamp();

    private final Vision2DIO io;
    private final Vision2DIOInputsAutoLogged inputs = new Vision2DIOInputsAutoLogged();

    public Vision2D(
            NetworkTableComms nt,
            Vision2DIO io) {
        this.nt = nt;
        this.io = io;
    }

    @Override
    public void periodic() {
        io.updateInputs(inputs);
        Logger.processInputs("Vision2D", inputs);

        var now = Timer.getFPGATimestamp();

        // if the tag is valid, compute the direction and distance to target
        if(inputs.limelight_tv) {

            this.tx = inputs.limelight_tx;
            var theta_rad = -Math.toRadians(this.tx); // tx is positive right, but theta should be positive counter-clockwise
            this.optionTheta_rad = OptionalDouble.of(theta_rad);

            this.ty = inputs.limelight_ty;
            this.distance_ft = this.limelightDistanceLookup.GetDistance_ft(this.ty);

            this.lastValidTargetTime_sec = now;
            this.targetValid = true;
        }
        else {
            this.optionTheta_rad = OptionalDouble.empty();
        }
        
        // if we haven't seen the tag in a long time, set valid to false
        var elapsed_sec = now - this.lastValidTargetTime_sec;
        if(elapsed_sec * 1000.0 >= TIMEOUT_MS) {
            this.targetValid = false;
        }

        // put these values on the dashboard for monitoring and debugging
        this.nt.setVision2dTargetValid(this.targetValid);
        if(this.optionTheta_rad.isPresent()) {
            this.nt.setVision2dThetaRad(this.optionTheta_rad.getAsDouble());
        }
        else {
            this.nt.setVision2dThetaRad(0);
        }
        this.nt.setVision2dDistanceFeet(this.distance_ft);
        this.nt.setVision2dElapsedSec(elapsed_sec);

        var alliance = DriverStation.getAlliance();
        if(alliance.isPresent()) {
            if(alliance.get() == Alliance.Red) {
                this.nt.setLimelightTargetRed();
            }
            else if(alliance.get() == Alliance.Blue) {
                this.nt.setLimelightTargetBlue();
            }
        }
    }

    @Override
    public OptionalDouble getTheta_rad() {
        return this.optionTheta_rad;
    }

    @Override
    public OptionalDouble getDistance_ft() {
        if(this.targetValid) {
            return OptionalDouble.of(this.distance_ft);
        }
        return OptionalDouble.empty();
    }
    
}
