package frc.robot.targeting.providers.limelight;

import frc.robot.communications.NetworkTableComms;

public class Vision2DIOReal implements Vision2DIO {
    
    private final NetworkTableComms nt;

    public Vision2DIOReal(NetworkTableComms nt) {
        this.nt = nt;
    }

    @Override
    public void updateInputs(Vision2DIOInputs inputs) {
        inputs.limelight_tv = nt.getLimelight_tv();
        inputs.limelight_tx = nt.getLimelight_tx(inputs.limelight_tx);
        inputs.limelight_ty = nt.getLimelight_ty(inputs.limelight_ty);
    }
}
