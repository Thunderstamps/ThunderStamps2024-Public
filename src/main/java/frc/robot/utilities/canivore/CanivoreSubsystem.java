package frc.robot.utilities.canivore;

import org.littletonrobotics.junction.Logger;

import frc.robot.utilities.*;

public class CanivoreSubsystem extends ThunderSubsystem {

    private final CanivoreIO io;
    private final CanivoreIOInputsAutoLogged inputs = new CanivoreIOInputsAutoLogged();
    
    public CanivoreSubsystem(CanivoreIO io) {
        this.io = io;
    }

    @Override
    public void readInputs() {
        io.updateInputs(inputs);
        Logger.processInputs("CANivore", inputs);
    }

    @Override
    public void recordOutputs() {

    }
    
}
