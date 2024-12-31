package frc.robot.utilities;

import com.ctre.phoenix6.StatusCode;

import edu.wpi.first.wpilibj2.command.*;

public abstract class ThunderSubsystem extends SubsystemBase {

    private boolean subsystemInitialized = false;
    
    public boolean IsInitialized() {
        return subsystemInitialized;
    }

    public void setSubsystemInitialized() {
        this.subsystemInitialized = true;
    }

    public static void printStatusIfBad(StatusCode statusCode, String label) {
        if(!statusCode.isOK()) {
            System.out.println(label + statusCode.getDescription());
        }
    }

    public abstract void readInputs();
    public abstract void recordOutputs();
}
