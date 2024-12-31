package frc.robot.utilities;

import edu.wpi.first.wpilibj.Timer;

public class OnDelayTimer {

    private final long delay_ms;
    private boolean lastValue;
    private double risingEdgeTime_sec = Timer.getFPGATimestamp();
    private boolean output = false;

    public OnDelayTimer(long delay_ms) {
        this.delay_ms = delay_ms;
    }

    public boolean execute(boolean input) {

        if(!lastValue && input) {
            risingEdgeTime_sec = Timer.getFPGATimestamp();
        }
        lastValue = input;

        if(input) {
            var duration_sec = Timer.getFPGATimestamp() - risingEdgeTime_sec;
            output = duration_sec * 1000.0 >= this.delay_ms;
        }
        else {
            output = false;
        }
        return output;
    }

    public boolean getOutput() {
        return this.output;
    }
}