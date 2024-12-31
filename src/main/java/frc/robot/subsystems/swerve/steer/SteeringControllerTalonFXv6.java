
package frc.robot.subsystems.swerve.steer;

import org.littletonrobotics.junction.Logger;

import frc.robot.subsystems.swerve.util.SwerveModuleConstants;
import frc.robot.utilities.KrakenX60;

public class SteeringControllerTalonFXv6 implements ISteeringController {

    private final static String PREFIX = "SwerveSubsystem/SteeringController/";
    public final static double MAX_STATOR_CURRENT_A = 30.0;
    public final static int CANCODER_COUNTS = 4096;

    private final int homeSensorForward_counts;

    private boolean homed;
    private double revsOffset;
    private double motorRevsWhenHomed;

    private double logTargetPosition_revs;

    private final SteeringControllerIO io;
    private final SteeringControllerIOInputsAutoLogged inputs = new SteeringControllerIOInputsAutoLogged();

    public SteeringControllerTalonFXv6(
            SteeringControllerIO io,
            SwerveModuleConstants swerveModuleConstants){
        this.io = io;
        this.homeSensorForward_counts = swerveModuleConstants.HomeSensorForward;
        this.homed = false;
    }

    @Override
    public void readInputs(String moduleName) {
        io.updateInputs(inputs);
        Logger.processInputs(PREFIX + moduleName, inputs);
    }

    @Override
    public void recordOutputs(String moduleName) {
        Logger.recordOutput(PREFIX + moduleName + "/Target_revs", this.logTargetPosition_revs);
        var efficiency = KrakenX60.CalculateEfficiencyFOC(
            inputs.motorStatorCurrent_A, 
            inputs.motorSupplyCurrent_A, 
            inputs.motorVelocity_rps);
        Logger.recordOutput(PREFIX + moduleName + "/Efficiency", efficiency);
    }

    @Override
    public boolean isHomed() {
        return homed;
    }

    @Override
    public double getMotorRevCount() {
        return this.getMotorRevs() - this.motorRevsWhenHomed + this.revsOffset;
    }

    // Raw motor revs
    public double getMotorRevs() {
        return inputs.motorPosition_revs;
    }

    @Override
    public void setTargetMotorRev(double targetMotorRev) {
        if (!homed) {
            this.tryHoming();
        }
        else {
            var positionRevs = targetMotorRev + this.motorRevsWhenHomed - this.revsOffset;
            io.goToPositionPid(positionRevs);
            this.logTargetPosition_revs = positionRevs;
        }
    }

    public void tryHoming() {
        if (!homed) {
            referenceMotorPosition();
        }
    }

    @Override
    public double getMotorRevsPerSteeringRev() {
        return 1.0; // Using Fused CANcoder
    }

    // This is needed to determine the value of the homeSensorForward constructor parameter
    public int getAbsoluteCANCoderPosition() {
        return inputs.homeSensorPosition_counts;
    }

    private void referenceMotorPosition() {
        io.setDutyCycle(0);

        this.motorRevsWhenHomed = this.getMotorRevs();

        var cancoderCounts = this.getAbsoluteCANCoderPosition();
        double diffCounts = cancoderCounts - this.homeSensorForward_counts;
        var halfCounts = CANCODER_COUNTS / 2;
        while(diffCounts > halfCounts) {
            diffCounts -= CANCODER_COUNTS;
        }
        while(diffCounts <= -halfCounts) {
            diffCounts += CANCODER_COUNTS;
        }
        var diffSteeringRevs = diffCounts / CANCODER_COUNTS;
        var diffMotorRevs = diffSteeringRevs * getMotorRevsPerSteeringRev();
        this.revsOffset = diffMotorRevs;

        this.homed = true;
    }
}