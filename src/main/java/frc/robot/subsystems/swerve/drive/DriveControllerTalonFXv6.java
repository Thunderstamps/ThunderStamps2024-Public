package frc.robot.subsystems.swerve.drive;

import org.littletonrobotics.junction.Logger;

import frc.robot.Constants;
import frc.robot.utilities.KrakenX60;

public class DriveControllerTalonFXv6 implements IDriveController {

    private final static String PREFIX = "SwerveSubsystem/DriveController/";
    private final static double MAX_MOTOR_RPM = 5800.0;
    public final static double MAX_STATOR_CURRENT_A = Constants.MAX_DRIVE_STATOR_CURRENT_A;
    public final static double MAX_INPUT_CURRENT_A = Constants.MAX_DRIVE_INPUT_CURRENT_A;

    private final static double WHEEL_DIAMETER_INCHES = Constants.WHEEL_DIAMETER_INCHES;
    private final static double WHEEL_CIRCUMFERENCE_INCHES = WHEEL_DIAMETER_INCHES * Math.PI;

    private double logTargetMotorRpm;

    private final DriveControllerIO io;
    private final DriveControllerIOInputsAutoLogged inputs = new DriveControllerIOInputsAutoLogged();

    public DriveControllerTalonFXv6(
            DriveControllerIO io) {
        this.io = io;
    }

    @Override
    public void readInputs(String moduleName) {
        io.updateInputs(inputs);
        Logger.processInputs(PREFIX + moduleName, inputs);
    }

    @Override
    public void recordOutputs(String moduleName) {
        Logger.recordOutput(PREFIX + moduleName + "/VelocityRPM", inputs.motorVelocity_rps * 60);
        Logger.recordOutput(PREFIX + moduleName + "/SetpointRPM", logTargetMotorRpm);
        var efficiency = KrakenX60.CalculateEfficiencyTrapezoidal(
            inputs.motorStatorCurrent_A, 
            inputs.motorSupplyCurrent_A, 
            inputs.motorVelocity_rps);
        Logger.recordOutput(PREFIX + moduleName + "/Efficiency", efficiency);
    }

    public void executeVelocityMode(double targetMotorRpmInput) {
        double targetMotorRPM = targetMotorRpmInput;

        // enforce sane limits
        if(targetMotorRPM > MAX_MOTOR_RPM) {
            targetMotorRPM = MAX_MOTOR_RPM;
        }
        if(targetMotorRPM < -MAX_MOTOR_RPM) {
            targetMotorRPM = -MAX_MOTOR_RPM;
        }

        this.logTargetMotorRpm = targetMotorRPM;

        var rotationsPerSecond = targetMotorRPM / 60.0;
        io.setVelocity(rotationsPerSecond);
    }

    public double getMotorRpm() {
        return 60.0 * inputs.motorVelocity_rps;
    }

    public void executePositionMode(double targetMotorRev) {
        // not implementing yet
    }

    public double getMotorRevCount() {
        return inputs.absolutePosition_revs;
    }

    public double getMotorRevsPerWheelRev() {
        return Constants.SWERVE_DRIVE_MOTOR_REVS_PER_WHEEL_REV;
    }

    public double getWheelDiameterInches() {
        return WHEEL_DIAMETER_INCHES; 
    }

    @Override
    public double getWheelCircumferenceInches() {
        return WHEEL_CIRCUMFERENCE_INCHES;
    }

    public double getMaxSpeedRpm() {
        return MAX_MOTOR_RPM;
    }

    public void tune_kF(double minusOneToOne) {
        // run the motor at x% output and see how fast it goes
        this.executePercent(minusOneToOne);
        var velocityRps = inputs.motorVelocity_rps;

        // calculate what kF should be
        if(Math.abs(minusOneToOne) > 0.05 && Math.abs(velocityRps) > 10) {
            double kF = (minusOneToOne) / velocityRps;
            System.out.printf("%.2f", minusOneToOne); System.out.print(", ");
            System.out.print(velocityRps); System.out.print(", ");
            System.out.printf("kF: %.6f", kF); System.out.print(", ");
            this.printInputCurrentAndRPM();
        }
    }

    public void printInputCurrentAndRPM() {
        System.out.printf("%.0f A, ", this.getSupplyCurrent());
        System.out.printf("%.0f actual RPM", getMotorRpm());
        System.out.println();
    }

    private void executePercent(double minusOneToOne) {
        io.setDutyCycle(minusOneToOne);
    }

    public double GetMotorStatorCurrent() {
        return inputs.motorStatorCurrent_A;
    }

    @Override
    public double getSupplyCurrent() {
        return inputs.motorSupplyCurrent_A;
    }

}