package frc.robot.utilities;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.RobotController;

public class KrakenX60 {

    // formula for kT is stall torque (in Nm) / (stall current - free current)
    private static final double KRAKEN_X60_FREE_CURRENT = 2.0;
    private static final double KRAKEN_X60_TRAPEZOIDAL_KT = 7.09 / (366.0 - KRAKEN_X60_FREE_CURRENT);
    private static final double KRAKEN_X60_FOC_KT = 9.37 / (483.0 - KRAKEN_X60_FREE_CURRENT);

    // Calculates instantaneous efficiency (mechanical output power / electrical input power)
    public static double CalculateEfficiencyTrapezoidal(
            double motorStatorCurrent_A,
            double motorSupplyCurrent_A,
            double motorVelocity_rps) {
        var torque_Nm = KrakenX60.calculateTorqueTrapezoidal_Nm(motorStatorCurrent_A);
        return calculateEfficiency(torque_Nm, motorSupplyCurrent_A, motorVelocity_rps);
    }

    // Calculates instantaneous efficiency (mechanical output power / electrical input power)
    public static double CalculateEfficiencyFOC(
            double motorStatorCurrent_A,
            double motorSupplyCurrent_A,
            double motorVelocity_rps) {
        var torque_Nm = KrakenX60.calculateTorqueFOC_Nm(motorStatorCurrent_A);
        return calculateEfficiency(torque_Nm, motorSupplyCurrent_A, motorVelocity_rps);
    }

    private static double calculateEfficiency(
            double torque_Nm,
            double motorSupplyCurrent_A,
            double motorVelocity_rps) {
        var speed_rad_per_sec = motorVelocity_rps * 2.0 * Math.PI;
        var outputPower_W = speed_rad_per_sec * torque_Nm;
        var inputPower_W = RobotController.getBatteryVoltage() * motorSupplyCurrent_A;
        var efficiency = 1.0;
        if(inputPower_W != 0) {
            efficiency = outputPower_W / inputPower_W;
            efficiency = MathUtil.clamp(efficiency, 0.0, 1.0);
        }
        return efficiency;
    }
    
    public static double calculateTorqueTrapezoidal_Nm(double statorCurrent_A) {
        var current_A = Math.max(statorCurrent_A - KRAKEN_X60_FREE_CURRENT, 0.0);
        return current_A * KRAKEN_X60_TRAPEZOIDAL_KT;
    }
    
    public static double calculateTorqueFOC_Nm(double statorCurrent_A) {
        var current_A = Math.max(statorCurrent_A - KRAKEN_X60_FREE_CURRENT, 0.0);
        return current_A * KRAKEN_X60_FOC_KT;
    }
}
