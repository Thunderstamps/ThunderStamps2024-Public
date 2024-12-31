package frc.robot.subsystems.swerve.util;

public class SwerveModuleConstants
{
    public final String ModuleName;
    public final IVector2D ModulePosition_in;
    public final double ModuleOrientation_rad;
    public final int DriveCanbus;
    public final int SteerCanbus;
    public final int EncoderCanbus;
    public final int HomeSensorForward; // in counts (0 to 4095)

    public SwerveModuleConstants(
        String moduleName,
        IVector2D modulePosition_in,
        double moduleOrientation_rad,
        int driveCanbus,
        int steerCanbus,
        int encoderCanbus,
        int homeSensorForward
    ) {
        this.ModuleName = moduleName;
        this.ModulePosition_in = modulePosition_in;
        this.ModuleOrientation_rad = moduleOrientation_rad;
        this.DriveCanbus = driveCanbus;
        this.SteerCanbus = steerCanbus;
        this.EncoderCanbus = encoderCanbus;
        this.HomeSensorForward = homeSensorForward;
    }
}
