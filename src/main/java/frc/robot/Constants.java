package frc.robot;

import frc.robot.subsystems.swerve.util.*;

public final class Constants {
    
    public static final double SCAN_TIME_S = 0.020; // default is 20 milliseconds, or 0.020 seconds

    public static final String CANBUS_NAME = "Default Name"; // CANivore name
    
    public static final double SHOOTER_CORRECTION_DEG = -5.0;  // shoots a bit to the left, so compensate for it
    
    // Wheelbase dimensions (measured from center of swerve wheel)
    private static final double WIDTH_in = 20.5;
    private static final double LENGTH_in = 18.5;
    private static final double OFFSET_X_in = 6.0;  // frame extends 8" out front of wheelbase, so nominal is 4"
    public static final double DRIVE_BASE_RADIUS_m = Vector2D.FromXY(LENGTH_in/2.0 - OFFSET_X_in, WIDTH_in/2.0).getMagnitude() * 0.0254;

    public static final double SWERVE_DRIVE_MOTOR_REVS_PER_WHEEL_REV = 5.14;
    public static final double SWERVE_MOTOR_REVS_PER_STEERING_REV = 12.8;
    
    // measured wheel at 3.75... measured with custom-made treads at 3.98, decided 3.93 4.8% more than 3.75 and roughly the same as auto overshoot
    public static final double WHEEL_DIAMETER_INCHES = 3.93;
    
    public static final double MAX_DRIVE_STATOR_CURRENT_A = 90.0; // prevents wheel slip while accelerating
    public static final double MAX_DRIVE_INPUT_CURRENT_A = 80.0; // prevents drawing too much power from battery

    public static final double MAX_ACCELERATION_TELEOP_G = 0.60; // keeps it from tipping over and helps prevent wheel slip during decel
    public static final double MAX_ACCELERATION_LINEAR_G = 0.35; // true acceleration (going faster)
    public static final double MAX_ACCELERATION_AUTO_G = 0.35; // be less aggressive during auto to really prevent wheel slip, overrides teleop_g above
    public static final double IN_SEC2_PER_G = 386.0;

    public static final double ROTATION_P = 4.0;
    public static final double SNAP_ANGLE_deg = 30.0;
    public static final double MAX_ROTATION_RATE_RAD_S = Math.toRadians(300); // was 300 in 2022
    public static final double MAX_SPEED_IN_SEC = 160.0; // L1: 125.0, COMP: 167 in 2022, L4 is 200 in 2023 - you can't really reach 200

    public static final SwerveModuleConstants SwerveModule1 = new SwerveModuleConstants(
        "RightFront", 
        Vector2D.FromXY(LENGTH_in/2.0 - OFFSET_X_in, -WIDTH_in/2.0),
        0.0,
        11, 
        21, 
        31,
        1604);

    public static final SwerveModuleConstants SwerveModule2 = new SwerveModuleConstants(
        "RightRear", 
        Vector2D.FromXY(-LENGTH_in/2.0 - OFFSET_X_in, -WIDTH_in/2.0),
        0.0,
        12, 
        22, 
        32,
        437);

    public static final SwerveModuleConstants SwerveModule3 = new SwerveModuleConstants(
        "LeftRear", 
        Vector2D.FromXY(-LENGTH_in/2.0 - OFFSET_X_in, WIDTH_in/2.0),
        0.0,
        13, 
        23, 
        33,
        55);

    public static final SwerveModuleConstants SwerveModule4 = new SwerveModuleConstants(
        "LeftFront", 
        Vector2D.FromXY(LENGTH_in/2.0 - OFFSET_X_in, WIDTH_in/2.0),
        0.0,
        14, 
        24, 
        34,
        746);

    public static final int Pigeon2Canbus = 40;

    public static final int ShooterLeftCanbus = 41;
    public static final int ShooterRightCanbus = 42;

    public static final int IndexerCanbus = 50;

    public static final int AngleLeftCanbus = 51;
    public static final int AngleRightCanbus = 52;

    public static final int IntakeCanbus = 55;

    public static final int TrampCanbus = 59;

    public static final int ElevatorCanbus = 60;

    public static final int WallClimbCanbus = 61;
}
