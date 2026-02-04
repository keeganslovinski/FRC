package frc.robot;

public final class AutoConstants {
    // Matched values from AutoConstants namespace in RobotContainer.h
    public static final double kMaxSpeed = 4.0; // meters per second (4_mps)
    public static final double kMaxAcceleration = 6.0; // meters per second squared (6_mps_sq)
    
    // C++ used std::numbers::pi * 1_rad_per_s
    public static final double kMaxAngularSpeed = Math.PI; // radians per second
    
    // C++ used std::numbers::pi * 2_rad_per_s_sq
    public static final double kMaxAngularAcceleration = Math.PI * 2.0; // radians per second squared

    // PID Constants from AutoConstants namespace
    public static final double kPXController = 0.5;
    public static final double kPYController = 0.5;
    public static final double kPThetaController = 0.5;
}
