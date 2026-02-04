package frc.robot;

import edu.wpi.first.wpilibj.XboxController;

public final class OperatorConstants {
    // Port Numbers from RobotContainer.h
    public static final int kSwerveControllerPort = 0;
    public static final int kElevatorControllerPort = 1;

    // Input Constants
    public static final double kDeadband = 0.08; // Matched C++ value (was 0.1 in Java)

    // Axis Mappings
    public static final int kStrafeAxis = XboxController.Axis.kLeftX.value;
    public static final int kForwardAxis = XboxController.Axis.kLeftY.value;
    public static final int kRotationAxis = XboxController.Axis.kRightX.value;
    public static final int kFieldRelativeButton = XboxController.Button.kRightBumper.value;
}
