package frc.robot;

import com.studica.frc.AHRS;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.ClosedLoopConfig.FeedbackSensor;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.wpilibj.XboxController;

public class RobotContainer {
    // NavX gyro (Studica)
    private final AHRS navx;

    // Controllers
    private final XboxController swerveController = new XboxController(OperatorConstants.kSwerveControllerPort);
    private final XboxController elevatorController = new XboxController(OperatorConstants.kElevatorControllerPort);

    // Swerve modules
    // IDs and Offsets matched to RobotContainer.h:
    // SwerveModule(int driveID, int steerID, int encoderID, double offset)
    
    // C++: m_frontLeft{5, 1, 9, 3.14}
    private final SwerveModule frontLeft = new SwerveModule(5, 1, 9, 3.14);
    
    // C++: m_frontRight{6, 2, 10, 0.9}
    private final SwerveModule frontRight = new SwerveModule(6, 2, 10, 0.9);
    
    // C++: m_backLeft{8, 4, 12, 0.7}
    private final SwerveModule backLeft = new SwerveModule(8, 4, 12, 0.7);
    
    // C++: m_backRight{7, 3, 11, 1.7}
    private final SwerveModule backRight = new SwerveModule(7, 3, 11, 1.7);

    // Physical Constants from C++ header
    private static final double kWheelBase = 0.495;
    private static final double kTrackWidth = 0.495;

    // Swerve kinematics
    private final SwerveDriveKinematics kinematics = new SwerveDriveKinematics(
        new Translation2d(kWheelBase/2, kTrackWidth/2),   // Front Left
        new Translation2d(kWheelBase/2, -kTrackWidth/2),  // Front Right
        new Translation2d(-kWheelBase/2, kTrackWidth/2),  // Back Left
        new Translation2d(-kWheelBase/2, -kTrackWidth/2)  // Back Right
    );

    // Odometry
    private final SwerveDriveOdometry odometry;

    // Elevator motors (SparkMax)
    // C++: m_elevatorPivot{13, ...}, m_elevatorPivot2{14, ...}
    private final SparkMax elevatorPivot = new SparkMax(13, MotorType.kBrushless);
    private final SparkMax elevatorPivot2 = new SparkMax(14, MotorType.kBrushless);

    public RobotContainer() {
        // Initialize NavX
        navx = new AHRS(AHRS.NavXComType.kMXP_SPI, (byte) 200);

        // Wait for NavX to connect
        int attempts = 0;
        final int maxAttempts = 250;
        while (!navx.isConnected() && attempts < maxAttempts) {
            try {
                Thread.sleep(20);
            } catch (InterruptedException e) {
                Thread.currentThread().interrupt();
            }
            attempts++;
        }

        // Wait for NavX calibration
        if (navx.isConnected()) {
            while (navx.isCalibrating()) {
                try {
                    Thread.sleep(20);
                } catch (InterruptedException e) {
                    Thread.currentThread().interrupt();
                }
            }
            navx.zeroYaw();
        }

        // Initialize odometry
        odometry = new SwerveDriveOdometry(
            kinematics,
            Rotation2d.fromDegrees(0),
            new SwerveModulePosition[] {
                frontLeft.getPosition(),
                frontRight.getPosition(),
                backLeft.getPosition(),
                backRight.getPosition()
            },
            new Pose2d()
        );

        // Configure elevator leader motor (Using REVLib 2025 Config)
        SparkMaxConfig leaderConfig = new SparkMaxConfig();
        leaderConfig.inverted(false).idleMode(IdleMode.kBrake);
        
        leaderConfig.closedLoop
            .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
            .pid(0.1, 0.0, 0.0)
            .outputRange(-0.5, 0.5);

        elevatorPivot.configure(leaderConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

        // Configure elevator follower motor
        SparkMaxConfig followerConfig = new SparkMaxConfig();
        followerConfig
            .inverted(true)
            .idleMode(IdleMode.kBrake)
            .follow(elevatorPivot); // Note: 2025 follow syntax takes the object, not ID

        elevatorPivot2.configure(followerConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    }

    public void drive(double xSpeed, double ySpeed, double rot, boolean fieldRelative) {
        // Apply deadband
        if (Math.abs(xSpeed) < OperatorConstants.kDeadband) xSpeed = 0.0;
        if (Math.abs(ySpeed) < OperatorConstants.kDeadband) ySpeed = 0.0;
        if (Math.abs(rot) < OperatorConstants.kDeadband) rot = 0.0;

        // Stop all modules if no input
        if (xSpeed == 0.0 && ySpeed == 0.0 && rot == 0.0) {
            frontLeft.setDesiredState(new SwerveModuleState(0.0, frontLeft.getState().angle));
            frontRight.setDesiredState(new SwerveModuleState(0.0, frontRight.getState().angle));
            backLeft.setDesiredState(new SwerveModuleState(0.0, backLeft.getState().angle));
            backRight.setDesiredState(new SwerveModuleState(0.0, backRight.getState().angle));
            return;
        }

        // Convert to chassis speeds
        ChassisSpeeds speeds;
        if (fieldRelative) {
            speeds = ChassisSpeeds.fromFieldRelativeSpeeds(
                xSpeed * AutoConstants.kMaxSpeed,
                ySpeed * AutoConstants.kMaxSpeed,
                rot * AutoConstants.kMaxAngularSpeed,
                Rotation2d.fromDegrees(navx.getYaw())
            );
        } else {
            speeds = new ChassisSpeeds(
                xSpeed * AutoConstants.kMaxSpeed,
                ySpeed * AutoConstants.kMaxSpeed,
                rot * AutoConstants.kMaxAngularSpeed
            );
        }

        // Convert to swerve module states
        var states = kinematics.toSwerveModuleStates(speeds);
        SwerveDriveKinematics.desaturateWheelSpeeds(states, AutoConstants.kMaxSpeed);

        // Set desired states
        frontLeft.setDesiredState(states[0]);
        frontRight.setDesiredState(states[1]);
        backLeft.setDesiredState(states[2]);
        backRight.setDesiredState(states[3]);
    }

    public void updateOdometry() {
        odometry.update(
            Rotation2d.fromDegrees(navx.getYaw()),
            new SwerveModulePosition[] {
                frontLeft.getPosition(),
                frontRight.getPosition(),
                backLeft.getPosition(),
                backRight.getPosition()
            }
        );
    }

    public void setMechanismPosition(double joystickY) {
        final double kMaxPosition = 10.0;
        final double kMinPosition = -10.0;
        final double kDeadband = 0.1;

        if (Math.abs(joystickY) < kDeadband) {
            return;
        }

        double targetPosition = joystickY * kMaxPosition;
        targetPosition = Math.max(kMinPosition, Math.min(kMaxPosition, targetPosition));

        elevatorPivot.getClosedLoopController().setReference(
            targetPosition, ControlType.kPosition
        );
    }

    public Pose2d getPose() {
        return odometry.getPoseMeters();
    }

    public XboxController getSwerveController() {
        return swerveController;
    }

    public XboxController getElevatorController() {
        return elevatorController;
    }

    public AHRS getNavX() {
        return navx;
    }
}
