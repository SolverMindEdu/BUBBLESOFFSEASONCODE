package frc.robot.subsystems;

import static edu.wpi.first.units.Units.*;

import java.util.function.Supplier;

import com.ctre.phoenix6.SignalLogger;
import com.ctre.phoenix6.Utils;
import com.ctre.phoenix6.swerve.SwerveDrivetrainConstants;
import com.ctre.phoenix6.swerve.SwerveModuleConstants;
import com.ctre.phoenix6.swerve.SwerveRequest;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;
import com.pathplanner.lib.path.PathConstraints;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.Notifier;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Subsystem;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import java.util.ArrayList;
import java.util.Arrays;
import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.LimelightHelpers;
import frc.robot.generated.TunerConstants.TunerSwerveDrivetrain;

/**
 * Class that extends the Phoenix 6 SwerveDrivetrain class and implements
 * Subsystem so it can easily be used in command-based projects.
 */
public class CommandSwerveDrivetrain extends TunerSwerveDrivetrain implements Subsystem {
    private static final double kSimLoopPeriod = 0.005; // 5 ms
    private Notifier m_simNotifier = null;
    private double m_lastSimTime;

    /* Blue alliance sees forward as 0 degrees (toward red alliance wall) */
    private static final Rotation2d kBlueAlliancePerspectiveRotation = Rotation2d.kZero;
    /* Red alliance sees forward as 180 degrees (toward blue alliance wall) */
    private static final Rotation2d kRedAlliancePerspectiveRotation = Rotation2d.k180deg;
    /* Keep track if we've ever applied the operator perspective before or not */
    private boolean m_hasAppliedOperatorPerspective = false;

    /* Swerve requests to apply during SysId characterization */
    private final SwerveRequest.SysIdSwerveTranslation m_translationCharacterization = new SwerveRequest.SysIdSwerveTranslation();
    private final SwerveRequest.SysIdSwerveSteerGains m_steerCharacterization = new SwerveRequest.SysIdSwerveSteerGains();
    private final SwerveRequest.SysIdSwerveRotation m_rotationCharacterization = new SwerveRequest.SysIdSwerveRotation();
    // MegaTag localization variables
    private final AprilTagFieldLayout aprilTagFieldLayout = AprilTagFields.k2025ReefscapeAndyMark.loadAprilTagLayoutField();
    private int aprilID;
    private boolean visionEnabled = true;
    private boolean useMegaTag2 = true;

    // // Scoring positions - adjust these coordinates for your field
    // private final Pose2d[] scoringPositionsBlue = {
    //     new Pose2d(1.5, 1.0, Rotation2d.fromDegrees(0)), 
    //     new Pose2d(1.5, 2.0, Rotation2d.fromDegrees(0)), 
    //     new Pose2d(1.5, 3.0, Rotation2d.fromDegrees(0)), 
    //     new Pose2d(1.5, 4.0, Rotation2d.fromDegrees(0))
    // };

    // private final Pose2d[] scoringPositionsRed = {
    //     new Pose2d(20, -14, Rotation2d.fromDegrees(0)),
    //     new Pose2d(20, -14, Rotation2d.fromDegrees(0)),
    //     new Pose2d(20, -14, Rotation2d.fromDegrees(0)), 
    //     new Pose2d(20, -14, Rotation2d.fromDegrees(0))
    // };

    public Pose2d getLeftReefPose() {
        if (isRedAlliance()) {
            // Red alliance left reef position (adjust these coordinates for your field)
            return new Pose2d(20, -14, Rotation2d.fromDegrees(0));
        } else {
            // Blue alliance left reef position (adjust these coordinates for your field)
            return new Pose2d(3.5, 5.5, Rotation2d.fromDegrees(30));
        }
    }
    
    /**
     * Get right side reef position based on alliance
     */
    public Pose2d getRightReefPose() {
        if (isRedAlliance()) {
            // Red alliance right reef position (adjust these coordinates for your field)
            return new Pose2d(20, -14, Rotation2d.fromDegrees(0));
        } else {
            // Blue alliance right reef position (adjust these coordinates for your field)
            return new Pose2d(3.5, 2.5, Rotation2d.fromDegrees(-30));
        }
    }

    /* SysId routine for characterizing translation. This is used to find PID gains for the drive motors. */
    private final SysIdRoutine m_sysIdRoutineTranslation = new SysIdRoutine(
        new SysIdRoutine.Config(
            null,        // Use default ramp rate (1 V/s)
            Volts.of(4), // Reduce dynamic step voltage to 4 V to prevent brownout
            null,        // Use default timeout (10 s)
            // Log state with SignalLogger class
            state -> SignalLogger.writeString("SysIdTranslation_State", state.toString())
        ),
        new SysIdRoutine.Mechanism(
            output -> setControl(m_translationCharacterization.withVolts(output)),
            null,
            this
        )
    );

    /* SysId routine for characterizing steer. This is used to find PID gains for the steer motors. */
    private final SysIdRoutine m_sysIdRoutineSteer = new SysIdRoutine(
        new SysIdRoutine.Config(
            null,        // Use default ramp rate (1 V/s)
            Volts.of(7), // Use dynamic voltage of 7 V
            null,        // Use default timeout (10 s)
            // Log state with SignalLogger class
            state -> SignalLogger.writeString("SysIdSteer_State", state.toString())
        ),
        new SysIdRoutine.Mechanism(
            volts -> setControl(m_steerCharacterization.withVolts(volts)),
            null,
            this
        )
    );

    /*
     * SysId routine for characterizing rotation.
     * This is used to find PID gains for the FieldCentricFacingAngle HeadingController.
     * See the documentation of SwerveRequest.SysIdSwerveRotation for info on importing the log to SysId.
     */
    private final SysIdRoutine m_sysIdRoutineRotation = new SysIdRoutine(
        new SysIdRoutine.Config(
            /* This is in radians per second², but SysId only supports "volts per second" */
            Volts.of(Math.PI / 6).per(Second),
            /* This is in radians per second, but SysId only supports "volts" */
            Volts.of(Math.PI),
            null, // Use default timeout (10 s)
            // Log state with SignalLogger class
            state -> SignalLogger.writeString("SysIdRotation_State", state.toString())
        ),
        new SysIdRoutine.Mechanism(
            output -> {
                /* output is actually radians per second, but SysId only supports "volts" */
                setControl(m_rotationCharacterization.withRotationalRate(output.in(Volts)));
                /* also log the requested output for SysId */
                SignalLogger.writeDouble("Rotational_Rate", output.in(Volts));
            },
            null,
            this
        )
    );

    /* The SysId routine to test */
    private SysIdRoutine m_sysIdRoutineToApply = m_sysIdRoutineTranslation;

    /**
     * Constructs a CTRE SwerveDrivetrain using the specified constants.
     * <p>
     * This constructs the underlying hardware devices, so users should not construct
     * the devices themselves. If they need the devices, they can access them through
     * getters in the classes.
     *
     * @param drivetrainConstants   Drivetrain-wide constants for the swerve drive
     * @param modules               Constants for each specific module
     */
    public CommandSwerveDrivetrain(
        SwerveDrivetrainConstants drivetrainConstants,
        SwerveModuleConstants<?, ?, ?>... modules
    ) {
        super(drivetrainConstants, modules);
        configureAutoBuilder();
        if (Utils.isSimulation()) {
            startSimThread();
        }
    }

    /**
     * Constructs a CTRE SwerveDrivetrain using the specified constants.
     * <p>
     * This constructs the underlying hardware devices, so users should not construct
     * the devices themselves. If they need the devices, they can access them through
     * getters in the classes.
     *
     * @param drivetrainConstants     Drivetrain-wide constants for the swerve drive
     * @param odometryUpdateFrequency The frequency to run the odometry loop. If
     *                                unspecified or set to 0 Hz, this is 250 Hz on
     *                                CAN FD, and 100 Hz on CAN 2.0.
     * @param modules                 Constants for each specific module
     */
    public CommandSwerveDrivetrain(
        SwerveDrivetrainConstants drivetrainConstants,
        double odometryUpdateFrequency,
        SwerveModuleConstants<?, ?, ?>... modules
    ) {
        super(drivetrainConstants, odometryUpdateFrequency, modules);
        configureAutoBuilder();
        if (Utils.isSimulation()) {
            startSimThread();
        }
    }

    /**
     * Constructs a CTRE SwerveDrivetrain using the specified constants.
     * <p>
     * This constructs the underlying hardware devices, so users should not construct
     * the devices themselves. If they need the devices, they can access them through
     * getters in the classes.
     *
     * @param drivetrainConstants       Drivetrain-wide constants for the swerve drive
     * @param odometryUpdateFrequency   The frequency to run the odometry loop. If
     *                                  unspecified or set to 0 Hz, this is 250 Hz on
     *                                  CAN FD, and 100 Hz on CAN 2.0.
     * @param odometryStandardDeviation The standard deviation for odometry calculation
     *                                  in the form [x, y, theta]ᵀ, with units in meters
     *                                  and radians
     * @param visionStandardDeviation   The standard deviation for vision calculation
     *                                  in the form [x, y, theta]ᵀ, with units in meters
     *                                  and radians
     * @param modules                   Constants for each specific module
     */
    public CommandSwerveDrivetrain(
        SwerveDrivetrainConstants drivetrainConstants,
        double odometryUpdateFrequency,
        Matrix<N3, N1> odometryStandardDeviation,
        Matrix<N3, N1> visionStandardDeviation,
        SwerveModuleConstants<?, ?, ?>... modules
    ) {
        super(drivetrainConstants, odometryUpdateFrequency, odometryStandardDeviation, visionStandardDeviation, modules);
        configureAutoBuilder();
        if (Utils.isSimulation()) {
            startSimThread();
        }
    }

    public Pose2d getPose() {
        return getState().Pose;
    }    

    public ChassisSpeeds getRobotRelativeSpeeds() {
        return getState().Speeds;
    }

    public void driveRobotRelative(ChassisSpeeds speeds) {
        setControl(new SwerveRequest.ApplyChassisSpeeds().withSpeeds(speeds));
    }

    private void configureAutoBuilder() {
        RobotConfig config;
        try {
            config = RobotConfig.fromGUISettings();
        } catch (Exception e) {
            e.printStackTrace();
            return;
        }
        
        AutoBuilder.configure(
            this::getPose,
            this::resetPose,
            this::getRobotRelativeSpeeds,
            (speeds, feedforwards) -> driveRobotRelative(speeds),
            new PPHolonomicDriveController(
                new PIDConstants(5.0, 0.0, 0.0), // Translation PID
                new PIDConstants(5.0, 0.0, 0.0)  // Rotation PID
            ),
            config,
            () -> {
                var alliance = DriverStation.getAlliance();
                return alliance.isPresent() && alliance.get() == DriverStation.Alliance.Red;
            },
            this
        );
    }

    /**
     * Returns a command that applies the specified control request to this swerve drivetrain.
     *
     * @param request Function returning the request to apply
     * @return Command to run
     */
    public Command applyRequest(Supplier<SwerveRequest> requestSupplier) {
        return run(() -> this.setControl(requestSupplier.get()));
    }

    /**
     * Runs the SysId Quasistatic test in the given direction for the routine
     * specified by {@link #m_sysIdRoutineToApply}.
     *
     * @param direction Direction of the SysId Quasistatic test
     * @return Command to run
     */
    public Command sysIdQuasistatic(SysIdRoutine.Direction direction) {
        return m_sysIdRoutineToApply.quasistatic(direction);
    }

    /**
     * Runs the SysId Dynamic test in the given direction for the routine
     * specified by {@link #m_sysIdRoutineToApply}.
     *
     * @param direction Direction of the SysId Dynamic test
     * @return Command to run
     */
    public Command sysIdDynamic(SysIdRoutine.Direction direction) {
        return m_sysIdRoutineToApply.dynamic(direction);
    }

    @Override
    public void periodic() {
        /*
         * Periodically try to apply the operator perspective.
         * If we haven't applied the operator perspective before, then we should apply it regardless of DS state.
         * This allows us to correct the perspective in case the robot code restarts mid-match.
         * Otherwise, only check and apply the operator perspective if the DS is disabled.
         * This ensures driving behavior doesn't change until an explicit disable event occurs during testing.
         */
        if (!m_hasAppliedOperatorPerspective || DriverStation.isDisabled()) {
            DriverStation.getAlliance().ifPresent(allianceColor -> {
                setOperatorPerspectiveForward(
                    allianceColor == Alliance.Red
                        ? kRedAlliancePerspectiveRotation
                        : kBlueAlliancePerspectiveRotation
                );
                m_hasAppliedOperatorPerspective = true;
            });
        }
        if (visionEnabled) {
            updateMegaTagOdometry();
        }
    }

    private void mt1HeadingUpdate() {
        boolean doRejectUpdate = false;
        LimelightHelpers.PoseEstimate mt1 = LimelightHelpers.getBotPoseEstimate_wpiBlue("limelight");
        LimelightHelpers.SetRobotOrientation(
            "limelight", getState().Pose.getRotation().getDegrees(), 0, 0, 0, 0, 0);
        
        if (mt1 != null) {
            if (mt1.tagCount == 1 && mt1.rawFiducials.length == 1) {
                if (mt1.rawFiducials[0].ambiguity > 0.7) {
                    doRejectUpdate = true;
                }
                if (mt1.rawFiducials[0].distToCamera > 3) {
                    doRejectUpdate = true;
                }
            }
            if (mt1.tagCount == 0) {
                doRejectUpdate = true;
            }
            if (!doRejectUpdate) {
                // Only trust rotation from MT1, not position
                setVisionMeasurementStdDevs(VecBuilder.fill(9999999, 9999999, 0.7));
                addVisionMeasurement(mt1.pose, mt1.timestampSeconds);
            }
        }
    }

    public void updateMegaTagOdometry() {
        // Set robot orientation for Limelight
        LimelightHelpers.SetRobotOrientation(
            "limelight",
            getState().Pose.getRotation().getDegrees(),
            0, 0, 0, 0, 0
        );

        if (useMegaTag2) {
            // Get MegaTag2 pose estimate
            LimelightHelpers.PoseEstimate mt2 =
                LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2("limelight");

            // Reject updates under certain conditions
            boolean doRejectUpdate = false;

            // Reject if spinning too fast (720 degrees per second)
            if (Math.abs(getState().Speeds.omegaRadiansPerSecond) > Math.toRadians(720)) {
                doRejectUpdate = true;
            }

            if (mt2.tagCount == 0) {
                doRejectUpdate = true;
            }

            // Add vision measurement if valid
            if (!doRejectUpdate) {
                setVisionMeasurementStdDevs(VecBuilder.fill(.7, .7, 9999999));
                addVisionMeasurement(mt2.pose, mt2.timestampSeconds);
                SmartDashboard.putString("Vision Status", "MT2 Updated with " + mt2.tagCount + " tags");
            } else {
                SmartDashboard.putString("Vision Status", "MT2 Update Rejected");
            }
        } else {
            // MegaTag1 fallback implementation
            mt1HeadingUpdate();
        }
    }

    // private void updateMegaTag2() {
    //     // Tell Limelight our robot's current orientation
    //     LimelightHelpers.SetRobotOrientation("limelight", 
    //         getState().Pose.getRotation().getDegrees(), 
    //         0, 0, 0, 0, 0);
        
    //     // Get MegaTag2 pose estimate
    //     LimelightHelpers.PoseEstimate mt2 = LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2("limelight");
        
    //     boolean doRejectUpdate = false;
        
    //     // Reject if no tags visible
    //     if (mt2.tagCount == 0) {
    //         doRejectUpdate = true;
    //     }
        
    //     // Reject if we're spinning too fast
    //     if (Math.abs(getState().Speeds.omegaRadiansPerSecond) > Math.toRadians(360)) {
    //         doRejectUpdate = true;
    //     }
        
    //     // Reject if pose is too far from current estimate (likely bad measurement)
    //     Pose2d currentPose = getState().Pose;
    //     double distanceFromCurrent = currentPose.getTranslation().getDistance(mt2.pose.getTranslation());
    //     if (distanceFromCurrent > 2.0) { // More than 2 meters away = probably bad
    //         doRejectUpdate = true;
    //         SmartDashboard.putString("Vision Status", "Rejected: Too far (" + String.format("%.2f", distanceFromCurrent) + "m)");
    //     }
        
    //     // Only update if we have good data
    //     if (!doRejectUpdate) {
    //         // Use conservative standard deviations
    //         setVisionMeasurementStdDevs(VecBuilder.fill(1.0, 1.0, 9999999));
    //         addVisionMeasurement(mt2.pose, mt2.timestampSeconds);
    //         SmartDashboard.putString("Vision Status", "Updated with " + mt2.tagCount + " tags");
    //     }
    // }

    private void startSimThread() {
        m_lastSimTime = Utils.getCurrentTimeSeconds();

        /* Run simulation at a faster rate so PID gains behave more reasonably */
        m_simNotifier = new Notifier(() -> {
            final double currentTime = Utils.getCurrentTimeSeconds();
            double deltaTime = currentTime - m_lastSimTime;
            m_lastSimTime = currentTime;

            /* use the measured time delta, get battery voltage from WPILib */
            updateSimState(deltaTime, RobotController.getBatteryVoltage());
        });
        m_simNotifier.startPeriodic(kSimLoopPeriod);
    }

    /**
     * Adds a vision measurement to the Kalman Filter. This will correct the odometry pose estimate
     * while still accounting for measurement noise.
     *
     * @param visionRobotPoseMeters The pose of the robot as measured by the vision camera.
     * @param timestampSeconds The timestamp of the vision measurement in seconds.
     */
    @Override
    public void addVisionMeasurement(Pose2d visionRobotPoseMeters, double timestampSeconds) {
        super.addVisionMeasurement(visionRobotPoseMeters, Utils.fpgaToCurrentTime(timestampSeconds));
    }

    /**
     * Adds a vision measurement to the Kalman Filter. This will correct the odometry pose estimate
     * while still accounting for measurement noise.
     * <p>
     * Note that the vision measurement standard deviations passed into this method
     * will continue to apply to future measurements until a subsequent call to
     * {@link #setVisionMeasurementStdDevs(Matrix)} or this method.
     *
     * @param visionRobotPoseMeters The pose of the robot as measured by the vision camera.
     * @param timestampSeconds The timestamp of the vision measurement in seconds.
     * @param visionMeasurementStdDevs Standard deviations of the vision pose measurement
     *     in the form [x, y, theta]ᵀ, with units in meters and radians.
     */
    @Override
    public void addVisionMeasurement(
        Pose2d visionRobotPoseMeters,
        double timestampSeconds,
        Matrix<N3, N1> visionMeasurementStdDevs
    ) {
        super.addVisionMeasurement(visionRobotPoseMeters, Utils.fpgaToCurrentTime(timestampSeconds), visionMeasurementStdDevs);
    }

        /**
     * Get the nearest scoring position based on current robot position
     */
    // public Pose2d getNearestScoringPose() {
    //     Pose2d[] targetPoses = isRedAlliance() ? scoringPositionsRed : scoringPositionsBlue;
    //     Pose2d nearestPose = getPose().nearest(Arrays.asList(targetPoses));
    //     SmartDashboard.putString("Nearest Scoring Pose", nearestPose.toString());
    //     return nearestPose;
    // }

    /**
     * Get the nearest AprilTag reef pose
     */
    // public Pose2d getNearestReefPose() {
    //     int initTag;
    //     int endTag;
    //     ArrayList<Pose2d> reefTagPoses = new ArrayList<Pose2d>(6);
        
    //     if (isRedAlliance()) {
    //         initTag = 6;
    //         endTag = 12;
    //     } else {
    //         initTag = 17; 
    //         endTag = 23;
    //     }
        
    //     // Get poses of reef april tags based on alliance
    //     for (int i = initTag; i < endTag; i++) {
    //         if (aprilTagFieldLayout.getTagPose(i).isPresent()) {
    //             reefTagPoses.add(aprilTagFieldLayout.getTagPose(i).get().toPose2d());
    //         }
    //     }
        
    //     if (reefTagPoses.isEmpty()) {
    //         return getPose(); // Return current pose if no tags found
    //     }
        
    //     // Find nearest reef tag
    //     Pose2d nearestPose = new Pose2d(
    //         getPose().nearest(reefTagPoses).getTranslation(),
    //         getPose().nearest(reefTagPoses).getRotation().rotateBy(Rotation2d.k180deg)
    //     );
        
    //     aprilID = reefTagPoses.indexOf(nearestPose) + (isRedAlliance() ? 6 : 17);
    //     SmartDashboard.putNumber("AprilTag ID", aprilID);
    //     SmartDashboard.putString("Nearest Reef Pose", nearestPose.toString());
        
    //     // Apply chassis offset
    //     double chassisOffset = 0.5; // meters - adjust for your robot
    //     double angle = nearestPose.getRotation().getRadians();
        
    //     return new Pose2d(
    //         nearestPose.getX() - chassisOffset * Math.cos(angle),
    //         nearestPose.getY() + -chassisOffset * Math.sin(angle), 
    //         nearestPose.getRotation()
    //     );
    // }
    
    /**
     * Drive to a specific pose using PathPlanner
     * @param pose Target pose to drive to
     * @return Command that drives to the pose
     */
    public Command driveToPose(Pose2d pose) {
        PathConstraints constraints = new PathConstraints(
            2.0, 2.0,                    // max velocity and acceleration (m/s, m/s^2)
            Math.toRadians(360),         // max angular velocity (rad/s)
            Math.toRadians(720)          // max angular acceleration (rad/s^2)
        );
        
        return AutoBuilder.pathfindToPose(
            pose,
            constraints,
            edu.wpi.first.units.Units.MetersPerSecond.of(0) // end velocity = 0
        );
    }

    /**
     * Auto-align command - drives to the specified pose
     */
    public Command autoAlign(Pose2d pose) {
        return driveToPose(pose);
    }

    /**
     * Check if we're on red alliance
     */
    private boolean isRedAlliance() {
        var alliance = DriverStation.getAlliance();
        return alliance.isPresent() && alliance.get() == DriverStation.Alliance.Red;
    }

    /**
     * Enable or disable vision updates
     */
    public void setVisionEnabled(boolean enabled) {
        this.visionEnabled = enabled;
        SmartDashboard.putBoolean("Vision Enabled", visionEnabled);
    }

    /**
     * Get current vision enabled state
     */
    public boolean isVisionEnabled() {
        return visionEnabled;
    }

    /**
     * Toggle between MegaTag1 and MegaTag2
     */
    public void setUseMegaTag2(boolean useMegaTag2) {
        this.useMegaTag2 = useMegaTag2;
        SmartDashboard.putBoolean("Use MegaTag2", useMegaTag2);
    }

    /**
     * Get current MegaTag mode
     */
    public boolean isUsingMegaTag2() {
        return useMegaTag2;
    }

    // public Command driveToPose(Pose2d pose) {
    //     PathConstraints constraints = new PathConstraints(
    //         2.5, 2.5, // max linear velocity and accel (m/s, m/s^2)
    //         this.getMaximumChassisAngularVelocity(),
    //         Units.degreesToRadians(720) // max angular accel
    //     );
        
    //     return AutoBuilder.pathfindToPose(
    //         pose,
    //         constraints,
    //         edu.wpi.first.units.Units.MetersPerSecond.of(0) // end velocity = 0
    //     );
    // }

    // public Pose2d getPose() {
    //     return getState().Pose;
    // }
    
    // public void resetPose(Pose2d pose) {
    //     super.resetPose(pose);
    // }
    
    
    // public ChassisSpeeds getRobotRelativeSpeeds() {
    //     return getState().Speeds;
    // }
    
    // public void driveRobotRelative(ChassisSpeeds speeds) {
    //     setControl(new SwerveRequest.ApplyChassisSpeeds().withSpeeds(speeds));
    // }
    
    // public Command driveToPose(Pose2d pose) {
    //     PathConstraints constraints = new PathConstraints(
    //         1.5, 1.5,               // max linear velocity and accel (m/s, m/s^2)
    //         MAX_ANGULAR_VELOCITY,   // max angular velocity (rad/s)
    //         Math.toRadians(720)     // max angular accel (rad/s^2)
    //     );
    
    //     return AutoBuilder.pathfindToPose(
    //         pose,
    //         constraints,
    //         edu.wpi.first.units.Units.MetersPerSecond.of(0) // end velocity = 0
    //     );
    // }
}