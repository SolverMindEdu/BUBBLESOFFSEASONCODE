package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.LimelightHelpers;
import com.ctre.phoenix6.swerve.SwerveRequest;

public class LimelightLineupCommand extends Command {
    private final CommandSwerveDrivetrain drivetrain;
    private final CommandXboxController joystick;
    private final PIDController xController;
    private final PIDController yController;
    private final PIDController rotationController;
    
    private final SwerveRequest.FieldCentric driveRequest;
    
    private final double targetTx; // Horizontal offset target
    private final double targetTy; // Vertical offset target  
    private final boolean maintainRotation; // Whether to keep current rotation
    
    /**
     * Creates a Limelight lineup command
     * @param drivetrain The swerve drivetrain
     * @param joystick The Xbox controller to check for driver override
     * @param targetTx Horizontal offset (-27 to 27 degrees). 0=center, negative=left, positive=right
     * @param targetTy Vertical offset (-20 to 20 degrees). Negative=closer, positive=farther
     * @param maintainRotation True to keep current robot rotation, false to rotate toward target
     */
    public LimelightLineupCommand(CommandSwerveDrivetrain drivetrain, CommandXboxController joystick, double targetTx, double targetTy, boolean maintainRotation) {
        this.drivetrain = drivetrain;
        this.joystick = joystick;
        this.targetTx = targetTx;
        this.targetTy = targetTy;
        this.maintainRotation = maintainRotation;
        
        // PID controllers for alignment
        this.xController = new PIDController(0.07, 0.005, 0.005);  // Very smooth
        this.yController = new PIDController(0.06, 0.003, 0.005);  // Very smooth   
        // Distance control  
        this.rotationController = new PIDController(0.04, 0, 0.004); // Rotation alignment
        
        // Set tolerances
        xController.setTolerance(1.0);     // 1 degree tolerance for horizontal
        yController.setTolerance(2.0);     // 2 degree tolerance for vertical
        rotationController.setTolerance(2.0); // 2 degree tolerance for rotation
        
        // Create swerve request
        driveRequest = new SwerveRequest.FieldCentric();
        
        addRequirements(drivetrain);
    }
    
    /**
     * Convenience constructor for center alignment
     */
    public LimelightLineupCommand(CommandSwerveDrivetrain drivetrain, CommandXboxController joystick) {
        this(drivetrain, joystick, 0.0, -5.0, false); // Center, medium distance, rotate to face
    }
    
    /**
     * Create command for left side alignment
     */
    public static LimelightLineupCommand leftSide(CommandSwerveDrivetrain drivetrain, CommandXboxController joystick) {
        return new LimelightLineupCommand(drivetrain, joystick, -17.2, -12.6, true); // Left side, maintain rotation
    }
    
    /**
     * Create command for right side alignment
     */
    public static LimelightLineupCommand rightSide(CommandSwerveDrivetrain drivetrain, CommandXboxController joystick) {
        return new LimelightLineupCommand(drivetrain, joystick, 20.08, -9.75, true); // Right side, maintain rotation
    }
    
    /**
     * Create command for center alignment
     */
    public static LimelightLineupCommand center(CommandSwerveDrivetrain drivetrain, CommandXboxController joystick) {
        return new LimelightLineupCommand(drivetrain, joystick, 0.0, -5.0, false); // Center, rotate to face
    }
    
    private Rotation2d initialRotation;
    
    /**
     * Check if driver is giving significant input on the controller
     */
    private boolean isDriverControlling() {
        double deadband = 0.1; // 10% deadband
        
        return Math.abs(joystick.getLeftY()) > deadband ||
                Math.abs(joystick.getLeftX()) > deadband ||
                Math.abs(joystick.getRightX()) > deadband;
    }
    
    @Override
    public void initialize() {
        xController.reset();
        yController.reset();
        rotationController.reset();
        
        // Save current rotation if we want to maintain it
        if (maintainRotation) {
            initialRotation = drivetrain.getState().Pose.getRotation();
        }
    }
    
    @Override
    public void execute() {
        // Check if driver is controlling - if so, end the command
        if (isDriverControlling()) {
            return; // This will cause isFinished() to be called, ending the command
        }
        
        // Check if we have a valid target
        if (!LimelightHelpers.getTV("limelight")) {
            // No target - stop the robot
            drivetrain.setControl(driveRequest.withVelocityX(0).withVelocityY(0).withRotationalRate(0));
            return;
        }
        
        // Get Limelight values
        double tx = LimelightHelpers.getTX("limelight");  // Horizontal offset (-27 to 27 degrees)
        double ty = LimelightHelpers.getTY("limelight");  // Vertical offset (-20.5 to 20.5 degrees)
        
        // PID calculations
        double xSpeed = -xController.calculate(tx, targetTx);     // Move left/right to target offset
        double ySpeed = -yController.calculate(ty, targetTy);     // Move forward/back to target distance
        
        double rotSpeed = 0;
        if (maintainRotation) {
            // Maintain the initial rotation
            Rotation2d currentRotation = drivetrain.getState().Pose.getRotation();
            double rotationError = initialRotation.getDegrees() - currentRotation.getDegrees();
            
            // Handle angle wraparound
            while (rotationError > 180) rotationError -= 360;
            while (rotationError < -180) rotationError += 360;
            
            rotSpeed = rotationController.calculate(0, rotationError);
        } else {
            // Rotate to face the target (center it horizontally for rotation)
            rotSpeed = -rotationController.calculate(tx, 0);
        }
        
        // Apply speed limits
        xSpeed = Math.max(-1.0, Math.min(1.0, xSpeed));     // Limit to ±1 m/s
        ySpeed = Math.max(-1.0, Math.min(1.0, ySpeed));     // Limit to ±1 m/s
        rotSpeed = Math.max(-2.0, Math.min(2.0, rotSpeed)); // Limit to ±2 rad/s
        
        // Send commands to drivetrain
        drivetrain.setControl(driveRequest
            .withVelocityX(ySpeed)           // Forward/backward
            .withVelocityY(xSpeed)           // Left/right
            .withRotationalRate(rotSpeed));  // Rotation
    }
    
    @Override
    public boolean isFinished() {
        // End command if driver is controlling
        if (isDriverControlling()) {
            return true;
        }
        
        // Command finishes when all controllers are at setpoint and we have a target
        boolean positionReached = xController.atSetpoint() && yController.atSetpoint();
        boolean rotationReached = maintainRotation ? 
            rotationController.atSetpoint() : 
            rotationController.atSetpoint();
            
        return positionReached && rotationReached && LimelightHelpers.getTV("limelight");
    }
    
    @Override
    public void end(boolean interrupted) {
        // Stop the robot
        drivetrain.setControl(driveRequest.withVelocityX(0).withVelocityY(0).withRotationalRate(0));
    }
}