// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.*;

import java.util.Set;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.RobotModeTriggers;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;
import frc.robot.commands.AlgaePivotCommands;
import frc.robot.commands.ElevatorCommands;
import frc.robot.commands.EndEffectorCommands;
import frc.robot.commands.IntakeRollersCommands;
import frc.robot.commands.RunEndEffectorCommand;
import frc.robot.commands.SensorRangeCommands;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.AlgaeDetectRangeSubsystem;
import frc.robot.subsystems.AlgaePivotSubsystem;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.EECoralRangeSubsystem;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.EndEffectorSubsystem;
import frc.robot.subsystems.IntakeRangeSubsystem;
import frc.robot.subsystems.IntakeRollersSubsystems;
import frc.robot.subsystems.LimelightSubsystem;

public class RobotContainer {
    private final ElevatorSubsystem elevator = new ElevatorSubsystem();
    private final AlgaePivotSubsystem algaepivot = new AlgaePivotSubsystem();
    private final EndEffectorSubsystem endEffector = new EndEffectorSubsystem();
    private final IntakeRollersSubsystems intakeRollers = new IntakeRollersSubsystems();
    private final EECoralRangeSubsystem eeCoral = new EECoralRangeSubsystem();
    private final AlgaeDetectRangeSubsystem algaeDetect = new AlgaeDetectRangeSubsystem();
    private final IntakeRangeSubsystem intakeRange = new IntakeRangeSubsystem();
    private final LimelightSubsystem limelight = new LimelightSubsystem();  // ✅ Added here
    private final SendableChooser<Command> autoChooser;

    private RobotMode selectedMode = RobotMode.NONE;
    private int modeStep = 0;

    private double SetMax = 0.4;
    private double MaxSpeed = TunerConstants.kSpeedAt12Volts.in(MetersPerSecond); // kSpeedAt12Volts desired top speed
    private double MaxAngularRate = RotationsPerSecond.of(0.5).in(RadiansPerSecond); // 3/4 of a rotation per second max angular velocity

    /* Setting up bindings for necessary control of the swerve drive platform */
    private final SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric()
            .withDeadband(MaxSpeed * 0.1).withRotationalDeadband(MaxAngularRate * 0.1) // Add a 10% deadband
            .withDriveRequestType(DriveRequestType.OpenLoopVoltage); // Use open-loop control for drive motors
    private final SwerveRequest.SwerveDriveBrake brake = new SwerveRequest.SwerveDriveBrake();
    private final SwerveRequest.PointWheelsAt point = new SwerveRequest.PointWheelsAt();

    private final Telemetry logger = new Telemetry(MaxSpeed);

    private final CommandXboxController joystick = new CommandXboxController(0);
    private final CommandXboxController opjoystick = new CommandXboxController(1);

    public final CommandSwerveDrivetrain drivetrain = TunerConstants.createDrivetrain();

    public RobotContainer() {
        configureBindings();
        autoChooser = AutoBuilder.buildAutoChooser();
        SmartDashboard.putData("Auto Chooser", autoChooser);
    }

    private void configureBindings() {
        // Note that X is defined as forward according to WPILib convention,
        // and Y is defined as to the left according to WPILib convention.
        drivetrain.setDefaultCommand(
            // Drivetrain will execute this command periodically
            drivetrain.applyRequest(() ->
                drive.withVelocityX(-joystick.getLeftY() * MaxSpeed * SetMax) // Drive forward with negative Y (forward)
                    .withVelocityY(-joystick.getLeftX() * MaxSpeed * SetMax) // Drive left with negative X (left)
                    .withRotationalRate(-joystick.getRightX() * MaxAngularRate) // Drive counterclockwise with negative X (left)
            )
        );

        // Idle while the robot is disabled. This ensures the configured
        // neutral mode is applied to the drive motors while disabled.
        final var idle = new SwerveRequest.Idle();
        RobotModeTriggers.disabled().whileTrue(
            drivetrain.applyRequest(() -> idle).ignoringDisable(true)
        );

        // Run SysId routines when holding back/start and X/Y.
        // Note that each routine should be run exactly once in a single log.
        joystick.back().and(joystick.y()).whileTrue(drivetrain.sysIdDynamic(Direction.kForward));
        joystick.back().and(joystick.x()).whileTrue(drivetrain.sysIdDynamic(Direction.kReverse));
        joystick.start().and(joystick.y()).whileTrue(drivetrain.sysIdQuasistatic(Direction.kForward));
        joystick.start().and(joystick.x()).whileTrue(drivetrain.sysIdQuasistatic(Direction.kReverse));

        // reset the field-centric heading on left bumper press
        joystick.leftBumper().onTrue(drivetrain.runOnce(() -> drivetrain.seedFieldCentric()));

        // Operator selects mode
        opjoystick.rightTrigger().onTrue(
            new InstantCommand(() -> selectedMode = RobotMode.INTAKE)
        );

        opjoystick.x().onTrue(
            new InstantCommand(() -> {
                selectedMode = RobotMode.SCORE_L2;
                modeStep = 0;
                System.out.println("Mode set to SCORE_L2");
            })
        );

        opjoystick.y().onTrue(
            new InstantCommand(() -> {
                selectedMode = RobotMode.SCORE_L3;
                modeStep = 0;
                System.out.println("Mode set to SCORE_L3");
            })
        );

        opjoystick.a().onTrue(
            new InstantCommand(() -> {
                selectedMode = RobotMode.SCORE_L1;
                modeStep = 0;
                System.out.println("Mode set to SCORE_L1-");
            })
        );

        opjoystick.b().onTrue(
            new InstantCommand(() -> {
                selectedMode = RobotMode.SCORE_L4;
                modeStep = 0;
                System.out.println("Mode set to SCORE_L4-");
            })
        );

        opjoystick.rightTrigger().and(opjoystick.x()).onTrue(
            new InstantCommand(() -> {
                selectedMode = RobotMode.ALGAE1;
                modeStep = 0;
                System.out.println("Mode set to Algae1-");
            })
        );

        opjoystick.rightTrigger().and(opjoystick.b()).onTrue(
            new InstantCommand(() -> {
                selectedMode = RobotMode.PROCESSOR;
                modeStep = 0;
                System.out.println("Mode set to Processor-");
            })
        );

        joystick.rightBumper().onTrue(
            drivetrain.autoAlign(drivetrain.getNearestReefPose())
        );


        // joystick.rightBumper().onTrue(
        //     AutoAlignToPose.create(
        //         drivetrain,
        //         new Pose2d(-22.0, -16.7, new Rotation2d(Math.PI)) // Example pose (x=3m, y=4m, facing 180°)
        //     )
        // );


        joystick.rightTrigger().onTrue(
            new InstantCommand(() -> {
                switch (selectedMode) {
                    case INTAKE:
                        Commands.sequence(
                            AlgaePivotCommands.intakeMove(algaepivot),
                            Commands.waitSeconds(0.1),
                            ElevatorCommands.intake(elevator),
                            IntakeRollersCommands.run(intakeRollers, 0.5),
                            EndEffectorCommands.run(endEffector, 0.3),
                            SensorRangeCommands.waitForIntake(intakeRange, 0, 280),
                            IntakeRollersCommands.run(intakeRollers, 0.15),
                            SensorRangeCommands.waitForCoral(eeCoral, 0, 100),
                            Commands.waitSeconds(0.15),
                            EndEffectorCommands.stop(endEffector),
                            IntakeRollersCommands.stop(intakeRollers),
                            ElevatorCommands.reset(elevator),
                            AlgaePivotCommands.stow(algaepivot)
                        ).schedule();
                        break;

                    case ALGAE1:
                        Commands.sequence(
                            AlgaePivotCommands.elevatorMove(algaepivot),
                            Commands.waitSeconds(0.1),
                            ElevatorCommands.Algae1(elevator),
                            AlgaePivotCommands.algae(algaepivot),
                            EndEffectorCommands.run(endEffector, 0.4),
                            SensorRangeCommands.waitForAlgae(algaeDetect, 0, 70),
                            Commands.waitSeconds(1),
                            AlgaePivotCommands.elevatorMove(algaepivot),
                            ElevatorCommands.reset(elevator),
                            AlgaePivotCommands.stow(algaepivot),
                            EndEffectorCommands.run(endEffector, 0.08)
                        ).schedule();
                        break;

                    case PROCESSOR:
                        Commands.sequence(
                            AlgaePivotCommands.process(algaepivot),
                            EndEffectorCommands.run(endEffector, -0.3),
                            Commands.waitSeconds(1),
                            EndEffectorCommands.stop(endEffector),
                            AlgaePivotCommands.stow(algaepivot)
                        ).schedule();
                        break;                

                    case SCORE_L1:
                        Commands.sequence(
                            SensorRangeCommands.waitForCoral(eeCoral, 0, 100),
                                    AlgaePivotCommands.elevatorMove(algaepivot),
                                    Commands.waitSeconds(0.1),
                                    ElevatorCommands.goToLevel1(elevator),
                                    AlgaePivotCommands.stow(algaepivot),
                                    Commands.waitSeconds(1),
                                    EndEffectorCommands.run(endEffector, 0.2),
                                    Commands.waitSeconds(0.8),
                                    EndEffectorCommands.stop(endEffector),
                                    AlgaePivotCommands.elevatorMove(algaepivot),
                                    ElevatorCommands.reset(elevator),
                                    AlgaePivotCommands.stow(algaepivot)
                        ).schedule();
                        break;

                    case SCORE_L2:
                        switch (modeStep) {
                            case 0:
                                // Step 1: Get ready at scoring position
                                Commands.sequence(
                                    SensorRangeCommands.waitForCoral(eeCoral, 0, 100),
                                    AlgaePivotCommands.elevatorMove(algaepivot),
                                    Commands.waitSeconds(0.1),
                                    ElevatorCommands.goToLevel2(elevator),
                                    AlgaePivotCommands.stow(algaepivot)
                                ).schedule();
                                System.out.println("SCORE_L2 Step 1 done, press RT for Step 2");
                                modeStep++;
                                break;
        
                            case 1:
                                Commands.sequence(
                                    EndEffectorCommands.run(endEffector, 0.6),
                                    Commands.waitSeconds(0.8),
                                    EndEffectorCommands.stop(endEffector),
                                    AlgaePivotCommands.elevatorMove(algaepivot),
                                    ElevatorCommands.reset(elevator),
                                    AlgaePivotCommands.stow(algaepivot)
                                ).schedule();
                                System.out.println("SCORE_L2 Step 2 done, sequence complete");
                                modeStep = 0; // reset steps
                                break;
        
                            default:
                                modeStep = 0;
                                break;
                        }
                        break;

                    case SCORE_L3:
                        switch (modeStep) {
                            case 0:
                                // Step 1: Get ready at scoring position
                                Commands.sequence(
                                    SensorRangeCommands.waitForCoral(eeCoral, 0, 100),
                                    AlgaePivotCommands.elevatorMove(algaepivot),
                                    Commands.waitSeconds(0.1),
                                    ElevatorCommands.goToLevel3(elevator),
                                    AlgaePivotCommands.stow(algaepivot)
                                ).schedule();
                                System.out.println("SCORE_L3 Step 1 done, press RT for Step 2");
                                modeStep++;
                                break;
        
                            case 1:
                                Commands.sequence(
                                    EndEffectorCommands.run(endEffector, 0.6),
                                    Commands.waitSeconds(0.8),
                                    EndEffectorCommands.stop(endEffector),
                                    AlgaePivotCommands.elevatorMove(algaepivot),
                                    ElevatorCommands.reset(elevator),
                                    AlgaePivotCommands.stow(algaepivot)
                                ).schedule();
                                System.out.println("SCORE_L3 Step 2 done, sequence complete");
                                modeStep = 0; // reset steps
                                break;
        
                            default:
                                modeStep = 0;
                                break;
                        }
                        break;

                    case SCORE_L4:
                        switch (modeStep) {
                            case 0:
                                // Step 1: Get ready at scoring position
                                Commands.sequence(
                                    SensorRangeCommands.waitForCoral(eeCoral, 0, 100),
                                    AlgaePivotCommands.elevatorMove(algaepivot),
                                    Commands.waitSeconds(0.1),
                                    ElevatorCommands.goToLevel4(elevator),
                                    AlgaePivotCommands.stow(algaepivot)
                                ).schedule();
                                System.out.println("SCORE_L4 Step 1 done, press RT for Step 2");
                                modeStep++;
                                break;
        
                            case 1:
                                Commands.sequence(
                                    EndEffectorCommands.run(endEffector, 0.6),
                                    Commands.waitSeconds(0.8),
                                    EndEffectorCommands.stop(endEffector),
                                    AlgaePivotCommands.elevatorMove(algaepivot),
                                    ElevatorCommands.reset(elevator),
                                    AlgaePivotCommands.stow(algaepivot)
                                ).schedule();
                                System.out.println("SCORE_L3 Step 2 done, sequence complete");
                                modeStep = 0; // reset steps
                                break;
        
                            default:
                                modeStep = 0;
                                break;
                        }
                        break;

                    default:
                        System.out.println("No mode selected!");
                        break;
                }
            })
        );


        // --------- ELEVATOR BINDINGS ---------
        // joystick.x().onTrue(AlgaePivotCommands.elevatorMove(algaepivot));
        // joystick.a().onTrue(AlgaePivotCommands.stow(algaepivot));
        //joystick.x().onTrue(ElevatorCommands.goToLevel1(elevator));

        // joystick.x().onTrue(new InstantCommand(() -> SetMax = 0.15));
        // joystick.x().onTrue(
        //     Commands.sequence(
        //         AlgaePivotCommands.elevatorMove(algaepivot),
        //         Commands.waitSecond     s(0.1),      
        //         ElevatorCommands.goToLevel3(elevator),
        //         Commands.waitSeconds(0.5),
        //         AlgaePivotCommands.stow(algaepivot),
        //         Commands.waitSeconds(3),
        //         EndEffectorCommands.run(endEffector, 0.6)
        //     )
            // );

        // joystick.a().onTrue(
        //     Commands.sequence(
        //         AlgaePivotCommands.elevatorMove(algaepivot),
        //         Commands.waitSeconds(0.2),
        //         ElevatorCommands.reset(elevator),
        //         Commands.waitSeconds(0.1), 
        //         AlgaePivotCommands.stow(algaepivot),
        //         Commands.waitSeconds(2), 
        //         new InstantCommand(() -> SetMax = 0.5)
        //     )
        // );

        // joystick.rightTrigger().onTrue(
        //     Commands.sequence(
        //         AlgaePivotCommands.elevatorMove(algaepivot),
        //         Commands.waitSeconds(0.1),
        //         ElevatorCommands.intake(elevator),
        //         IntakeRollersCommands.run(intakeRollers, 0.26),
        //         EndEffectorCommands.run(endEffector, 0.2)
        //     )
        // );

        //joystick.rightTrigger().onTrue(EndEffectorCommands.run(endEffector, 0.2));
        // joystick.rightBumper().onTrue(EndEffectorCommands.stop(endEffector));
        // joystick.rightBumper().onTrue(IntakeRollersCommands.stop(intakeRollers));
        // joystick.rightBumper().onTrue(AlgaePivotCommands.stow(algaepivot));

        drivetrain.registerTelemetry(logger::telemeterize);
    }

    public Command getAutonomousCommand() {
        return autoChooser.getSelected();
    }
}
