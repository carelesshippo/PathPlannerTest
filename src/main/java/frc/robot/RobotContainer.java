// Copyright (c) FIRST and other WPILib contributors.

// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.io.IOException;
import java.nio.file.Path;
import java.util.List;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.RamseteController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.math.trajectory.TrajectoryUtil;
import edu.wpi.first.math.trajectory.constraint.DifferentialDriveVoltageConstraint;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.XboxController.Button;
import frc.robot.subsystems.DriveSubsystem;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.RamseteCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;



/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the Robot periodic
 * methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer
{
    // The robot's subsystems
    private final DriveSubsystem robotDrive = new DriveSubsystem();
    
    // The driver's controller
    XboxController driverController =
            new XboxController(Constants.OIConstants.DRIVER_CONTROLLER_PORT);
    
    
    /** The container for the robot. Contains subsystems, OI devices, and commands. */
    public RobotContainer()
    {
        // Configure the button bindings
        configureButtonBindings();
        
        // Configure default commands
        // Set the default drive command to split-stick arcade drive
        robotDrive.setDefaultCommand(
                // A split-stick arcade command, with forward/backward controlled by the left
                // hand, and turning controlled by the right.
                new RunCommand(
                        () ->
                                robotDrive.arcadeDrive(
                                        -driverController.getLeftY(), driverController.getRightX()),
                        robotDrive));
    }
    
    
    /**
     * Use this method to define your button->command mappings. Buttons can be created by
     * instantiating a {@link edu.wpi.first.wpilibj.GenericHID} or one of its subclasses ({@link
     * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then calling passing it to a
     * {@link JoystickButton}.
     */
    private void configureButtonBindings()
    {
        // Drive at half speed when the right bumper is held
        new JoystickButton(driverController, Button.kRightBumper.value)
                .whenPressed(() -> robotDrive.setMaxOutput(0.5))
                .whenReleased(() -> robotDrive.setMaxOutput(1));
    }
    
    
    public DriveSubsystem getRobotDrive()
    {
        return robotDrive;
    }
    
    
    /** Zeros the outputs of all subsystems. */
    public void zeroAllOutputs()
    {
        robotDrive.tankDriveVolts(0, 0);
    }
    
    
    /**
     * Use this to pass the autonomous command to the main {@link Robot} class.
     *
     * @return the command to run in autonomous
     */
    public Command getAutonomousCommand()
    {
//        Path trajectoryPath = Filesystem.getDeployDirectory().toPath().resolve("output/Unnamed.wpilib.json");
        Path trajectoryPath = Filesystem.getDeployDirectory().toPath().resolve("pathplanner/generatedJSON/New Path.wpilib.json");
        Trajectory exampleTrajectory;
        try {
            exampleTrajectory = TrajectoryUtil.fromPathweaverJson(trajectoryPath);
        } catch (IOException e) {
            DriverStation.reportError("Cannot find trajectory", e.getStackTrace());
            return null;
        }

        robotDrive.addTrajectoryToField(exampleTrajectory);

        RamseteCommand ramseteCommand =
                new RamseteCommand(
                        exampleTrajectory,
                        robotDrive::getPose,
                        new RamseteController(
                                Constants.AutoConstants.RAMSETE_B, Constants.AutoConstants.RAMSETE_ZETA),
                        new SimpleMotorFeedforward(
                                Constants.DriveConstants.ksVolts,
                                Constants.DriveConstants.kvVoltSecondsPerMeter,
                                Constants.DriveConstants.kaVoltSecondsSquaredPerMeter),
                        Constants.DriveConstants.DRIVE_KINEMATICS,
                        robotDrive::getWheelSpeeds,
                        new PIDController(Constants.DriveConstants.P_DRIVE_VEL, 0, 0),
                        new PIDController(Constants.DriveConstants.P_DRIVE_VEL, 0, 0),
                        // RamseteCommand passes volts to the callback
                        robotDrive::tankDriveVolts,
                        robotDrive);
        
        // Reset odometry to starting pose of trajectory.
        robotDrive.resetOdometry(exampleTrajectory.getInitialPose());
        
        // Run path following command, then stop at the end.
        return ramseteCommand.andThen(() -> robotDrive.tankDriveVolts(0, 0));
    }
}
