// Copyright (c) FIRST and other WPILib contributors.

// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.math.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.wpilibj.ADXRS450_Gyro;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import frc.robot.Constants.DriveConstants;
import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;
import edu.wpi.first.wpilibj.motorcontrol.PWMSparkMax;
import edu.wpi.first.wpilibj.simulation.ADXRS450_GyroSim;
import edu.wpi.first.wpilibj.simulation.DifferentialDrivetrainSim;
import edu.wpi.first.wpilibj.simulation.EncoderSim;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;



public class DriveSubsystem extends SubsystemBase
{
    // The motors on the left side of the drive.
    private final MotorControllerGroup leftMotors =
            new MotorControllerGroup(
                    new PWMSparkMax(DriveConstants.LEFT_MOTOR_1_PORT),
                    new PWMSparkMax(DriveConstants.LEFT_MOTOR_2_PORT));
    
    // The motors on the right side of the drive.
    private final MotorControllerGroup rightMotors =
            new MotorControllerGroup(
                    new PWMSparkMax(DriveConstants.RIGHT_MOTOR_1_PORT),
                    new PWMSparkMax(DriveConstants.RIGHT_MOTOR_2_PORT));
    
    // The robot's drive
    private final DifferentialDrive drive = new DifferentialDrive(leftMotors, rightMotors);
    
    // The left-side drive encoder
    private final Encoder leftEncoder =
            new Encoder(
                    DriveConstants.LEFT_ENCODER_PORTS[0],
                    DriveConstants.LEFT_ENCODER_PORTS[1],
                    DriveConstants.LEFT_ENCODER_REVERSED);
    
    // The right-side drive encoder
    private final Encoder rightEncoder =
            new Encoder(
                    DriveConstants.RIGHT_ENCODER_PORTS[0],
                    DriveConstants.RIGHT_ENCODER_PORTS[1],
                    DriveConstants.RIGHT_ENCODER_REVERSED);
    
    // The gyro sensor
    private final ADXRS450_Gyro gyro = new ADXRS450_Gyro();
    
    // Odometry class for tracking robot pose
    private final DifferentialDriveOdometry odometry;
    
    // These classes help us simulate our drivetrain
    public DifferentialDrivetrainSim drivetrainSimulator;
    private EncoderSim leftEncoderSim;
    private EncoderSim rightEncoderSim;
    // The Field2d class shows the field in the sim GUI
    private Field2d fieldSim;
    private ADXRS450_GyroSim gyroSim;
    
    
    /** Creates a new DriveSubsystem. */
    public DriveSubsystem()
    {
        // We need to invert one side of the drivetrain so that positive voltages
        // result in both sides moving forward. Depending on how your robot's
        // gearbox is constructed, you might have to invert the left side instead.
        rightMotors.setInverted(true);
        
        // Sets the distance per pulse for the encoders
        leftEncoder.setDistancePerPulse(DriveConstants.ENCODER_DISTANCE_PER_PULSE);
        rightEncoder.setDistancePerPulse(DriveConstants.ENCODER_DISTANCE_PER_PULSE);
        
        resetEncoders();
        odometry = new DifferentialDriveOdometry(Rotation2d.fromDegrees(getHeading()));
        
        if (RobotBase.isSimulation())
        { // If our robot is simulated
            // This class simulates our drivetrain's motion around the field.
            drivetrainSimulator =
                    new DifferentialDrivetrainSim(
                            DriveConstants.DRIVETRAIN_PLANT,
                            DriveConstants.DRIVE_GEARBOX,
                            DriveConstants.DRIVE_GEARING,
                            DriveConstants.TRACK_WIDTH_METERS,
                            DriveConstants.WHEEL_DIAMETER_METERS / 2.0,
                            VecBuilder.fill(0, 0, 0.0001, 0.1, 0.1, 0.005, 0.005));
            
            // The encoder and gyro angle sims let us set simulated sensor readings
            leftEncoderSim = new EncoderSim(leftEncoder);
            rightEncoderSim = new EncoderSim(rightEncoder);
            gyroSim = new ADXRS450_GyroSim(gyro);
            
            // the Field2d class lets us visualize our robot in the simulation GUI.
            fieldSim = new Field2d();
            SmartDashboard.putData("Field", fieldSim);
        }
    }
    
    
    @Override
    public void periodic()
    {
        // Update the odometry in the periodic block
        odometry.update(
                Rotation2d.fromDegrees(getHeading()),
                leftEncoder.getDistance(),
                rightEncoder.getDistance());
        fieldSim.setRobotPose(getPose());
    }
    
    
    @Override
    public void simulationPeriodic()
    {
        // To update our simulation, we set motor voltage inputs, update the simulation,
        // and write the simulated positions and velocities to our simulated encoder and gyro.
        // We negate the right side so that positive voltages make the right side
        // move forward.
        drivetrainSimulator.setInputs(
                leftMotors.get() * RobotController.getBatteryVoltage(),
                rightMotors.get() * RobotController.getBatteryVoltage());
        drivetrainSimulator.update(0.020);
        
        leftEncoderSim.setDistance(drivetrainSimulator.getLeftPositionMeters());
        leftEncoderSim.setRate(drivetrainSimulator.getLeftVelocityMetersPerSecond());
        rightEncoderSim.setDistance(drivetrainSimulator.getRightPositionMeters());
        rightEncoderSim.setRate(drivetrainSimulator.getRightVelocityMetersPerSecond());
        gyroSim.setAngle(-drivetrainSimulator.getHeading().getDegrees());
    }
    
    
    /**
     * Returns the current being drawn by the drivetrain. This works in SIMULATION ONLY! If you want
     * it to work elsewhere, use the code in {@link DifferentialDrivetrainSim#getCurrentDrawAmps()}
     *
     * @return The drawn current in Amps.
     */
    public double getDrawnCurrentAmps()
    {
        return drivetrainSimulator.getCurrentDrawAmps();
    }
    
    
    /**
     * Returns the currently-estimated pose of the robot.
     *
     * @return The pose.
     */
    public Pose2d getPose()
    {
        return odometry.getPoseMeters();
    }
    
    
    /**
     * Returns the current wheel speeds of the robot.
     *
     * @return The current wheel speeds.
     */
    public DifferentialDriveWheelSpeeds getWheelSpeeds()
    {
        return new DifferentialDriveWheelSpeeds(leftEncoder.getRate(), rightEncoder.getRate());
    }
    
    
    /**
     * Resets the odometry to the specified pose.
     *
     * @param pose The pose to which to set the odometry.
     */
    public void resetOdometry(Pose2d pose)
    {
        resetEncoders();
//        drivetrainSimulator.setPose(pose);
        odometry.resetPosition(pose, Rotation2d.fromDegrees(getHeading()));
    }
    
    
    /**
     * Drives the robot using arcade controls.
     *
     * @param fwd      the commanded forward movement
     * @param rotation the commanded rotation
     */
    public void arcadeDrive(double fwd, double rotation)
    {
        drive.arcadeDrive(fwd, rotation);
    }
    
    
    /**
     * Controls the left and right sides of the drive directly with voltages.
     *
     * @param leftVolts  the commanded left output
     * @param rightVolts the commanded right output
     */
    public void tankDriveVolts(double leftVolts, double rightVolts)
    {
        var batteryVoltage = RobotController.getBatteryVoltage();
        if (Math.max(Math.abs(leftVolts), Math.abs(rightVolts)) > batteryVoltage)
        {
            leftVolts *= batteryVoltage / 12.0;
            rightVolts *= batteryVoltage / 12.0;
        }
        leftMotors.setVoltage(leftVolts);
        rightMotors.setVoltage(rightVolts);
        drive.feed();
    }
    
    
    /** Resets the drive encoders to currently read a position of 0. */
    public void resetEncoders()
    {
        leftEncoder.reset();
        rightEncoder.reset();
    }
    
    
    /**
     * Gets the average distance of the two encoders.
     *
     * @return the average of the two encoder readings
     */
    public double getAverageEncoderDistance()
    {
        return (leftEncoder.getDistance() + rightEncoder.getDistance()) / 2.0;
    }
    
    
    /**
     * Gets the left drive encoder.
     *
     * @return the left drive encoder
     */
    public Encoder getLeftEncoder()
    {
        return leftEncoder;
    }
    
    
    /**
     * Gets the right drive encoder.
     *
     * @return the right drive encoder
     */
    public Encoder getRightEncoder()
    {
        return rightEncoder;
    }
    
    
    /**
     * Sets the max output of the drive. Useful for scaling the drive to drive more slowly.
     *
     * @param maxOutput the maximum output to which the drive will be constrained
     */
    public void setMaxOutput(double maxOutput)
    {
        drive.setMaxOutput(maxOutput);
    }
    
    
    /** Zeroes the heading of the robot. */
    public void zeroHeading()
    {
        gyro.reset();
    }
    
    
    /**
     * Returns the heading of the robot.
     *
     * @return the robot's heading in degrees, from -180 to 180
     */
    public double getHeading()
    {
        return Math.IEEEremainder(gyro.getAngle(), 360) * (DriveConstants.GYRO_REVERSED ? -1.0 : 1.0);
    }


    public void addTrajectoryToField(Trajectory trajectory) {
        fieldSim.getObject("traj").setTrajectory(trajectory);
    }
}
