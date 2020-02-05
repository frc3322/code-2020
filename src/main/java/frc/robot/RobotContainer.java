/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import frc.robot.commands.DriveControl;
import frc.robot.commands.LedControl;
import frc.robot.subsystems.*;

import java.util.List;

import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.controller.PIDController;
import edu.wpi.first.wpilibj.controller.RamseteController;
import edu.wpi.first.wpilibj.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.geometry.Translation2d;
import edu.wpi.first.wpilibj.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.trajectory.Trajectory;
import edu.wpi.first.wpilibj.trajectory.TrajectoryConfig;
import edu.wpi.first.wpilibj.trajectory.TrajectoryGenerator;
import edu.wpi.first.wpilibj.trajectory.constraint.DifferentialDriveVoltageConstraint;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RamseteCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.button.*;
import frc.robot.Constants.RobotMap;

/**
 * This class is where the bulk of the robot should be declared. Since
 * Command-based is a "declarative" paradigm, very little robot logic should
 * actually be handled in the {@link Robot} periodic methods (other than the
 * scheduler calls). Instead, the structure of the robot (including subsystems,
 * commands, and button mappings) should be declared here.
 */

public class RobotContainer {
    private final Drivetrain drivetrain = new Drivetrain();
    private final Shooter shooter = new Shooter();
    private final Hopper hopper = new Hopper();
    private final Intake intake = new Intake();
    private final Feeder feeder = new Feeder();

    private final Joystick lowerChassis = new Joystick(0);
    private final Joystick upperChassis = new Joystick(1);

    public RobotContainer() {
        
        configureButtonBindings();
        getAutonomousCommand();

        drivetrain.setDefaultCommand(new DriveControl(drivetrain, lowerChassis));

        feeder.putInitialDash();
        shooter.putInitialDash();
        hopper.putInitialDash();
    }

    private void configureButtonBindings() {
        Button button_a_upper = new JoystickButton(upperChassis, RobotMap.XBOX.BUTTON_A);
        Button button_b_upper = new JoystickButton(upperChassis, RobotMap.XBOX.BUTTON_B);
        Button button_x_upper = new JoystickButton(upperChassis, RobotMap.XBOX.BUTTON_X);
        Button button_y_upper = new JoystickButton(upperChassis, RobotMap.XBOX.BUTTON_Y);
        Button bumper_left_upper = new JoystickButton(upperChassis, RobotMap.XBOX.BUMPER_LEFT);
        Button bumper_right_upper = new JoystickButton(upperChassis, RobotMap.XBOX.BUMPER_RIGHT);
        Button button_back_upper = new JoystickButton(upperChassis, RobotMap.XBOX.BUTTON_BACK);
        Button button_start_upper = new JoystickButton(upperChassis, RobotMap.XBOX.BUTTON_START);

        Button bumper_left_lower = new JoystickButton(lowerChassis, RobotMap.XBOX.BUMPER_LEFT);
        Button bumper_right_lower = new JoystickButton(lowerChassis, RobotMap.XBOX.BUMPER_RIGHT);
        Button button_back_lower = new JoystickButton(lowerChassis, RobotMap.XBOX.BUTTON_BACK);
        Button button_start_lower = new JoystickButton(lowerChassis, RobotMap.XBOX.BUTTON_START);
        Button left_stick_lower = new JoystickButton(lowerChassis, RobotMap.XBOX.STICK_LEFT);
        Button right_stick_lower = new JoystickButton(lowerChassis, RobotMap.XBOX.STICK_RIGHT);
        Button button_a_lower = new JoystickButton(lowerChassis, RobotMap.XBOX.BUTTON_A);
        Button button_x_lower = new JoystickButton(lowerChassis, RobotMap.XBOX.BUTTON_X);
        Button button_y_lower = new JoystickButton(lowerChassis, RobotMap.XBOX.BUTTON_Y);


        button_a_upper.whenPressed(new InstantCommand(() -> shooter.setSetpoint(SmartDashboard.getNumber("Shooter Setpoint", 3000))))
                .whenReleased(new InstantCommand(() -> shooter.stop()));

        button_a_lower.whenPressed(new RunCommand(() -> drivetrain.pidDrive(0.0),
                drivetrain)/* .withInterrupt(()->drivetrain.onTarget()) */);
        button_x_lower.whenPressed(new DriveControl(drivetrain, lowerChassis));

        button_y_lower.whenPressed(new InstantCommand(() -> feeder.feed(0.2)));

        //! Uncomment when testing LedControl 
        // button_x_upper.whenPressed(new LedControl(LedControl.LedMode.OFF));
        // button_y_upper.whenPressed(new LedControl(LedControl.LedMode.ON));
    }

    public Drivetrain getDrivetrain() {
        return drivetrain;
    }

    DifferentialDriveVoltageConstraint autoVoltageConstraint =
        new DifferentialDriveVoltageConstraint(
            new SimpleMotorFeedforward(Constants.DriveConstants.ksVolts,
                                       Constants.DriveConstants.kvVoltSecondsPerMeter,
                                       Constants.DriveConstants.kaVoltSecondsSquaredPerMeter),
                                       Constants.DriveConstants.kDriveKinematics,
                                       10);

    TrajectoryConfig config =
    new TrajectoryConfig(Constants.AutoConstants.kMaxSpeedMetersPerSecond,
                            Constants.AutoConstants.kMaxAccelerationMetersPerSecondSquared)
                            .setKinematics(Constants.DriveConstants.kDriveKinematics)
                            .addConstraint(autoVoltageConstraint);

    Trajectory testTrajectory = TrajectoryGenerator.generateTrajectory(
    new Pose2d(0, 0, new Rotation2d(0)),
    List.of(
        new Translation2d(1, 1),
        new Translation2d(2, -1)
    ),
    new Pose2d(3, 0, new Rotation2d(0)),
    config
    );

    public Command getAutonomousCommand() {    
        var transform = drivetrain.getPose().minus(testTrajectory.getInitialPose());
        testTrajectory = testTrajectory.transformBy(transform);

        RamseteController disabledRamsete = new RamseteController() {
            @Override
            public ChassisSpeeds calculate(Pose2d currentPose, Pose2d poseRef, double linearVelocityRefMeters,
                    double angularVelocityRefRadiansPerSecond) {
                return new ChassisSpeeds(linearVelocityRefMeters, 0.0, angularVelocityRefRadiansPerSecond);
            }
        };

        var feedForward = new SimpleMotorFeedforward(Constants.DriveConstants.ksVolts, Constants.DriveConstants.kvVoltSecondsPerMeter,
        Constants.DriveConstants.kaVoltSecondsSquaredPerMeter);
        //second element was:
        
        var m_leftReference = NetworkTableInstance.getDefault().getTable("troubleshooting").getEntry("left_reference");
        var m_leftMeasurement = NetworkTableInstance.getDefault().getTable("troubleshooting").getEntry("left_measurement");
        var m_rightReference = NetworkTableInstance.getDefault().getTable("troubleshooting").getEntry("right_reference");
        var m_rightMeasurement = NetworkTableInstance.getDefault().getTable("troubleshooting").getEntry("right_measurement");
        //new RamseteController(Constants.AutoConstants.kRamseteB, Constants.AutoConstants.kRamseteZeta)
        
        RamseteCommand ramseteCommand = new RamseteCommand(testTrajectory, drivetrain::getPose,
        disabledRamsete,
        feedForward,
        Constants.DriveConstants.kDriveKinematics, drivetrain::getWheelSpeeds,
        new PIDController(Constants.DriveConstants.kPDriveVel, 0, 0), new PIDController(Constants.DriveConstants.kPDriveVel, 0, 0),
        // RamseteCommand passes volts to the callback
        (leftVolts, rightVolts) ->{
            drivetrain.tankDriveVolts(leftVolts, rightVolts);

            m_leftMeasurement.setNumber(feedForward.calculate(drivetrain.getWheelSpeeds().leftMetersPerSecond));
            m_leftReference.setNumber(leftVolts);

            m_rightMeasurement.setNumber(feedForward.calculate(drivetrain.getWheelSpeeds().rightMetersPerSecond));
            m_rightReference.setNumber(-rightVolts);
        }, drivetrain);

        // Run path following command, then stop at the end.
        return ramseteCommand.andThen(() -> drivetrain.tankDriveVolts(0, 0));
    }
}
