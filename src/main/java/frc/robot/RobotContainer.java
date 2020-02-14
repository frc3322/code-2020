/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import frc.robot.commands.DriveControl;
import frc.robot.commands.LedControl;
import frc.robot.commands.Shoot;
import frc.robot.commands.auton.DriveDistance;
import frc.robot.commands.auton.TurnToAngle;
import frc.robot.subsystems.*;
import frc.robot.subsystems.Drivetrain.PIDMode;

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
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
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
    private final Climber climber = new Climber();
    
    private final Joystick lowerChassis = new Joystick(0);
    private final Joystick upperChassis = new Joystick(1);

    public enum auton {
        DEFAULT,
        TRENCH_FIVE,
        TRENCH_SIX,
        MIDDLE_FIVE;
    }

    private Command shoot = new Shoot(drivetrain, shooter, feeder, hopper, true);
    private Command testDriveDistance = new DriveDistance(drivetrain, 2000);
    private Command testTurnToAngle = new TurnToAngle(drivetrain, 180);

    public static boolean shot = false;
    public static boolean intook = false;
    
    public RobotContainer() {
        
        configureButtonBindings();

        drivetrain.setDefaultCommand(new DriveControl(drivetrain, lowerChassis));

        feeder.putInitialDash();
        shooter.putInitialDash();
        hopper.putInitialDash();
        climber.putInitialDash();
        
        
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

        //upper
        button_a_upper.whenPressed(new InstantCommand(() -> shooter.setSetpoint(shooter.findRPM())))
                        .whenReleased(new InstantCommand(() -> shooter.stop()));

        button_b_upper.whenPressed(new InstantCommand(() -> testDriveDistance.schedule()))
                        .whenReleased(new InstantCommand(() -> testDriveDistance.cancel()));
                
        button_x_upper.whenPressed(new InstantCommand(() -> testTurnToAngle.schedule()))
                        .whenReleased(new InstantCommand(() -> testTurnToAngle.cancel()));

        //lower
        bumper_right_lower.whenPressed(new InstantCommand(() -> intake.begin()).alongWith(new InstantCommand(() -> hopper.cycle(-.5, -0.5))))
                            .whenReleased(new InstantCommand(() -> intake.end()).alongWith(new InstantCommand(() -> hopper.stop())));

        button_a_lower.whenPressed(new InstantCommand(() -> shoot.schedule()))
                        .whenReleased(new InstantCommand(() -> shoot.cancel()));

        button_x_lower.whenPressed(new InstantCommand(() -> intake.outtakeBegin()))
                        .whenReleased(new InstantCommand(() -> intake.end()));

    }

    public Drivetrain getDrivetrain() {
        return drivetrain;
    }
    
    public Command getAutonomousCommand(auton selected) {
        //Set up auton trajectory
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

        switch (selected) {
            case MIDDLE_FIVE: 
                return  new DriveDistance(drivetrain, 2000)
                            .alongWith(new InstantCommand(() -> intake.begin()))
                        .andThen(new RunCommand(() -> drivetrain.delay()).withTimeout(1)) /*possibly unnecessary depending on intake speed*/
                        .andThen(new DriveDistance(drivetrain, -500))
                            .alongWith(new InstantCommand(() -> intake.end()))
                        .andThen(new TurnToAngle(drivetrain, 170))
                        .andThen(shoot).withTimeout(5);
            case TRENCH_FIVE:
                return  new DriveDistance(drivetrain, 2000)
                            .alongWith(new InstantCommand(() -> intake.begin()))
                        .andThen(new DriveDistance(drivetrain, -1000))
                            .alongWith(new InstantCommand(() -> intake.begin()))
                        .andThen(new TurnToAngle(drivetrain, -170))
                        .andThen(shoot).withTimeout(5);
            case TRENCH_SIX:
                return  shoot.withTimeout(3)
                        .andThen(new TurnToAngle(drivetrain, 180))
                        .andThen(new DriveDistance(drivetrain, 2000)).alongWith(new InstantCommand(() -> intake.begin()))
                        .andThen(new TurnToAngle(drivetrain, 180)).alongWith(new InstantCommand(() -> intake.end()))
                        .andThen(shoot.withTimeout(4));
            case DEFAULT:
                return  shoot.withTimeout(3)
                        .andThen(new DriveDistance(drivetrain, -300));
            default:
                return  shoot.withTimeout(3)
                        .andThen(new DriveDistance(drivetrain, -300));
        }

        // Run path following command, then stop at the end.
        //return ramseteCommand.andThen(() -> drivetrain.tankDriveVolts(0, 0));
    }
}
