/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import frc.robot.commands.DriveControl;
import frc.robot.subsystems.*;

import java.util.function.BooleanSupplier;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.button.*;

/**
 * This class is where the bulk of the robot should be declared.  Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls).  Instead, the structure of the robot
 * (including subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  private final Drivetrain drivetrain = new Drivetrain();
  private final Shooter shooter = new Shooter();

  private Joystick lowerChassis = new Joystick(0);
  private Joystick upperChassis = new Joystick(1);




  public RobotContainer() {
    configureButtonBindings();

    drivetrain.setDefaultCommand(new DriveControl(drivetrain, ()->lowerChassis.getX(), ()->lowerChassis.getY()));
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

    button_a_upper
      .whenPressed(new InstantCommand(()->shooter.setSetpoint(3000)))
      .whenReleased(new InstantCommand(()->shooter.setSetpoint(0)));

    button_a_lower.whenPressed(new RunCommand(()->drivetrain.pidDrive(0.0), drivetrain).withInterrupt(()->drivetrain.onTarget()));
  }

  // public Command getAutonomousCommand() {

  // }
}
