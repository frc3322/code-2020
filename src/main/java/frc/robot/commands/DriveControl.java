/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import frc.robot.RobotContainer;
import frc.robot.subsystems.Drivetrain;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.RobotMap;

/**
 * An example command that uses an example subsystem.
 */
public class DriveControl extends CommandBase {
  @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
  private final Drivetrain drivetrain;
  private final Joystick lowerChassis;
  private double sX;
  private double sY;
  private double s;
  private double theta;
  private double newX;
  private double newY;
  private double x;
  private double y;

  public DriveControl(Drivetrain subsystem, Joystick joystick) {
    drivetrain = subsystem;
    lowerChassis = joystick;
    addRequirements(subsystem);
  }

  @Override
  public void execute() {
    x = -lowerChassis.getRawAxis(RobotMap.XBOX.STICK_R_X_AXIS);
    y = lowerChassis.getRawAxis(RobotMap.XBOX.STICK_L_Y_AXIS);

    theta = Math.atan(Math.abs(y)/Math.abs(x));

    if (Math.abs(y) > Math.abs(x)) {
        sY = 1;
        sX = 1 / Math.tan(theta);
        s = sX + sY;
    } else if (Math.abs(x) > Math.abs(y)) {
        sX = 1;
        sY = Math.tan(theta);
        s = sX + sY;
    } else if (Math.abs(x) == Math.abs(y)) {
        s = 2;
    }

    newX = x / s;
    newY = y / s;

    drivetrain.tankDrive(newX + newY, newY - newX);
    }
}
