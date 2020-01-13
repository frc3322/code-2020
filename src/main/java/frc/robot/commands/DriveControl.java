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

import edu.wpi.first.wpilibj2.command.CommandBase;

/**
 * An example command that uses an example subsystem.
 */
public class DriveControl extends CommandBase {
  @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
  private final Drivetrain drivetrain;

  private DoubleSupplier m_y;
  private DoubleSupplier m_x;
  private double sX;
  private double sY;
  private double s;
  private double theta;
  private double newX;
  private double newY;
  private double x = m_x.getAsDouble();
  private double y = m_y.getAsDouble();

  public DriveControl(Drivetrain subsystem, DoubleSupplier x, DoubleSupplier y) {
    drivetrain = subsystem;
    m_y = y;
    m_x = x;
    addRequirements(subsystem);
  }

  @Override
    public void execute() {
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
