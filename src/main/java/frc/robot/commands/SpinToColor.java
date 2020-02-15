/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Robot;
import frc.robot.RobotContainer;
import frc.robot.subsystems.Spinner;

public class SpinToColor extends CommandBase {
  Spinner spinner;
  char desiredColor;
  public SpinToColor(Spinner spinner) {
    switch (DriverStation.getInstance().getGameSpecificMessage().charAt(0)) {
      case 'R':
        desiredColor = 'B';
        break;
      case 'G':
        desiredColor = 'Y';
        break;
      case 'B':
        desiredColor = 'R';
        break;
      case 'Y':
        desiredColor = 'G';
        break;
      default:
        desiredColor = 'U';
        break;
    }
    this.spinner = spinner;
    addRequirements(spinner);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    spinner.spin(0.2);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    spinner.stop();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return spinner.getColor() == desiredColor;
  }
}
