/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Intake;

public class IntakeControl extends CommandBase {
  public enum intakeMode {
    INTAKE,
    OUTTAKE,
    STOP,
    TOGGLE
  }
  private intakeMode mode;
  private Intake intake;
  public IntakeControl(Intake intake, intakeMode mode) {
    this.intake = intake;
    this.mode = mode;
    addRequirements(intake);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    if (mode == intakeMode.TOGGLE) {
      intake.toggle();
    }
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    //TODO: this code is probably unnecessary, we can use instant commands to directly call these methods in RobotContainer.java
    switch (mode) {
      case INTAKE:
        intake.intakeStart();
        break;
      case OUTTAKE:
        intake.outtake();
        break;
      case STOP: 
        intake.stop();
        break;
      case TOGGLE:
        break;
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return true;
  }
}