/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Climber;

public class ClimbControl extends CommandBase {
  public enum climbMode {
    TOGGLE,
    RAISE,
    LOWER,
    CLIMB,
    DECLIMB
  }
  private climbMode mode;
  private Climber climber;

  public ClimbControl(Climber climber, climbMode mode) {
    this.climber = climber;
    this.mode = mode;
    addRequirements(climber);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    if (mode == climbMode.TOGGLE) {
      climber.toggle();
    }
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    // TODO: adjust motor values
    switch(mode) {
      case TOGGLE:
        break;
      case RAISE:
        climber.raiseClimber(0.4);
        break;
      case LOWER:
        climber.lowerClimber(0.4);
        break;
      case CLIMB:
        climber.pullWinch(0.4);
        break;
      case DECLIMB:
        climber.pushWinch(0.4);
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
