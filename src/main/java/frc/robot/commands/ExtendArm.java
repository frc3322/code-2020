/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.Climber;

public class ExtendArm extends CommandBase {
  private Climber climber;
  private int RAISE = 0, CLIMB = 1;
  

  public ExtendArm(Climber climber) {
    this.climber = climber;
    addRequirements(climber);
  }

  @Override
  public void initialize() {
    climber.extendArm();
  }

  @Override
  public void execute() {
    if (climber.getEncoder(RAISE) < Constants.ClimberContants.CLIMBER_ARM_TOP_LIMIT){
        climber.raiseClimber(Constants.ClimberContants.ARM_EXTEND_SPEED);
        climber.pushWinch(Constants.ClimberContants.WINCH_EXTEND_SPEED);
    } else {
        climber.stopClimber();
        climber.stopWinch();
    }
    
  }

  @Override
  public void end(boolean interrupted) {
    climber.stopClimber();
    climber.stopWinch();
  }

  @Override
  public boolean isFinished() {
    return false;
  }
}
