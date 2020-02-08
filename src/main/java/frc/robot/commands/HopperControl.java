/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Hopper;

public class HopperControl extends CommandBase {
  /**
   * Creates a new HopperControl.
   */
  private final Hopper hopper;
  public enum hopperMode {
    STOP,
    CYCLE,
  }
  private hopperMode mode;

  public HopperControl(Hopper hopper, hopperMode mode) {
    this.hopper = hopper;
    this.mode = mode;
    addRequirements(hopper);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    //TODO: this code is probably unnecessary, we can use instant commands to directly call these methods in RobotContainer.java
    switch (mode) {
      case CYCLE:
        hopper.cycle(0, 0);
        break;
      case STOP:
        hopper.stop();
        break;
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    hopper.stop();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
