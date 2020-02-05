/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import frc.robot.LedData;
import edu.wpi.first.wpilibj2.command.InstantCommand;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/latest/docs/software/commandbased/convenience-features.html
public class LedControl extends InstantCommand {
  public enum LedMode {
    OFF,
    ON
  }

  private LedMode state;

  public LedControl(LedMode state) {
    this.state = state;
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    switch (this.state) {
      case OFF:
        LedData.getInstance().startPattern(0);
        break;
      case ON:
        LedData.getInstance().startPattern(1);
        break;
    }
  }
}
