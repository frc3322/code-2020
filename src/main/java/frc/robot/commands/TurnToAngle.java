package frc.robot.commands;

import edu.wpi.first.wpilibj.command.Command;

import static frc.robot.Robot.drivetrain;

public class TurnToAngle extends Command {

    @Override
    protected void execute() {
        super.execute();
        drivetrain.pidDrive(0.0);
    }

    @Override
    protected boolean isFinished() {
        return drivetrain.onTarget();
    }

}