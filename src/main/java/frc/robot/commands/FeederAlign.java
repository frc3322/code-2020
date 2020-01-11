package frc.robot.commands;

import edu.wpi.first.wpilibj.command.Command;

import static frc.robot.Robot.drivetrain;
import static frc.robot.Robot.limelight;

public class FeederAlign extends Command {

    double speed;

    public FeederAlign() {

    }

    @Override
    protected void execute() {
        if (limelight.isTarget()) {
            speed = Math.sqrt(limelight.getDistance())/5;
        } else {
            speed = .2;
        }
        drivetrain.pidDrive(speed);
    }

    @Override
    protected boolean isFinished() {
        return drivetrain.onTarget();
    }
}