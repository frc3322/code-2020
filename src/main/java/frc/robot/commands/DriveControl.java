package frc.robot.commands;

import edu.wpi.first.wpilibj.command.Command;
import frc.robot.RobotMap;

import static frc.robot.Robot.drivetrain;
import static frc.robot.Robot.oi;

public class DriveControl extends Command {

    private final int SPEED_AXIS;
    private final int ROTATION_AXIS;

    private double y;
    private double x;
    private double sX;
    private double sY;
    private double s;
    private double theta;
    private double newX;
    private double newY;

    public DriveControl() {
        requires(drivetrain);

        SPEED_AXIS = RobotMap.XBOX.STICK_L_Y_AXIS;
        ROTATION_AXIS = RobotMap.XBOX.STICK_R_X_AXIS;
    }

    @Override
    protected void execute() {
        x = oi.getLowerChassis().getRawAxis(ROTATION_AXIS);
        y = oi.getLowerChassis().getRawAxis(SPEED_AXIS);
        
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

    @Override
    protected boolean isFinished() {
        return false;
    }

}