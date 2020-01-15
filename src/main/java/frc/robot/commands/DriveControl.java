package frc.robot.commands;

import java.util.HashSet;
import java.util.Set;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Subsystem;
import frc.robot.RobotMap;

import static frc.robot.Robot.drivetrain;
import static frc.robot.Robot.oi;

public class DriveControl implements Command {

    private final int SPEED_AXIS;
    private final int ROTATION_AXIS;

    private double rotationModifier;

    private double turnDirection;

    private double speed;

    private double turn;

    private double y;
    private double x;
    private double sX;
    private double sY;
    private double s;
    private double theta;
    private double newX;
    private double newY;

    public DriveControl() {
        

        SPEED_AXIS = RobotMap.XBOX.STICK_L_Y_AXIS;
        ROTATION_AXIS = RobotMap.XBOX.STICK_R_X_AXIS;
    }

    @Override
    public void execute() {
        speed = oi.getLowerChassis().getRawAxis(SPEED_AXIS);

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
    public Set<Subsystem> getRequirements() {
        Set<Subsystem> requSet = new HashSet<Subsystem>();
        requSet.add(drivetrain);
        return requSet;
        
    }

    @Override
    public boolean isFinished() {
        return false;
    }

}