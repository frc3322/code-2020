/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import frc.robot.subsystems.Drivetrain;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.RobotMap;

/**
 * An example command that uses an example subsystem.
 */
public class DriveControl extends CommandBase {
    @SuppressWarnings({ "PMD.UnusedPrivateField", "PMD.SingularField" })
    private final Drivetrain drivetrain;
    private final Joystick lowerChassis;
    // private double sX;
    // private double sY;
    // private double s;
    // private double theta;
    // private double newX;
    // private double newY;
    private double x;
    private double y;
    private double outputTurn;
    private double a = -67551.4;
    private double b = 67549.5;
    private double c = 2.8809;
    private double pow1 = 1.66736;
    private double pow2 = 1.6674;
    private double direction = 1.0;

    public DriveControl(Drivetrain subsystem, Joystick joystick) {
        drivetrain = subsystem;
        lowerChassis = joystick;
        addRequirements(subsystem);
    }

    @Override
    public void execute() {
        double rawX = -lowerChassis.getRawAxis(RobotMap.XBOX.STICK_R_X_AXIS);
        double y = lowerChassis.getRawAxis(RobotMap.XBOX.STICK_L_Y_AXIS);


        if(rawX >= 0){
            direction = 1;
        } else {
            direction = -1;
        }



        double x = Math.abs(rawX);

        outputTurn = direction * ((a*Math.pow(x, pow1)) + (b*Math.pow(x, pow2) + (c * x)));
        SmartDashboard.putNumber("Turn", outputTurn);
        if(Math.abs(y) < 0.15){
            drivetrain.drive(y, outputTurn);
        } else {
            drivetrain.drive(y, rawX);
        }
            
        // SmartDashboard.putNumber("x", x);
        // SmartDashboard.putNumber("y", y);

        // theta = Math.atan(Math.abs(y) / Math.abs(x));

        // if (Math.abs(y) > Math.abs(x)) {
        //     sY = 1;
        //     sX = 1 / Math.tan(theta);
        //     s = sX + sY;
        // } else if (Math.abs(x) > Math.abs(y)) {
        //     sX = 1;
        //     sY = Math.tan(theta);
        //     s = sX + sY;
        // } else if (Math.abs(x) == Math.abs(y)) {
        //     s = 2;
        // }

        // newX = x / s;
        // newY = y / s;

        // double left = newX + newY;
        // double right = newY - newX;

        // if (Math.abs(x) > 0.15) {
        //     drivetrain.tankDrive(left, right);
        // } else {
        //     drivetrain.drive(left, 0);
        // }

    }
}
