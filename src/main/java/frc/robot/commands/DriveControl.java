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
        x = lowerChassis.getRawAxis(RobotMap.XBOX.STICK_R_X_AXIS);
        y = -lowerChassis.getRawAxis(RobotMap.XBOX.STICK_L_Y_AXIS);

        SmartDashboard.putNumber("X", x);
        SmartDashboard.putNumber("Y", y);

        double m_x = Math.abs(x);

        if(x >= 0){
            direction = 1;
        } else {
            direction = -1;
        }

        outputTurn = direction * ((a*Math.pow(m_x, pow1)) + (b*Math.pow(m_x, pow2) + (c * m_x)));

        if(Math.abs(y) < 0.37){
            drivetrain.drive(y, outputTurn);
        } else {
            drivetrain.drive(y, x);
        }

    }
}
