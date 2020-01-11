/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.buttons.*;
import frc.robot.commands.*;
/**
 * This class is the glue that binds the controls on the physical operator
 * interface to the commands and command groups that allow control of the robot.
 */
public class OI {
    private Joystick lowerChassis = new Joystick(0);
    private Joystick upperChassis = new Joystick(1);
    
    private Button button_a_upper = new JoystickButton(upperChassis, RobotMap.XBOX.BUTTON_A);
    private Button button_b_upper = new JoystickButton(upperChassis, RobotMap.XBOX.BUTTON_B);
    private Button button_x_upper = new JoystickButton(upperChassis, RobotMap.XBOX.BUTTON_X);
    private Button button_y_upper = new JoystickButton(upperChassis, RobotMap.XBOX.BUTTON_Y);
    private Button bumper_left_upper = new JoystickButton(upperChassis, RobotMap.XBOX.BUMPER_LEFT);
    private Button bumper_right_upper = new JoystickButton(upperChassis, RobotMap.XBOX.BUMPER_RIGHT);
    private Button button_back_upper = new JoystickButton(upperChassis, RobotMap.XBOX.BUTTON_BACK);
    private Button button_start_upper = new JoystickButton(upperChassis, RobotMap.XBOX.BUTTON_START);

    private Button bumper_left_lower = new JoystickButton(lowerChassis, RobotMap.XBOX.BUMPER_LEFT);
    private Button bumper_right_lower = new JoystickButton(lowerChassis, RobotMap.XBOX.BUMPER_RIGHT);
    private Button button_back_lower = new JoystickButton(lowerChassis, RobotMap.XBOX.BUTTON_BACK);
    private Button button_start_lower = new JoystickButton(lowerChassis, RobotMap.XBOX.BUTTON_START);
    private Button left_stick_lower = new JoystickButton(lowerChassis, RobotMap.XBOX.STICK_LEFT);
    private Button right_stick_lower = new JoystickButton(lowerChassis, RobotMap.XBOX.STICK_RIGHT);
    private Button button_a_lower = new JoystickButton(lowerChassis, RobotMap.XBOX.BUTTON_A);
    private Button button_x_lower = new JoystickButton(lowerChassis, RobotMap.XBOX.BUTTON_X);

    public OI() {
        button_a_lower.whenPressed(new TurnToAngle());
    }

    public Joystick getUpperChassis(){
        return upperChassis;
    }

    public Joystick getLowerChassis(){
        return lowerChassis;
    }
}
