/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.ColorSensorV3;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Robot;

import static frc.robot.Robot.m_can;

public class Spinner extends SubsystemBase {
  private CANSparkMax motor;
  private DoubleSolenoid SpinnerExtender;
  private ColorSensorV3 m_colorSensor;

  public Spinner() {
    motor = new CANSparkMax(m_can.SPINNER_MOTOR, MotorType.kBrushless);
    m_colorSensor = new ColorSensorV3(Robot.i2cPort);
  }

  @Override
  public void periodic() {
    Color detectedColor = m_colorSensor.getColor();

    double IR = m_colorSensor.getIR();
    /*
    SmartDashboard.putNumber("Red", detectedColor.red);
    SmartDashboard.putNumber("Green", detectedColor.green);
    SmartDashboard.putNumber("Blue", detectedColor.blue);
    SmartDashboard.putNumber("IR", IR);
    */

    // rough thresholds for color detection
    if (detectedColor.red > 0.5) {
      SmartDashboard.putString("Color", "Red");
    } else if (detectedColor.green > 0.5) {
      SmartDashboard.putString("Color", "Green");
    } else if (detectedColor.blue > 0.3) {
      SmartDashboard.putString("Color", "Blue");
    } else if (detectedColor.red > 0.35 && detectedColor.green > 0.35) {
      SmartDashboard.putString("Color", "Yellow");
    } else {
      SmartDashboard.putString("Color", "Unsure");
    }
  }
}
