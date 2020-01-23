/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.subsystems.Drivetrain;

public class Robot extends TimedRobot {
    
    private static RobotContainer m_robotContainer;
    private static Command m_autonomousCommand;
    private Drivetrain m_drivetrain;

    public static SendableChooser<Constants.whichBot> chooseBot = new SendableChooser<>();

    @Override
    public void robotInit() {
        m_robotContainer = new RobotContainer();
        m_drivetrain = m_robotContainer.getDrivetrain();
        m_autonomousCommand = m_robotContainer.getAutonomousCommand();

        // can be updated as our default bot changes
        chooseBot.setDefaultOption("TEST_BENCH", Constants.whichBot.TEST_BENCH);
        chooseBot.addOption("P1", Constants.whichBot.P1);
        chooseBot.addOption("P2", Constants.whichBot.P2);

        SmartDashboard.putData("Robot in use", chooseBot);

        Constants.RobotMap.CAN.setCANIDs();
    }

    @Override
    public void robotPeriodic() {
        CommandScheduler.getInstance().run();
    }

    @Override
    public void disabledInit() {
    }

    @Override
    public void disabledPeriodic() {
    }

    @Override
    public void autonomousInit() {
        m_drivetrain.resetForAuto();
        if (m_autonomousCommand != null) {
            m_autonomousCommand.schedule();
        }
    }

    @Override
    public void autonomousPeriodic() {

    }

    @Override
    public void teleopInit() {
        m_drivetrain.resetForAuto();
        if (m_autonomousCommand != null) {
            m_autonomousCommand.cancel();
        }
    }

    @Override
    public void teleopPeriodic() {
    }

    @Override
    public void testInit() {
        CommandScheduler.getInstance().cancelAll();
    }

    @Override
    public void testPeriodic() {
    }
}
