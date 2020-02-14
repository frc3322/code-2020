/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.command.Subsystem;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.RobotContainer;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Feeder;
import frc.robot.subsystems.Hopper;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.Drivetrain.PIDMode;

public class Shoot extends CommandBase {
    Drivetrain drivetrain;
    Shooter shooter;
    Feeder feeder;
    Hopper hopper;
    Boolean feed;
    boolean alime;

    double setpoint;

    public Shoot(Drivetrain drivetrain, Shooter shooter, Feeder feeder, Hopper hopper, boolean alime) {
        this.drivetrain = drivetrain;
        addRequirements(shooter);

        this.shooter = shooter;
        this.feeder = feeder;
        this.hopper = hopper;
        feed = false;
        this.alime = alime;
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
        if (alime) {
            setpoint = shooter.findRPM();
            drivetrain.setUpPID(PIDMode.LIMELIGHT);
        } else {
            setpoint = 3300;
        }

        shooter.setSetpoint(setpoint);
        feed = false;

        RobotContainer.shooting = true;
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        if (shooter.onTarget(setpoint)) {
            feed = true;
        }

        if (alime) {
            if (!drivetrain.limelightOnTarget()) {
                drivetrain.pidDrive(0.0);
            } else if (drivetrain.limelightOnTarget() && feed) {
                drivetrain.drive(0,0);
                feeder.feedTop(0.8);
                feeder.feedBottom(1);
                hopper.cycle(-1, -1);
            } 
        } else {
            if (feed) {
                drivetrain.drive(0,0);
                feeder.feedTop(0.8);
                feeder.feedBottom(1);
                hopper.cycle(-1, -1);
            }
        }
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
        feeder.feedTop(0);
        feeder.feedBottom(0);
        shooter.stop();
        feed = false;
        RobotContainer.shooting = false;
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        return false;
    }
}
