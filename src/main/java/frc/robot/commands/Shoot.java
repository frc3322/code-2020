/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
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
    boolean feed;
    boolean alime;
    double initAngle;
    double initTX;

    int timer = 0;
    int timeLimit = 30;

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
        shooter.updateConstants();
        if (alime) {
            setpoint = shooter.findRPM();
            drivetrain.setUpPID(PIDMode.LIMELIGHT);
            initAngle = drivetrain.getHeading();
            initTX = drivetrain.getLimelightX();
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
        if (!feed) {
            if (alime) {
                SmartDashboard.putBoolean("Drivetrain/Limelight/Limelight on Target?", drivetrain.angleOnTarget(initTX));
                drivetrain.alime(initAngle, initTX);
                
                if (drivetrain.angleOnTarget(initTX)) {
                    timer++;
                    if (shooter.onTarget(setpoint) && timer > timeLimit) {
                        feed = true;
                    }
                }
                
            } else {
                if (shooter.onTarget(setpoint)) {
                    feed = true;
                }         
            }
        } else {
            drivetrain.drive(0,0);
            feeder.feedTop(0.8);
            feeder.feedBottom(1);
            hopper.cycle(-1, -1);
        }
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
        feeder.feedTop(0);
        feeder.feedBottom(0);
        shooter.stop();
        timer = 0;
        feed = false;
        RobotContainer.shooting = false;
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        return false;
    }
}
