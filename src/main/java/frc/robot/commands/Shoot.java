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
    double angleSetpoint;

    int limeTimer = 0;
    int limeTimeLimit = 10; //50 per second

    int feedTimer = 0;
    int feedTimeLimit = 10;

    double shootSetpoint;

    boolean runTimer;

    public Shoot(Drivetrain drivetrain, Shooter shooter, Feeder feeder, Hopper hopper, boolean alime) {
        this.drivetrain = drivetrain;
        addRequirements(shooter);

        this.shooter = shooter;
        this.feeder = feeder;
        this.hopper = hopper;
        feed = false;
        this.alime = alime;
        runTimer = false;
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
        shooter.updateConstants();
        if (alime) {
            shootSetpoint = shooter.findRPM();
            //setpoint = SmartDashboard.getNumber("Shooter/ShootPID/Shooter Setpoint", 3000);
            drivetrain.setUpPID(PIDMode.LIMELIGHT);
            initAngle = drivetrain.getHeading();
            initTX = drivetrain.getLimelightX();
            angleSetpoint = initAngle - initTX;
        } else {
            shootSetpoint = SmartDashboard.getNumber("Shooter/ShootPID/Shooter Setpoint", 3000);
        }

        shooter.setSetpoint(shootSetpoint);
        feed = false;
        runTimer = false;
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        if (!feed) {
            if (alime) {
                SmartDashboard.putBoolean("Shooter/ShootPID/Shooter On Target", shooter.onTarget(shootSetpoint));
                SmartDashboard.putBoolean("Drivetrain/Limelight/Limelight on Target?", drivetrain.alimeOnTarget());
                //SmartDashboard.putBoolean("Drivetrain/Limelight/Limelight on Target?", shooter.onTarget(shootSetpoint));
                drivetrain.alime(initAngle, initTX);

                if (drivetrain.alimeOnTarget()) {
                    limeTimer++;
                    if(limeTimer > limeTimeLimit){
                        drivetrain.drive(0,0);
                        if (shooter.onTarget(shootSetpoint)) {
                            feedTimer++;
                            if(feedTimer > feedTimeLimit){
                                feed = true;
                            }
                        } else {
                            feedTimer = 0;
                        }
                    }
                } else {
                    limeTimer = 0;
                }
                
            } else {
                if (shooter.onTarget(shootSetpoint)) {
                    feedTimer++;
                    if(feedTimer > feedTimeLimit){
                        feed = true;
                    }
                }         
            }
        } else {
            drivetrain.drive(0,0);
            feeder.feedTop(0.4);
            feeder.feedBottom(0.6);
            hopper.cycle(-0.5, -0.5);
        }
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
        feeder.stop();
        shooter.stop();
        limeTimer = 0;
        feedTimer = 0;
        feed = false;
        runTimer = false;
        hopper.cycle(0, 0);
        feeder.setShotSinceFed(true);
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        return false;
    }
}
