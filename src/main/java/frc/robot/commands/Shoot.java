/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj.command.Subsystem;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.RobotContainer;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Feeder;
import frc.robot.subsystems.Hopper;
import frc.robot.subsystems.Shooter;

public class Shoot extends CommandBase {
    Drivetrain drivetrain;
    Shooter shooter;
    Feeder feeder;
    Hopper hopper;

    double setpoint;

    public Shoot(Drivetrain d_subsystem, Shooter s_subsystem, Feeder f_subsystem, Hopper h_subsystem) {
        drivetrain = d_subsystem;
        addRequirements(s_subsystem);

        shooter = s_subsystem;
        feeder = f_subsystem;
        hopper = h_subsystem;
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
        setpoint = shooter.findRPM();
        shooter.setSetpoint(setpoint);

    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        if (!drivetrain.onTarget()) {
            drivetrain.pidDrive(0.0);
        } else if (drivetrain.onTarget() && shooter.onTarget(setpoint)) {
            feeder.feedTop(.3);
            feeder.feedBottom(.3);
            hopper.cycle(.3, .3);
        }
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
        feeder.feedTop(0);
        feeder.feedBottom(0);
        shooter.stop();
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        return !shooter.hasTarget();
    }
}
