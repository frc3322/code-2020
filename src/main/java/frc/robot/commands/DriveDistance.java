package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Drivetrain;

public class DriveDistance extends CommandBase {

    Drivetrain drivetrain;

    public DriveDistance(Drivetrain d_subsystem, double distance, double wait) {
        drivetrain = d_subsystem;
        addRequirements(drivetrain);
    }
}