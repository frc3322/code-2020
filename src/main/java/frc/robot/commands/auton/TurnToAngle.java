package frc.robot.commands.auton;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Drivetrain;

public class TurnToAngle extends CommandBase {

    Drivetrain drivetrain;

    private double angle;

    public TurnToAngle(Drivetrain d_subsystem, double angle) {
        drivetrain = d_subsystem;
        addRequirements(drivetrain);

        this.angle = angle;
    }

    @Override
    public void execute() {
        drivetrain.turnToAngle(angle);
    }
}