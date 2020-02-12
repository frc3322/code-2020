package frc.robot.commands.auton;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Drivetrain.PIDMode;

public class TurnToAngle extends CommandBase {

    Drivetrain drivetrain;

    private double angle;

    public TurnToAngle(Drivetrain d_subsystem, double angle) {
        drivetrain = d_subsystem;
        addRequirements(drivetrain);

        this.angle = angle;
    }

    @Override
    public void initialize(){
        drivetrain.setUpPID(PIDMode.ANGLE);
        drivetrain.resetNavX();
    }

    @Override
    public void execute() {
        drivetrain.turnToAngle(angle);
    }

    @Override
    public boolean isFinished(){
        return drivetrain.angleOnTarget(angle);
    }
}