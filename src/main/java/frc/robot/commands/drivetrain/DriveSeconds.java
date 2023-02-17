package frc.robot.commands.drivetrain;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Drivetrain;

public class DriveSeconds extends CommandBase {
    private Drivetrain drivetrain;
    private double speed;

    public DriveSeconds(Drivetrain drivetrain, double speed) {
        this.drivetrain = drivetrain;
        this.speed = speed;

        addRequirements(this.drivetrain);
    }

    @Override
    public void execute() {
        this.drivetrain.driveStraight(this.speed);
    }
}
