package frc.robot.commands.drivetrain;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Drivetrain;

public class DriveDistance extends CommandBase {
    private Drivetrain drivetrain;
    private double distance;

    public DriveDistance(Drivetrain drivetrain, double distance) {
        this.drivetrain = drivetrain;
        this.distance = distance;

        addRequirements(drivetrain);
    }

    @Override
    public void initialize() {
        this.drivetrain.zeroEncoders();
    }

    @Override
    public void execute() {
        this.drivetrain.driveDistance(this.distance);
    }

    @Override
    public boolean isFinished() {
        double error = this.drivetrain.getPIDControlError(this.distance);
        if (0.12 > error && error > -0.12) { // momentum will carry forward, even in brake mode
            System.out.println("FINISHED");
            return true;
        }
        return false;
    }
}
