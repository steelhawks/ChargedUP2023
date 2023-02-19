package frc.robot.commands.drivetrain;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Drivetrain;

public class ManualDrive extends CommandBase {
    private Drivetrain drivetrain;
    private double rightMotorSpeed;
    private double leftMotorSpeed;

    public ManualDrive(Drivetrain drivetrain, double rightMotorSpeed, double leftMotorSpeed) {
        this.drivetrain = drivetrain;
        this.rightMotorSpeed = rightMotorSpeed;
        this.leftMotorSpeed = leftMotorSpeed;

        addRequirements(this.drivetrain);
    }

    @Override
    public void execute() {
        this.drivetrain.manualDrive(this.rightMotorSpeed, this.leftMotorSpeed);
    }
}
