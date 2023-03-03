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
    this.drivetrain.resetErrorSum();
  }

  @Override
  public void execute() {
    this.drivetrain.driveDistance(this.distance);
  }

  @Override
  public boolean isFinished() {
    double error = this.drivetrain.getPIDControlError();
    if (0.01 > error && error > -0.01) return true;
    return false;
  }
}
