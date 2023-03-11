package frc.robot.commands.drivetrain;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Drivetrain;

public class ArcadeDrive extends CommandBase {

  private Drivetrain drivetrain;
  private Joystick joystick;

  public ArcadeDrive(Drivetrain drivetrain, Joystick joystick) {
    this.drivetrain = drivetrain;
    this.joystick = joystick;

    addRequirements(drivetrain);
  }

  public void initialize() {
    this.drivetrain.arcadeDrive(this.joystick);
  }
}
