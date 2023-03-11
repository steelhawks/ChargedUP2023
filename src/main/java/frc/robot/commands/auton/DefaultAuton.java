package frc.robot.commands.auton;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.LED.Wave;
import frc.robot.commands.drivetrain.DriveDistance;
import frc.robot.commands.drivetrain.ManualDrive;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.LED;
import frc.util.LEDColor;

public class DefaultAuton extends SequentialCommandGroup {

  public DefaultAuton(Drivetrain drivetrain, LED LED) {
    addCommands(
      new DriveDistance(drivetrain, 10),
      new ManualDrive(drivetrain, 0.1, 0.1).withTimeout(3),
      new Wave(
        LED,
        20,
        new LEDColor[] {
          new LEDColor(255, 0, 0),
          new LEDColor(0, 255, 0),
          new LEDColor(0, 0, 255),
        }
      )
        .withTimeout(3),
      new ManualDrive(drivetrain, 0.1, -0.1).withTimeout(3)
    );
  }
}
