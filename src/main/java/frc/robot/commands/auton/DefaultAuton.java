package frc.robot.commands.auton;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.LED.SetColor;
import frc.robot.commands.drivetrain.DriveStraight;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.LED;
import frc.util.LEDColor;

public class DefaultAuton extends SequentialCommandGroup {
    public DefaultAuton(Drivetrain drivetrain, LED LED) {
        addCommands(
            new DriveStraight(drivetrain, 0.1).withTimeout(2), 
            new DriveStraight(drivetrain, -0.1).withTimeout(2),
            new SetColor(LED, new LEDColor(255, 0, 255))
        );
    }
}