package frc.robot.commands.auton;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.drivetrain.DriveStraight;
import frc.robot.subsystems.Drivetrain;

public class DefaultAuton extends SequentialCommandGroup {
    public DefaultAuton(Drivetrain drivetrain) {
        addCommands(
            new DriveStraight(drivetrain, 0.1).withTimeout(3), 
            new DriveStraight(drivetrain, -0.1).withTimeout(3)
        );
    }
}