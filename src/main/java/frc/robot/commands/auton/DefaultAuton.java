package frc.robot.commands.auton;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.drivetrain.DriveSeconds;
import frc.robot.subsystems.Drivetrain;

public class DefaultAuton extends SequentialCommandGroup {
    public DefaultAuton(Drivetrain drivetrain) {
        addRequirements(drivetrain);
        System.out.println("Called Default Auton");
        // Drive straight for 5 seconds at 0.5 motor speed
        addCommands(new DriveSeconds(drivetrain, 0.2).withTimeout(2));
    }
}