package frc.robot.commands.auton;

import java.util.ArrayList;
import java.util.Arrays;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.LED.Wave;
import frc.robot.commands.drivetrain.ManualDrive;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.LED;
import frc.util.LEDColor;

public class DefaultAuton extends SequentialCommandGroup {
    public DefaultAuton(Drivetrain drivetrain, LED LED) {
        addCommands(
            new ManualDrive(drivetrain, 0.1, 0.1).withTimeout(3), 
            new ManualDrive(drivetrain, -0.1, -0.1).withTimeout(1),
            new Wave(LED, 20, new ArrayList<LEDColor>(Arrays.asList(new LEDColor(255, 0, 255)))).withTimeout(3),
            new ManualDrive(drivetrain, 0.1, -0.1).withTimeout(3)
        );
    }
}