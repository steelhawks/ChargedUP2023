package frc.robot;

import frc.robot.controllers.Stick;
import frc.robot.subsystems.Drivetrain;

public class RobotContainer {
    public final Stick stick;
    public final Drivetrain drivetrain;

    public RobotContainer() {
        this.stick = new Stick();
        this.drivetrain = new Drivetrain();
    }
}
