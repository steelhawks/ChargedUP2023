package frc.robot.commands;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Drivetrain;

public class ArcadeDrive extends CommandBase {
    private Drivetrain drivetrain;
    private Joystick joystick;

    public ArcadeDrive(Drivetrain drivetrain, Joystick joystick) {
        this.drivetrain = drivetrain;
        this.joystick = joystick;
    }

    @Override
    public void initialize() {
        addRequirements(this.drivetrain);
    }

    @Override
    public void execute() {
        drivetrain.arcadeDrive(this.joystick);
    }

    @Override
    public void end(boolean interrupted) {
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}
