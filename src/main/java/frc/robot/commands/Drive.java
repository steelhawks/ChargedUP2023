package frc.robot.commands;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.CommandBase;

import frc.robot.subsystems.Drivetrain;
import frc.robot.Robot;


public class Drive extends CommandBase{
    private Drivetrain drivetrain;
    private Joystick joystick;

    public Drive(Drivetrain drivetrain, Joystick joystick){
        this.drivetrain = drivetrain;
        this.joystick = Robot.COMMAND_LINKER.joystick;

        addRequirements(drivetrain);
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
        System.out.println("Starting joystick drive command");
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        
        // double speed = joystick.getRawAxis(1) * 0.6;
        // double turn = joystick.getRawAxis(0) * 0.3;

        // double left = speed; // +turn
        // double right = speed; // -turn

        // Robot.DRIVETRAIN.LEFT_MOTOR_ONE.set(left);
        // Robot.DRIVETRAIN.RIGHT_MOTOR_ONE.set(-right);

        this.drivetrain.arcadeDrive(this.joystick);
        
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        return false;
    }
    }