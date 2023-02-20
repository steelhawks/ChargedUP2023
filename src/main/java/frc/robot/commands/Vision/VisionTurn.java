package frc.robot.commands.Vision;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.lib.util.Limelight;
import frc.robot.Constants;
import frc.robot.RobotContainer;
import frc.robot.subsystems.Vision;

public class VisionTurn extends CommandBase {

    public VisionTurn() {
        addRequirements(RobotContainer.s_Swerve);
        addRequirements(RobotContainer.s_Vision);
    }

    @Override
    public void initialize() {
    
    }

    @Override
    public void execute() {
        RobotContainer.s_Vision.turnRobot();
        
    }

    @Override
    public void end(boolean interrupted) {
        System.out.println("turnt");
        RobotContainer.s_Swerve.stop();
    }

    @Override
    public boolean isFinished() {
        return Limelight.hasValidTarget() && Math.abs(Limelight.getTagYaw()) < 1;
    }
}
