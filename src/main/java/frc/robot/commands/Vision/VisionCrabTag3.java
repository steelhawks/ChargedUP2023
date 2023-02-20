package frc.robot.commands.Vision;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.lib.util.Limelight;
import frc.robot.Constants;
import frc.robot.RobotContainer;
import frc.robot.subsystems.Vision;

public class VisionCrabTag3 extends CommandBase {

    public VisionCrabTag3() {
        addRequirements(RobotContainer.s_Swerve);
        addRequirements(RobotContainer.s_Vision);
    }

    @Override
    public void initialize() {
    
    }

    @Override
    public void execute() {
        double y_vel = (Math.sin((Math.PI / Constants.Vision.FiftyFour) * Limelight.getXOffset()));
        RobotContainer.s_Swerve.drive(new Translation2d(0, y_vel), 0, true, false);

    }

    @Override
    public void end(boolean interrupted) {
        System.out.println("tag3");
        RobotContainer.s_Swerve.stop();
    }

    @Override
    public boolean isFinished() {
        return Math.abs(Limelight.getXOffset()) < 15 && Limelight.getXOffset() != 0;
    }
}
