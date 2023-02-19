package frc.robot.commands.Vision;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.lib.util.Limelight;
import frc.robot.Constants;
import frc.robot.RobotContainer;
import frc.robot.subsystems.Vision;

public class VisionCrabTag1 extends CommandBase {

    public VisionCrabTag1() {
        addRequirements(RobotContainer.s_Swerve);
        addRequirements(RobotContainer.s_Vision);
    }

    @Override
    public void initialize() {
        Limelight.setPipeline(Vision.ID1_PIPELINE);
    
    }

    @Override
    public void execute() {

        RobotContainer.s_Swerve.drive(new Translation2d(0, 0.8), 0, true, false);

    }

    @Override
    public void end(boolean interrupted) {
        System.out.println("tag1");
        RobotContainer.s_Swerve.stop();
    }

    @Override
    public boolean isFinished() {
        return Math.abs(Limelight.getXOffset()) < 1 && Limelight.getXOffset() != 0;
    }
}
