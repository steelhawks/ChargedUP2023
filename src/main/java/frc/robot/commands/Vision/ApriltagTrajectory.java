package frc.robot.commands.Vision;

import java.util.List;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.lib.util.Limelight;
import frc.robot.Constants;
import frc.robot.RobotContainer;

public class ApriltagTrajectory extends CommandBase {
    public static Trajectory test;
    public static Pose2d target = Limelight.getApriltagPose();

    public ApriltagTrajectory() {
        addRequirements(RobotContainer.s_Swerve);
        addRequirements(RobotContainer.s_Vision);
    }

    @Override
    public void initialize() {
        TrajectoryConfig config = new TrajectoryConfig(
            Constants.AutoConstants.kMaxSpeedMetersPerSecond,
            Constants.AutoConstants.kMaxAccelerationMetersPerSecondSquared)
            .setKinematics(Constants.Swerve.swerveKinematics);

        test = TrajectoryGenerator.generateTrajectory(
            // Start at the origin facing the +X direction
            Limelight.getRobotSpace(),
            // Pass through these two interior waypoints, making an 's' curve path
            List.of(new Translation2d(1, 1), new Translation2d(2, -1)),
            // End 3 meters straight ahead of where we started, facing forward
            new Pose2d(3, 0, new Rotation2d(0)),
            config);       
    }

    @Override
    public void execute() {
        RobotContainer.s_Vision.testTrajectory(test);
    }

    @Override
    public void end(boolean interrupted) {
        RobotContainer.s_Swerve.stop();
    }

    @Override
    public boolean isFinished() {
        return Limelight.getRobotSpace() == target;
    }
}
