// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Vision;
import java.util.List;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SwerveControllerCommand;
import frc.lib.util.Limelight;
import frc.lib.util.LimelightTrajectory;
import frc.robot.Constants;
import frc.robot.Robot;
import frc.robot.RobotContainer;

public class GoCone extends CommandBase {
  /** Creates a new FollowTagTrajectory. */
  public static TrajectoryConfig config = new TrajectoryConfig(
    Constants.AutoConstants.kMaxSpeedMetersPerSecond,
    Constants.AutoConstants.kMaxAccelerationMetersPerSecondSquared)
    .setKinematics(Constants.Swerve.swerveKinematics);

  public GoCone() {
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(RobotContainer.s_Swerve);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {

  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
      // RobotContainer.s_Vision.traj = TrajectoryGenerator.generateTrajectory(
      // new Pose2d(0, 0, new Rotation2d(0)),
      // List.of(),
      // new Pose2d(Limelight.getAprilTagPose()[2], Limelight.getAprilTagPose()[0] + Constants.Vision.NodeDistance, new Rotation2d(0)), config);
      //chris

      //sathya
      RobotContainer.s_Vision.traj = RobotContainer.s_Vision.sathya.generateTargetTrajectory(Robot.config);
    
      System.out.println("generated new");
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {

  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return true;
  }

  
}
