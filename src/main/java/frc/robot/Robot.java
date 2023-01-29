// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.io.IOException;
import java.net.ConnectException;
import java.nio.file.Path;
import java.util.ArrayList;
import java.util.List;

import javax.swing.plaf.metal.MetalBorders.ScrollPaneBorder;

import com.pathplanner.lib.PathConstraints;
import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.commands.PPSwerveControllerCommand;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.math.trajectory.TrajectoryUtil;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.SwerveControllerCommand;

/**
 * The VM is configured to automatically run this class, and to call the functions corresponding to
 * each mode, as described in the TimedRobot documentation. If you change the name of this class or
 * the package after creating this project, you must also update the build.gradle file in the
 * project.
 */
public class Robot extends TimedRobot {
  public static CTREConfigs ctreConfigs;

  private Command m_autonomousCommand;
  private SendableChooser<Integer> moduleChooser;
  private SendableChooser<Integer> moduleCumulativeChooser;
  private SendableChooser<Integer> pathChooser;

  private RobotContainer m_robotContainer;

  private TrajectoryConfig config;
  private List<Trajectory> trajectories = new ArrayList<>();
  
  /**
   * This function is run when the robot is first started up and should be used for any
   * initialization code.
   */

   public Command followTrajectoryCommand(PathPlannerTrajectory traj, boolean isFirstPath) {
    return new SequentialCommandGroup(
         new InstantCommand(() -> {
           // Reset odometry for the first path you run during auto
           if(isFirstPath){
               this.m_robotContainer.s_Swerve.resetOdometry(traj.getInitialHolonomicPose());
           }
         }),
         new PPSwerveControllerCommand(
             traj, 
             m_robotContainer.s_Swerve::getPose,
             Constants.Swerve.swerveKinematics, // SwerveDriveKinematics
             new PIDController(Constants.AutoConstants.kPController, 0, Constants.AutoConstants.kDController), // X controller. Tune these values for your robot. Leaving them 0 will only use feedforwards.
             new PIDController(Constants.AutoConstants.kPController, 0, Constants.AutoConstants.kDController), // Y controller (usually the same values as X controller)
             new PIDController(Constants.AutoConstants.kPThetaController, Constants.AutoConstants.kIThetaController, Constants.AutoConstants.kDThetaController), // Rotation controller. Tune these values for your robot. Leaving them 0 will only use feedforwards.
             m_robotContainer.s_Swerve::setModuleStates, // Module states consumer
             true, // Should the path be automatically mirrored depending on alliance color. Optional, defaults to true
             m_robotContainer.s_Swerve // Requires this drive subsystem
         )
     );
 }
 

  @Override
  public void robotInit() {
    ctreConfigs = new CTREConfigs();

    moduleChooser = new SendableChooser<>();
    moduleCumulativeChooser = new SendableChooser<>();
    pathChooser = new SendableChooser<>();

    config = new TrajectoryConfig(
      Constants.AutoConstants.kMaxSpeedMetersPerSecond,
      Constants.AutoConstants.kMaxAccelerationMetersPerSecondSquared)
      .setKinematics(Constants.Swerve.swerveKinematics);

    Trajectory exampleTrajectory = TrajectoryGenerator.generateTrajectory(
      // Start at the origin facing the +X direction
      new Pose2d(0, 0, new Rotation2d(0)),
      // Pass through these two interior waypoints, making an 's' curve path
      List.of(new Translation2d(1, 1), new Translation2d(2, -1)),
      // End 3 meters straight ahead of where we started, facing forward
      new Pose2d(3, 0, new Rotation2d(0)),
      config);

    trajectories.add(exampleTrajectory); // S curve
    trajectories.add(loadTrajectory("pathplanner/generatedJSON/Test Path.wpilib.json")); // Left then up
    trajectories.add(loadTrajectory("pathplanner/generatedJSON/Red Bump Side.wpilib.json")); // Tank (Doesn't work)
    trajectories.add(loadTrajectory("pathplanner/generatedJSON/ChargeStationTest.wpilib.json")); // TEST PATH HOLONOMIC IN PATHPLANNER
    trajectories.add(loadTrajectory("pathplanner/generatedJSON/Test Spin.wpilib.json")); 

    moduleChooser.setDefaultOption("None", -1);
    moduleChooser.addOption("Module 0", 0);
    moduleChooser.addOption("Module 1", 1);
    moduleChooser.addOption("Module 2",2);
    moduleChooser.addOption("Module 3",3);
    
    moduleCumulativeChooser.setDefaultOption("1 Module", 1);
    moduleCumulativeChooser.addOption("2 Modules", 2);
    moduleCumulativeChooser.addOption("3 Modules", 3);
    moduleCumulativeChooser.addOption("4 Modules", 4);

    pathChooser.setDefaultOption("S Curve", 0);
    pathChooser.setDefaultOption("Test Path", 1);
    pathChooser.addOption("Red Bump Tank", 2);
    pathChooser.addOption("Charge Station", 3);
    pathChooser.addOption("Test Spin", 4);
    
    SmartDashboard.putData(moduleChooser);
    SmartDashboard.putData(moduleCumulativeChooser);
    SmartDashboard.putData(pathChooser);
    
      // Instantiate our RobotContainer.  This will perform all our button bindings, and put our
      // autonomous chooser on the dashboard.
      m_robotContainer = new RobotContainer();
    }
    
    private Trajectory loadTrajectory(String path) {
      Trajectory trajectory = new Trajectory();
      try {
        Path trajectoryPath = Filesystem.getDeployDirectory().toPath().resolve(path);
        trajectory = TrajectoryUtil.fromPathweaverJson(trajectoryPath);
      } catch (IOException ex) {
        DriverStation.reportError("Unable to open trajectory: " + path, ex.getStackTrace());
      }
      
    return trajectory;  
  }

  private Command loadCommand(Trajectory trajectory) {
    var thetaController = new ProfiledPIDController(Constants.AutoConstants.kPThetaController, 0, 0, Constants.AutoConstants.kThetaControllerConstraints);
    thetaController.enableContinuousInput(-Math.PI, Math.PI);

    SwerveControllerCommand swerveControllerCommand =
    new SwerveControllerCommand(
      trajectory, // PUT TRAJECTORY HERE
      m_robotContainer.s_Swerve::getPose,
      Constants.Swerve.swerveKinematics,
      new PIDController(Constants.AutoConstants.kPController, 0, Constants.AutoConstants.kDController),
      new PIDController(Constants.AutoConstants.kPController, 0, Constants.AutoConstants.kDController),
      thetaController,
      m_robotContainer.s_Swerve::setModuleStates,
      m_robotContainer.s_Swerve);
        
      // return swerveControllerCommand;
      return new InstantCommand(() -> m_robotContainer.s_Swerve.resetOdometry(trajectory.getInitialPose())).andThen(swerveControllerCommand);
  }

  /**
   * This function is called every robot packet, no matter the mode. Use this for items like
   * diagnostics that you want ran during disabled, autonomous, teleoperated and test.
   *
   * <p>This runs after the mode specific periodic functions, but before LiveWindow and
   * SmartDashboard integrated updating.
   */
  @Override
  public void robotPeriodic() {
    // Runs the Scheduler.  This is responsible for polling buttons, adding newly-scheduled
    // commands, running already-scheduled commands, removing finished or interrupted commands,
    // and running subsystem periodic() methods.  This must be called from the robot's periodic
    // block in order for anything in the Command-based framework to work.
    CommandScheduler.getInstance().run();
  }

  /** This function is called once each time the robot enters Disabled mode. */
  @Override
  public void disabledInit() {}

  @Override
  public void disabledPeriodic() {}

  /** This autonomous runs the autonomous command selected by your {@link RobotContainer} class. */
  @Override
  public void autonomousInit() {
    // PathPlannerTrajectory traj = loadPlannerTrajectory("pathplanner/generatedJSON/Test Spin.wpilib.json");
    // PathPlannerTrajectory traj = PathPlanner.loadPath("Straight Path", new PathConstraints(3, 3));

    Trajectory trajectory = trajectories.get(pathChooser.getSelected());
    m_autonomousCommand = loadCommand(trajectory);
    // m_autonomousCommand = followTrajectoryCommand(traj, true);

    // m_autonomousCommand = m_robotContainer.getAutonomousCommand();
    // PathPlannerTrajectory path = PathPlanner.loadPath("C:/Users/samis/Code/Steel Hawks/BaseFalconSwerveNEW23/BaseFalconSwerve/src/main/deploy/pathplanner/Test Path", new PathConstraints(Constants.AutoConstants.kMaxSpeedMetersPerSecond, Constants.AutoConstants.kMaxAccelerationMetersPerSecondSquared));
    // m_autonomousCommand = m_robotContainer.s_Swerve.followTrajectoryCommand(path, true);

    // schedule the autonomous command (example)
    if (m_autonomousCommand != null) {
      m_autonomousCommand.schedule();
    }
  }

  /** This function is called periodically during autonomous. */
  @Override
  public void autonomousPeriodic() {}

  @Override
  public void teleopInit() {
    //m_robotContainer.s_Swerve.resetModule(moduleChooser.getSelected());
    m_robotContainer.configureMoreButtonBindings(moduleCumulativeChooser.getSelected());
    // This makes sure that the autonomous stops running when
    // teleop starts running. If you want the autonomous to
    // continue until interrupted by another command, remove
    // this line or comment it out.
    if (m_autonomousCommand != null) {
      m_autonomousCommand.cancel();
    }

    //m_robotContainer.s_Swerve.resetModulesToAbsolute();
  }

  /** This function is called periodically during operator control. */
  @Override
  public void teleopPeriodic() {}

  @Override
  public void testInit() {
    // Cancels all running commands at the start of test mode.
    CommandScheduler.getInstance().cancelAll();
  }

  /** This function is called periodically during test mode. */
  @Override
  public void testPeriodic() {}
}
