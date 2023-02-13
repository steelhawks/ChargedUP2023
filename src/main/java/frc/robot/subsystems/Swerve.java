package frc.robot.subsystems;

import frc.robot.SwerveModule;
import frc.robot.Constants;

import frc.lib.math.Conversions;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;

import com.ctre.phoenix.sensors.Pigeon2;
import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.commands.PPSwerveControllerCommand;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Swerve extends SubsystemBase {
    public SwerveDriveOdometry swerveOdometry;
    public SwerveModule[] mSwerveMods;
    public Pigeon2 gyro;

    private double speedMultiplier;
    private boolean isShifted; 

    public Swerve() {
        gyro = new Pigeon2(Constants.Swerve.pigeonID);
        gyro.configFactoryDefault();
        zeroGyro();

        mSwerveMods = new SwerveModule[] {
            new SwerveModule(0, Constants.Swerve.Mod0.constants),
            new SwerveModule(1, Constants.Swerve.Mod1.constants),
            new SwerveModule(2, Constants.Swerve.Mod2.constants),
            new SwerveModule(3, Constants.Swerve.Mod3.constants)
        };

        Timer.delay(1);
        resetModulesToAbsolute();

        swerveOdometry = new SwerveDriveOdometry(Constants.Swerve.swerveKinematics, getYaw(), getModulePositions());

        speedMultiplier = 1;
        isShifted = false;
    }

    public void rotateToAngle(int angle) {
        double goal = getTargetAngle(angle);

        if (gyro.getYaw() < goal) {
            drive(new Translation2d(0, 0), -0.3 * Constants.Swerve.maxAngularVelocity, true, true);
        }
        else {
            drive(new Translation2d(0, 0), 0.3 * Constants.Swerve.maxAngularVelocity, true, true);
        }
    }

    public double getTargetAngle(int angle) {
        return Conversions.toNearestZero(gyro.getYaw(), angle);
    }

    public void shiftGear() {
        if (speedMultiplier == 1) {
            speedMultiplier = 0.2;
            isShifted = true;
        }
        else {
            speedMultiplier = 1;
            isShifted = false;
        }
    }

    public void resetModule(int index) {
        if(index < 0) {
            System.out.println("\n\n\n\n\n\nDID NOT ZERO\n\n\n\n\n");
            return;
        }
        mSwerveMods[index].resetToAbsolute();
        System.out.println("\n\n\n\n\nZeroed Module " + index + "\n\n\n\n\n");
    }

    public void resetCumulativeModules(int num) {
        for(int i = 0; i < num; i++) {
            resetModule(i);
        }
    }

    public void drive(Translation2d translation, double rotation, boolean fieldRelative, boolean isOpenLoop) {
        SwerveModuleState[] swerveModuleStates =
            Constants.Swerve.swerveKinematics.toSwerveModuleStates(
                fieldRelative ? ChassisSpeeds.fromFieldRelativeSpeeds(
                                    translation.getX() * speedMultiplier, 
                                    translation.getY() * speedMultiplier, 
                                    rotation * speedMultiplier,
                                    getYaw()
                                )
                                : new ChassisSpeeds(
                                    translation.getX(), 
                                    translation.getY(), 
                                    rotation)
                                );

        SwerveDriveKinematics.desaturateWheelSpeeds(swerveModuleStates, Constants.Swerve.maxSpeed);

        for(SwerveModule mod : mSwerveMods){
            mod.setDesiredState(swerveModuleStates[mod.moduleNumber], isOpenLoop);
        }
    }    

    /* Used by SwerveControllerCommand in Auto */
    public void setModuleStates(SwerveModuleState[] desiredStates) {
        SwerveDriveKinematics.desaturateWheelSpeeds(desiredStates, Constants.Swerve.maxSpeed);
        
        for(SwerveModule mod : mSwerveMods){
            mod.setDesiredState(desiredStates[mod.moduleNumber], true); // false for feedforward
        }
    }    

    public Pose2d getPose() {
        return swerveOdometry.getPoseMeters();
    }

    public void resetOdometry(Pose2d pose) {
        swerveOdometry.resetPosition(getYaw(), getModulePositions(), pose);
    }

    public SwerveModuleState[] getModuleStates(){
        SwerveModuleState[] states = new SwerveModuleState[4];
        for(SwerveModule mod : mSwerveMods){
            states[mod.moduleNumber] = mod.getState();
        }
        return states;
    }

    public SwerveModulePosition[] getModulePositions(){
        SwerveModulePosition[] positions = new SwerveModulePosition[4];
        for(SwerveModule mod : mSwerveMods){
            positions[mod.moduleNumber] = mod.getPosition();
        }
        return positions;
    }

    public void zeroGyro(){
        gyro.setYaw(0);
    }

    public Rotation2d getYaw() {
        return (Constants.Swerve.invertGyro) ? Rotation2d.fromDegrees(360 - gyro.getYaw()) : Rotation2d.fromDegrees(gyro.getYaw());
    }

    public double getRoll() {
        return gyro.getRoll();
    }

    public void resetModulesToAbsolute(){
        for(SwerveModule mod : mSwerveMods){
            mod.resetToAbsolute();
        }
    }

    public boolean isLowGear() {
        return isShifted;
    }

    /*
     * Call after robot is on ramp
     * Click button to call method
     * Move all wheels to zero degrees ("tank mode")
     * If roll positive, move wheels positive
     * If roll negative, move wheels negative
     * Speed proportional to roll
     */
    // public void autoBalance() {
    //     double roll = gyro.getRoll();
    //     double yaw = Math.abs(gyro.getYaw() % 360);
    //     int multiplier = 1;
    //     double deadband = 2;

    //     if (yaw > 180) {
    //         multiplier = -1;
    //         System.out.println("greater");
    //     }

    //     Translation2d translation = new Translation2d(0, 0);
    //     double rotation = multiplier * 0.15 * Constants.Swerve.maxAngularVelocity;

    //     while (yaw < 360 - deadband && yaw > deadband) { //yaw > deadband || 360 - yaw < deadband
    //         yaw = Math.abs(gyro.getYaw() % 360);

    //         drive(translation, rotation, true, true);
    //     }

    //     if (Math.abs(roll) <= rollDeadband) return;

        
    // }

    public Command followTrajectoryCommand(PathPlannerTrajectory traj, boolean isFirstPath) {
        return new SequentialCommandGroup(
             new InstantCommand(() -> {
               // Reset odometry for the first path you run during auto
               if(isFirstPath){
                   this.resetOdometry(traj.getInitialHolonomicPose());
               }
             }),
             new PPSwerveControllerCommand(
                 traj, 
                 this::getPose, // Pose supplier
                 Constants.Swerve.swerveKinematics, // SwerveDriveKinematics
                 new PIDController(0, 0, 0), // X controller. Tune these values for your robot. Leaving them 0 will only use feedforwards.
                 new PIDController(0, 0, 0), // Y controller (usually the same values as X controller)
                 new PIDController(0, 0, 0), // Rotation controller. Tune these values for your robot. Leaving them 0 will only use feedforwards.
                 this::setModuleStates, // Module states consumer
                 true, // Should the path be automatically mirrored depending on alliance color. Optional, defaults to true
                 this // Requires this drive subsystem
             )
         );
     }

     public void stop() {
        Translation2d translation = new Translation2d(0, 0);
        drive(translation, 0, true, true);
     }
     

    @Override
    public void periodic(){
        swerveOdometry.update(getYaw(), getModulePositions());  

        for(SwerveModule mod : mSwerveMods){
            SmartDashboard.putNumber("Mod " + mod.moduleNumber + " Cancoder", mod.getCanCoder().getDegrees());
            SmartDashboard.putNumber("Mod " + mod.moduleNumber + " Integrated", mod.getPosition().angle.getDegrees());
            SmartDashboard.putNumber("Mod " + mod.moduleNumber + " Velocity", mod.getState().speedMetersPerSecond);   
            SmartDashboard.putNumber("Mod " + mod.moduleNumber + " Angle Amp", mod.getAngleAmperage());
            SmartDashboard.putNumber("Mod " + mod.moduleNumber + " Drive Amp", mod.getDriveAmperage()); 
            SmartDashboard.putNumber("Mod " + mod.moduleNumber + " Angle Percent", mod.getAngleOutput());
            SmartDashboard.putNumber("Mod " + mod.moduleNumber + " Drive Percent", mod.getDriveOutput()); 
            SmartDashboard.putNumber("Mod " + mod.moduleNumber + " Drive Volt", mod.getVoltage()); 
            SmartDashboard.putBoolean("Is Shifted", isShifted);
        }

        SmartDashboard.putNumber("Yaw", gyro.getYaw()); // Math.abs(gyro.getYaw() % 360)
        // SmartDashboard.putNumber("Yaw", gyro.getYaw());
        SmartDashboard.putNumber("Pitch", gyro.getPitch());
        SmartDashboard.putNumber("Roll", gyro.getRoll());
    }
}