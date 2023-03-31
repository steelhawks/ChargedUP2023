package frc.robot;

import com.ctre.phoenix.motorcontrol.NeutralMode;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import frc.lib.util.COTSFalconSwerveConstants;
import frc.lib.util.SwerveModuleConstants;

public final class Constants {
    public static final double stickDeadband = 0.1;

    public static final class Swerve {
        public static final int pigeonID = 0;
        public static final boolean invertGyro = false; // Always ensure Gyro is CCW+ CW-

        public static final COTSFalconSwerveConstants chosenModule = 
            COTSFalconSwerveConstants.SDSMK4i(COTSFalconSwerveConstants.driveGearRatios.SDSMK4i_L2);

        /* Drivetrain Constants */
        public static final double trackWidth = Units.inchesToMeters(26.75);
        public static final double wheelBase = Units.inchesToMeters(26.75);
        public static final double wheelCircumference = chosenModule.wheelCircumference;

        /* Swerve Kinematics */
         public static final SwerveDriveKinematics swerveKinematics = new SwerveDriveKinematics(
            new Translation2d(wheelBase / 2.0, trackWidth / 2.0),
            new Translation2d(wheelBase / 2.0, -trackWidth / 2.0),
            new Translation2d(-wheelBase / 2.0, trackWidth / 2.0),
            new Translation2d(-wheelBase / 2.0, -trackWidth / 2.0));

        /* Module Gear Ratios */
        public static final double driveGearRatio = chosenModule.driveGearRatio;
        public static final double angleGearRatio = chosenModule.angleGearRatio;

        /* Motor Inverts */
        public static final boolean angleMotorInvert = chosenModule.angleMotorInvert;
        public static final boolean driveMotorInvert = chosenModule.driveMotorInvert;

        /* Angle Encoder Invert */
        public static final boolean canCoderInvert = chosenModule.canCoderInvert;

        /* Swerve Current Limiting */
        public static final int angleContinuousCurrentLimit = 25;
        public static final int anglePeakCurrentLimit = 40;
        public static final double anglePeakCurrentDuration = 0.1;
        public static final boolean angleEnableCurrentLimit = true;

        public static final int driveContinuousCurrentLimit = 35;
        public static final int drivePeakCurrentLimit = 60; 
        public static final double drivePeakCurrentDuration = 0.1; 
        public static final boolean driveEnableCurrentLimit = true;

        /* These values are used by the drive falcon to ramp in open loop and closed loop driving.
         * We found a small open loop ramp (0.25) helps with tread wear, tipping, etc */
        public static final double openLoopRamp = 0.25;
        public static final double closedLoopRamp = 0.0;

        /* Angle Motor PID Values */
        public static final double angleKP = chosenModule.angleKP;
        public static final double angleKI = chosenModule.angleKI;
        public static final double angleKD = chosenModule.angleKD;
        public static final double angleKF = chosenModule.angleKF;

        /* Drive Motor PID Values */
        public static final double driveKP = 0.26;
        public static final double driveKI = 0.0;
        public static final double driveKD = 0.0;
        public static final double driveKF = 0.0;

        /* Drive Motor Characterization Values 
         * Divide SYSID values by 12 to convert from volts to percent output for CTRE */
        public static final double driveKS = (0.23 / 12); // 0.2
        public static final double driveKV = (2.26 / 12); // 2.21
        public static final double driveKA = (0.29 / 12); // 0.22

        /* Swerve Profiling Values */
        public static final double maxSpeed = 4.5; // meters per second
        public static final double maxAngularVelocity = 10.0; // radians per second

        /* Neutral Modes */
        public static final NeutralMode angleNeutralMode = NeutralMode.Coast;
        public static final NeutralMode driveNeutralMode = NeutralMode.Brake;

        /* Module Specific Constants */
        /* Front Left Module - Module 0 */
        public static final class Mod0 { 
            public static final int driveMotorID = 7;
            public static final int angleMotorID = 8;
            public static final int canCoderID = 9;
            public static final Rotation2d angleOffset = Rotation2d.fromDegrees(39.8); //30.41
            public static final SwerveModuleConstants constants = 
                new SwerveModuleConstants(driveMotorID, angleMotorID, canCoderID, angleOffset);
        }

        /* Front Right Module - Module 1 */
        public static final class Mod1 {
            public static final int driveMotorID = 10;
            public static final int angleMotorID = 11;
            public static final int canCoderID = 12;
            public static final Rotation2d angleOffset = Rotation2d.fromDegrees(12.8); //81.563
            public static final SwerveModuleConstants constants = 
                new SwerveModuleConstants(driveMotorID, angleMotorID, canCoderID, angleOffset);
        }
        
        /* Back Left Module - Module 2 */
        public static final class Mod2 {
            public static final int driveMotorID = 4;
            public static final int angleMotorID = 5;
            public static final int canCoderID = 6;
            public static final Rotation2d angleOffset = Rotation2d.fromDegrees(88.4); // 92.725
            public static final SwerveModuleConstants constants = 
                new SwerveModuleConstants(driveMotorID, angleMotorID, canCoderID, angleOffset);
        }

        /* Back Right Module - Module 3 */
        public static final class Mod3 {
            public static final int driveMotorID = 1;
            public static final int angleMotorID = 2;
            public static final int canCoderID = 3;
            public static final Rotation2d angleOffset = Rotation2d.fromDegrees(329.1); //
            public static final SwerveModuleConstants constants = 
                new SwerveModuleConstants(driveMotorID, angleMotorID, canCoderID, angleOffset);
        }
    }

    // Elevator constants
    public static final class Elevator {
        public static final int motor1ID = 22;
        public static final int motor2ID = 21;
        public static final int SolenoidForward = 3;
        public static final int SolenoidReverse = 2;

        public static final NeutralMode motorNeutralMode = NeutralMode.Brake;

        /* Encoder Levels */
        public static final double homeEncoderPos = 0;
        public static final double lowEncoderPos = 1.235;
        public static final double midEncoderPos = 2.65; //2.85
        public static final double highEncoderPos = 4.1;
        public static final double doubleSubEncoderPos = 2.1;
        public static final double singleSubEncoderPos = 0.33;
        public static final double minPivotEncoderPos = 1.39; // When robot automatically pivots up
        public static final double minPivotTolerance = 0.41; // When to start decelerating when returning home
        public static final double maxPivotEncoderPos = 1.4; // When robot automatically pivots down
        public static final double maxEncoderPos = 4.2;

        public static final double elevatorSpeed = 0.4; // Manual speed

        public static final int canCoderID = 13;
        public static final boolean canCoderInvert = true;

        public static final int limitSwitchLowPort = 1;
        public static final int limitSwitchHighPort = 3;

        public static final double kP = 0.7;
        public static final double kI = 0.0;
        public static final double kD = 0.0;
        public static final double tolerance = 0.1;

        public static final double homeSpeed = 0.6;
        public static final double homeLowSpeed = 0.45;
    }

    // Claw constants
    public static final class Claw {
        public static final int SolenoidForward = 1;
        public static final int SolenoidReverse = 0;
        public static final int beamPort = 2;
    }

    // LED constants
    public static final class Led {
        public static final int port = 0;
        public static final int length = 137;
        // 1-69 and 70 - 137 is other side of robot
    }

    // Vision constants
    public static final class Vision {
        public static final double areaThreshold = 4;
        public static final double xOffsetThreshold = 0.3;
        public static final double xVelocity = 0.8;
        public static final double spinVelocity = 1;
        public static final int FiftyFour = 54;
        public static final double NodeDistance = 20; // TODO find constant
    }

    // Auto balance constants
    public static final class AutoBalance {
        public static final double kP = 0.05;
        public static final double kI = 0.0038;
        public static final double kD = 0.00595;

        public static final double setpoint = -0.4;
        public static final double tolerance = 0.4;
    }

    public static final class AutoConstants {
        public static final double kMaxSpeedMetersPerSecond = 3;
        public static final double kMaxAccelerationMetersPerSecondSquared = 3;
        public static final double kMaxAngularSpeedRadiansPerSecond = Math.PI;
        public static final double kMaxAngularSpeedRadiansPerSecondSquared = Math.PI;
    
        public static final double kPController = 5;
        public static final double kIController = 5; 
        public static final double kDController = 0;
        public static final double kPThetaController = 0.0;
        public static final double kIThetaController = 0.0;
        public static final double kDThetaController = 0.0;

        /* Constraint for the motion profilied robot angle controller */
        public static final TrapezoidProfile.Constraints kThetaControllerConstraints =
            new TrapezoidProfile.Constraints(
                kMaxAngularSpeedRadiansPerSecond, kMaxAngularSpeedRadiansPerSecondSquared);
    }
}