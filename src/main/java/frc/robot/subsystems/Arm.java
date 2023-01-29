package frc.robot.subsystems;

import frc.robot.SwerveModule;
import frc.robot.Constants;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import com.ctre.phoenix.sensors.Pigeon2;
import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.commands.PPSwerveControllerCommand;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

public class Arm extends SubsystemBase {

    private DoubleSolenoid armSolenoid;
    private DoubleSolenoid clawSolenoid;
    private WPI_TalonSRX MOTORONE;
    private WPI_TalonSRX MOTORTWO;
    private PneumaticsModuleType type = PneumaticsModuleType.CTREPCM;

    public Arm() {
        armSolenoid = new DoubleSolenoid(type , 0, 1);
        clawSolenoid = new DoubleSolenoid(type, 2, 3);
        MOTORONE = new WPI_TalonSRX(13);
        MOTORTWO =  new WPI_TalonSRX(15);
        configureMotors();
        MOTORONE.setInverted(true);
        MOTORTWO.setInverted(true);
    }

    private void configureMotors(){
        MOTORONE.configFactoryDefault();
        MOTORTWO.configFactoryDefault();
        MOTORONE.setNeutralMode(NeutralMode.Brake);
        MOTORTWO.setNeutralMode(NeutralMode.Brake);
    }

    public void toggleArm(){
        if(armSolenoid.get().equals(DoubleSolenoid.Value.kForward)){
            armSolenoid.set(DoubleSolenoid.Value.kReverse);
        }
        else{
            armSolenoid.set(DoubleSolenoid.Value.kForward);
        }
    }

    public void toggleClaw(){
        if(clawSolenoid.get().equals(DoubleSolenoid.Value.kForward)){
            clawSolenoid.set(DoubleSolenoid.Value.kReverse);
        }
        else{
            clawSolenoid.set(DoubleSolenoid.Value.kForward);
        }
    }

    public void armExtend(){
        MOTORONE.set(0.3);
        MOTORTWO.set(0.3);
    }

    public void armRetract(){
        MOTORONE.set(-0.3);
        MOTORTWO.set(-0.3);
    }

    public void armStop(){
        MOTORONE.stopMotor();
        MOTORTWO.stopMotor();
    }

    @Override
    public void periodic(){

    }
}