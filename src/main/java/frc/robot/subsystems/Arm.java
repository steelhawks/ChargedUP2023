package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Arm extends SubsystemBase {

    private DoubleSolenoid armSolenoid;
    private DoubleSolenoid clawSolenoid;
    // private DoubleSolenoid clawSolenoidTwo;
    private WPI_TalonSRX MOTORONE;
    private WPI_TalonSRX MOTORTWO;
    private PneumaticsModuleType type = PneumaticsModuleType.CTREPCM;

    public Arm() {
        armSolenoid = new DoubleSolenoid(type , 0, 1);
        clawSolenoid = new DoubleSolenoid(type, 2, 3);
        // clawSolenoidTwo = new DoubleSolenoid(type, 4,5);
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
            // clawSolenoidTwo.set(DoubleSolenoid.Value.kReverse);
        }
        else{
            clawSolenoid.set(DoubleSolenoid.Value.kForward);
            // clawSolenoidTwo.set(DoubleSolenoid.Value.kForward);
        }
    }

    public void armExtend(){
        MOTORONE.set(0.7);
        MOTORTWO.set(0.7);
    }

    public void armRetract(){
        MOTORONE.set(-0.7);
        MOTORTWO.set(-0.7);
    }

    public void armStop(){
        MOTORONE.stopMotor();
        MOTORTWO.stopMotor();
    }

    @Override
    public void periodic(){

    }
}