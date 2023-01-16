package frc.robot;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import com.revrobotics.CANSparkMax.IdleMode;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;

public final class Constants {

    private Constants() {}

    static double imperialToMeters(int feet, double inches) {
        return Units.feetToMeters(feet) + Units.inchesToMeters(inches);
    }

    // height of tape
    // robot starting positions
    // shooting distance

    double WIDTH = imperialToMeters(26, 3.5);
    double LENGTH = imperialToMeters(54, 3.25);
    Translation2d CENTER = new Translation2d(0, 0);


    public interface Balls {
        public interface Alliance {
            Translation2d UPPER = new Translation2d(-3.286667, 2.073733);
            Translation2d MIDDLE = new Translation2d(-3.173632, -2.242902);
            Translation2d LOWER = new Translation2d(-0.658126, -3.830068);
            Translation2d TERMINAL = new Translation2d(-7.164845, -2.990216);
        }

        public interface Opponent {
            Translation2d UPPER = new Translation2d(-2.242902, 3.173632);
            Translation2d MIDDLE = new Translation2d(-3.790375, -0.857674);
            Translation2d LOWER = new Translation2d(0.857674, -3.790375);
        }
    }

    public interface Hangar {
        double WIDTH = imperialToMeters(9, 8);
        double LENGTH = imperialToMeters(10, 8.75);
        double HEIGHT = imperialToMeters(6, 2);

        Translation2d BOTTOM_LEFT =
                new Translation2d(-imperialToMeters(27, 0), +imperialToMeters(3, 10));

        Translation2d BOTTOM_RIGHT =
                new Translation2d(-imperialToMeters(16, 5.75), +imperialToMeters(3, 10));

        Translation2d TOP_RIGHT =
                new Translation2d(-imperialToMeters(16, 5.75), +imperialToMeters(13, 6));
    }

    public interface Hub {
        Translation2d CENTER = new Translation2d(0, 0);

        double HEIGHT = imperialToMeters(8, 8);
        double UPPER_RADIUS = imperialToMeters(2, 0);
    }

    public interface Terminal {
        double LENGTH = imperialToMeters(7, 8.5);
    }
    

    public interface Drivetrain {
        int CURRENT_LIMIT_AMPS = 60; //questionable, not determined
        NeutralMode IDLE_MODE = NeutralMode.Brake;

        Config LEFT = new Config(true, IDLE_MODE, CURRENT_LIMIT_AMPS);
        Config RIGHT = new Config(false, IDLE_MODE, CURRENT_LIMIT_AMPS);
    }

    /** Class to store all of the values a motor needs */
    public static class Config {
        public final boolean INVERTED;
        public final NeutralMode NEUTRAL_MODE;
        public final int CURRENT_LIMIT_AMPS;
        public final double OPEN_LOOP_RAMP_RATE;

        public Config(
                boolean inverted,
                NeutralMode neutralMode,
                int currentLimitAmps,
                double openLoopRampRate) {
            this.INVERTED = inverted;
            this.NEUTRAL_MODE = neutralMode;
            this.CURRENT_LIMIT_AMPS = currentLimitAmps;
            this.OPEN_LOOP_RAMP_RATE = openLoopRampRate;
        }

        public Config(boolean inverted, NeutralMode neutralMode, int currentLimitAmps) {
            this(inverted, neutralMode, currentLimitAmps, 0.0);
        }

        public Config(boolean inverted, NeutralMode neutralMode) {
            this(inverted, neutralMode, 80);
        }

        public void configure(WPI_TalonSRX motor) {
            motor.configFactoryDefault();
            motor.setInverted(INVERTED);
            motor.setNeutralMode(NEUTRAL_MODE);
            motor.configPeakCurrentLimit(CURRENT_LIMIT_AMPS);
            motor.configOpenloopRamp(OPEN_LOOP_RAMP_RATE);
        }
    }


}