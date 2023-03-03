package frc.robot;

public final class RobotMap {

  public static interface DRIVETRAIN {
    int MOTOR_RIGHT_ONE_PORT = 4;
    int MOTOR_LEFT_ONE_PORT = 25;

    double TICK_TO_FEET_CONVERSION = 1.0 / 2048 * 6 * Math.PI / 12 * 1 / 12.75;
  }

  public static interface ELEVATOR {
    int MOTOR_RIGHT_ONE_PORT = 0;
    int MOTOR_LEFT_ONE_PORT = 1;
    int SOLENOID_RIGHT_FORWARD = 4;
    int SOLENOID_RIGHT_REVERSE = 5;
    int SOLENOID_LEFT_FORWARD = 6;
    int SOLENOID_LEFT_REVERSE = 7;
  }

  public static interface CLAW {
    int SOLENOID_FORWARD = 0;
    int SOLENOID_REVERSE = 1;
  }
}
