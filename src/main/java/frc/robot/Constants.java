package frc.robot;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide
 * numerical or boolean constants. This class should not be used for any other
 * purpose. All constants should be declared globally (i.e. public static). Do
 * not put anything functional in this class.
 *
 * <p>
 * It is advised to statically import this class (or one of its inner classes)
 * wherever the constants are needed, to reduce verbosity.
 */
public final class Constants {
  // Units are m kg s unless otherwise specified

  public static final class PortConstants {
    public static final int FRONT_LEFT_ROTATION_DRIVE = 1;
    public static final int FRONT_RIGHT_ROTATION_DRIVE = 3;
    public static final int BACK_LEFT_ROTATION_DRIVE = 5;
    public static final int BACK_RIGHT_ROTATION_DRIVE = 7;

    public static final int FRONT_LEFT_DIRECTION_DRIVE = 2;
    public static final int FRONT_RIGHT_DIRECTION_DRIVE = 4;
    public static final int BACK_LEFT_DIRECTION_DRIVE = 6;
    public static final int BACK_RIGHT_DIRECTION_DRIVE = 8;

    public static final int FRONT_LEFT_CODER_DRIVE = 1;
    public static final int FRONT_RIGHT_CODER_DRIVE = 3;
    public static final int BACK_LEFT_CODER_DRIVE = 5;
    public static final int BACK_RIGHT_CODER_DRIVE = 7;

    public static final int ARM_UP = 9;
    public static final int ARM_DOWN = 10;
    public static final int WRIST = 11;

    public static final int GATE = 0;
    
  }

  public static final class SwerveConstants{
    public static final double l = 0.61595;
    public static final double w = 0.61595;
    public static final double r = Math.sqrt(Math.pow(l, 2) + Math.pow(w, 2));
    public static final double centerToMotor = 0;//Distance from center of motor to center of robot
    public static final double unitsToRotations = (2048/360);
    //To Be Changed
    public static final double ksVolts = 0.22;
    public static final double kvVoltSecondsPerMeter = 1.98;
    public static final double kvVoltsSecondsSquaredPerMeter = 0.2;
    public static final double kPDriveVel = 8.5;
    public static final double kMaxSpeedMetersPerSecond = 3;
    public static final double kMaxAccelerationMeterPerSecondSquared = 1;
    public static final double kSwerveB = 2;
    public static final double kSwerveZeta = 0.7;
    public static final double WHEEL_CIRCUMFERENCE = 0.2865;
    public static final double SWERVE_POSITION_RATIO = WHEEL_CIRCUMFERENCE/13824; //units to meters
    public static final double PERIODIC_SPEED = 0.02;

    public static final double WEIGHT = 35;

    //THESE TWO VALUES BECOME LESS WITH MORE WEIGHT
    public static final double MAX_WHEEL_SPEED = 4.5; //in m/s, with no weight, it is 4.7
    public static final double MAX_WHEEL_ACCELERATION = 35; //in m/s^2, with no weight it is 45


    public static final double MS_TO_UPMS = (13824/28.4); //meters per second to units per 100ms
  }

  public static final class ArmConstants {
    public static final int ARM_GEAR_RATIO = 100;
    public static enum CLAW_STATE {Open, Closed};
  }

  public static final class OIConstants {
    public static final int mainStickPort = 0;
  }

}