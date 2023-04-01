package frc.robot;

import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;

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
    public static final int INTAKE = 12;

    public static final int GATE = 0;

    public static final int ARM_SWITCH = 0;
    public static final int CLAW_SWITCH = 1;
    
  }

  public static final class SwerveConstants{
    public static final double l = 0.595;
    public static final double w = 0.595;
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
    public static final double BATTERY_WEIGHT = 13;
    public static final double WEIGHT = 77.5 + BATTERY_WEIGHT;

    //THESE TWO VALUES BECOME LESS WITH MORE WEIGHT
    public static final double MAX_WHEEL_SPEED = 4.3; //in m/s, with no weight, it is 4.7
    public static final double MAX_WHEEL_ACCELERATION = -0.312 * WEIGHT + 45; //in m/s^2, with no weight it is 45

    public static final double AUTO_ROTATION = 2.45; //as this goes up, rotation speed goes down

    public static final double MS_TO_UPMS = (13824/28.4); //meters per second to units per 100ms
  }

  public static final class ArmConstants {
    public static final int ARM_GEAR_RATIO = 84;
    public static final int CLAW_GEAR_RATIO = 30;
    public static final double ARM_POSITION_CONVERSION = 0.2578;
    public static final double WRIST_POSITION_CONVERSION = 0.265;
    public static final double WRIST_BUMPER_SAFETY = -15;//-16;
    public static final double ARM_BUMPER_SAFETY = 11;//12;
    public static final double WRIST_BUMPER_SAFETY_SETPOINT = -10;//-10;
    public static final double KEPT_CLAW_ANGLE_WRIST_SAFETY = -5;
    public static final double ARM_LIMIT = 110;
    public static final double CLAW_LIMIT = -190;
  }

  public static final class VisionConstants {
    public static final double CAM_ANGLE = -0.13944; // MARCUS LABEL THESE PLEASE
    public static final double STREAM_WIDTH_PIXELS = 640;
    public static final double STREAM_HEIGHT_PIXELS = 640;
    public static final double[][] CAM_OFFSET = {{-0.273},{0.3175}}; //
    public static final double ROBOT_RADIUS = 0.5029; //
    public static final double INTAKE_OFFSET = 0.7;
  }

  public static final class AprilTagConstants {
    public static final double TAG_SIZE = 0.1524; // in metres, 6 inches converted, possibly wrong but the number they gave
    // ok so idk what focal length is but its supposed to be in pixels
    public static final Transform3d ROBOT_TO_CAM =
    new Transform3d(
            new Translation3d(0.0635, -0.2, 0.8),
            new Rotation3d(
                    0, 0,
                    0));  // Cam mounted facing forward, at center, half a meter up
  }

  public static final class OIConstants {
    public static final int mainStickPort = 0;
    public static final int controlPanelLeftPort = 1;
    public static final int controlPanelRightPort = 3;
  }

}