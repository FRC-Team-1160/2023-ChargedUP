/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems.DriveTrain;

import java.util.ArrayList;

import java.util.Optional;

import org.littletonrobotics.junction.Logger;
import org.photonvision.EstimatedRobotPose;

import com.ctre.phoenix.motorcontrol.TalonFXControlMode;
import com.ctre.phoenix.motorcontrol.TalonFXSensorCollection;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenix.sensors.CANCoder;
import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.SerialPort.Port;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.OIConstants;
import frc.robot.Constants.PortConstants;
import frc.robot.Constants.SwerveConstants;
import frc.robot.subsystems.Arm.Arm;
import frc.robot.subsystems.Vision.AprilTag;
import frc.robot.subsystems.Vision.Limelight;

import org.littletonrobotics.junction.Logger;


public class DriveTrain extends SubsystemBase{
  /** 
   * Creates a new DriveTrain.
   */

  private Joystick m_mainStick = new Joystick(OIConstants.mainStickPort);
  
  private static DriveTrain m_instance;

  private TalonFX m_frontLeftRotationMotor, m_frontRightRotationMotor, m_backLeftRotationMotor, m_backRightRotationMotor;

  private TalonFXSensorCollection m_frontLeftRotationEncoder, m_frontRightRotationEncoder, m_backLeftRotationEncoder, m_backRightRotationEncoder;

  private static TalonFX m_frontLeftDirectionMotor;

  private TalonFX m_frontRightDirectionMotor;

  private TalonFX m_backLeftDirectionMotor;

  private TalonFX m_backRightDirectionMotor;

  private TalonFXSensorCollection m_frontLeftDirectionEncoder, m_frontRightDirectionEncoder, m_backLeftDirectionEncoder, m_backRightDirectionEncoder;

  private SwerveDriveWheel m_frontLeftWheel, m_frontRightWheel, m_backLeftWheel, m_backRightWheel;
  
  public SwerveDriveController m_controller;

  private CANCoder m_frontLeftCoder, m_frontRightCoder, m_backLeftCoder, m_backRightCoder;

  private AHRS m_gyro;

  private Translation2d m_frontLeftLocation, m_frontRightLocation, m_backLeftLocation, m_backRightLocation;
  private SwerveDriveKinematics m_kinematics;

  private Solenoid m_gate;
  public double m_poseX;
  public double m_poseY;
  public double prevPoseX;
  public double prevPoseY;
  public Pose2d m_pose2D;
  public Pose2d prevPose2D;
  

  public Pose m_pose;
  public Field2d m_fieldSim = new Field2d();
  public int time;

  public ArrayList<Pose> poseLog;

  public boolean limelightEngage;
  private PIDController limelightEngageController;
  private double gyroOffset;

  public AprilTag at;

  double wkrP, wkrI, wkrD, wksP, wksI, wksD, wklP, wklI, wklD;
//

  //initializes the drive train
  
  public static DriveTrain getInstance(){
    if (m_instance == null){
      m_instance = new DriveTrain();
    }
    return m_instance;
  }

  private DriveTrain() {

    //swerve wheel PID values
    wkrP = 0.007;
    wkrI = 0.0005;
    wkrD = 0;
    wksP = 1.5; //1.5
    wksI = 0.05; //0.05
    wksD = 0;

    //limelight PID values
    wklP = 0.013;
    wklI = 0.0;
    wklD = 0;

    //swerve modules

    m_frontLeftWheel = initSwerveModule(wkrP, wkrI, wkrD, wksP, wksI, wksD, PortConstants.FRONT_LEFT_DIRECTION_DRIVE, PortConstants.FRONT_LEFT_ROTATION_DRIVE, PortConstants.FRONT_LEFT_CODER_DRIVE);
    m_frontRightWheel = initSwerveModule(wkrP, wkrI, wkrD, wksP, wksI, wksD, PortConstants.FRONT_RIGHT_DIRECTION_DRIVE, PortConstants.FRONT_RIGHT_ROTATION_DRIVE, PortConstants.FRONT_RIGHT_CODER_DRIVE);
    m_backLeftWheel = initSwerveModule(wkrP, wkrI, wkrD, wksP, wksI, wksD, PortConstants.BACK_LEFT_DIRECTION_DRIVE, PortConstants.BACK_LEFT_ROTATION_DRIVE, PortConstants.BACK_LEFT_CODER_DRIVE);
    m_backRightWheel = initSwerveModule(wkrP, wkrI, wkrD, wksP, wksI, wksD, PortConstants.BACK_RIGHT_DIRECTION_DRIVE, PortConstants.BACK_RIGHT_ROTATION_DRIVE, PortConstants.BACK_RIGHT_CODER_DRIVE);

    m_gyro = new AHRS(Port.kMXP);
    
    m_gyro.zeroYaw();
    gyroOffset = 0;
    m_poseX = 0;
    m_poseY = 0;
    prevPoseX = 0;
    prevPoseY = 0;
    time = 0;
    m_pose = new Pose(m_poseX, m_poseY, getGyroAngle(), 0);
    poseLog = new ArrayList<Pose>();
    m_controller = new SwerveDriveController(m_frontLeftWheel, m_frontRightWheel, m_backLeftWheel, m_backRightWheel, m_gyro);
  
    limelightEngage = false;
    limelightEngageController = new PIDController(wklP, wklI, wklD);
    
    at = AprilTag.getInstance();
  }

  public double getGyroAngle() {
    double angle = m_gyro.getYaw() + gyroOffset;
    if (angle > 180) {
      angle -= 360;
    }
    if (angle < -180) {
      angle += 360;
    }
    return angle;
  }

  public void resetWheelPositions() {
    m_frontRightWheel.resetPosition();
    m_frontLeftWheel.resetPosition();
    m_backRightWheel.resetPosition();
    m_backLeftWheel.resetPosition();
  }

  public void resetGyro() {
    m_gyro.zeroYaw();
    gyroOffset = 0;
  }

  public void resetGyroToPosition(double rot) {
    gyroOffset = rot;
    m_gyro.zeroYaw();
  }

  public double getGyroX() {
    return m_gyro.getRawGyroX();
  }

  public double getGyroY() {
    return m_gyro.getRawGyroY();
  }

  public double getGyroZ() {
    return m_gyro.getRawGyroZ();
  }

  public double getGyroPitch() {
    return m_gyro.getPitch();
  }

  public void resetPose() {
    m_poseX = 0;
    m_poseY = 0;
  }

  public void resetOdometry(double poseX, double poseY) {
    m_poseX = poseX;
    m_poseY = poseY;
  }

  public void turnBackRight(double speed) {
    m_backRightRotationMotor.set(TalonFXControlMode.PercentOutput, speed);
  }

  public void setGate(boolean b) {
    m_gate.set(b);
  }

  public boolean getGateStatus() {
    return m_gate.get();
  }

  public double locToAngle(double ogLoc) {
    if (ogLoc > 180) {
      ogLoc -= 360;
    }
    return ogLoc;
  }

  public double limelightEngagePID() {
    double feedback = limelightEngageController.calculate(Limelight.getTx(), 0);

    return feedback;
  }

  public void logPose() {
    poseLog.add(m_pose);
  }

  //gets the pose at sec seconds ago
  public Pose getPastPose(double sec) {
    int timestampBefore = time-(int)Math.ceil(sec/0.02);
    int timestampAfter = time-(int)Math.floor(sec/0.02);
    double pos = 1+(((time*0.02-sec)-(double)timestampAfter*0.02)/(0.02)); //value from 0 to 1, where 0 if it is exacty at timestamp before and 1 if exactly at timestamp after
    Pose poseBefore = poseLog.get(timestampBefore);
    Pose poseAfter = poseLog.get(timestampAfter);
    return Pose.getPoseBetweenPoses(poseBefore, poseAfter, pos);
  }
  /** Updates the field-relative position. */
  public void updateOdometry() {

    Optional<EstimatedRobotPose> result =
            at.getEstimatedGlobalPose(new Pose2d(m_poseX, m_poseY, new Rotation2d(Math.toRadians(getGyroAngle()))));

    if (result.isPresent()) {
        EstimatedRobotPose camPose = result.get();
        m_pose2D = camPose.estimatedPose.toPose2d();
        if (checkAprilAccuracy()) {
          m_poseX = m_pose2D.getX();
          m_poseY = m_pose2D.getY();
        }
    }
   
}
  public boolean checkAprilAccuracy() {
    double poseDiffX = m_poseX - prevPoseX;
    double poseDiffY = m_poseY - prevPoseY;
    Transform2d poseDiff = m_pose2D.minus(prevPose2D);
    if (Math.abs(poseDiffX-poseDiff.getX()) < 0.18 && Math.abs(poseDiffY-poseDiff.getY()) < 0.18) {
      return true;
    }
    return false;
  }

  
  @Override
  public void periodic() {
    prevPoseX = m_poseX;
    prevPoseY = m_poseY;
    prevPose2D = new Pose2d(m_pose2D.getX(), m_pose2D.getY(), m_pose2D.getRotation());
    double xAngle = m_mainStick.getRawAxis(0);
    double yAngle = m_mainStick.getRawAxis(1);
    double angle = Math.toDegrees(Math.atan(yAngle/xAngle)) + 90;
    if (xAngle < 0) {
      angle += 180;
    }
    double mag = Math.sqrt(xAngle*xAngle + yAngle*yAngle);
    if (mag > 1) {
      mag = 1;
    }
    double turn = m_mainStick.getRawAxis(4);
    SmartDashboard.putNumber("Angle", angle);
    SmartDashboard.putNumber("Mag", mag);
    SmartDashboard.putNumber("Turn", turn);
    
    SmartDashboard.putNumber("FLCoder", m_frontLeftWheel.getRotation());
    SmartDashboard.putNumber("FRCoder", m_frontRightWheel.getRotation());
    SmartDashboard.putNumber("BLCoder", m_backLeftWheel.getRotation());
    SmartDashboard.putNumber("BRCoder", m_backRightWheel.getRotation());
    // UPDATE ODOMETRY
    double[] odom = m_controller.getSwerveOdometry(locToAngle(getGyroAngle()));
    double fwd = odom[0];
    double str = odom[1];
    m_poseX += fwd*SwerveConstants.PERIODIC_SPEED;
    m_poseY += str*SwerveConstants.PERIODIC_SPEED;

    m_pose.updatePose(m_poseX, m_poseY, getGyroAngle(), time);
    if (DriverStation.isTeleopEnabled()) {
      logPose();
      time += 1;
    }
    if (!Limelight.getTv() || (Arm.getInstance().angle > 68 && Limelight.getPipeline().intValue() == 0) || (Arm.getInstance().angle > 55 && Limelight.getPipeline().intValue() == 1)) {
      limelightEngage = false;
    }
    SmartDashboard.putNumber("gyroPitch", getGyroPitch());
    

    updateOdometry();
    //
    Logger.getInstance().recordOutput("Odometry/RotationDegrees",
          getGyroAngle());
      Logger.getInstance().recordOutput("Odometry/XMeters", m_poseX);
      Logger.getInstance().recordOutput("Odometry/YMeters", m_poseY);
    
    SmartDashboard.putNumber("PoseX", m_poseX);
    SmartDashboard.putNumber("PoseY", m_poseY);
    SmartDashboard.putNumber("Gyro Yaw", getGyroAngle());
    SmartDashboard.putNumber("Pose2DX", m_pose2D.getX());
    SmartDashboard.putNumber("Pose2DY", m_pose2D.getY());
    SmartDashboard.putNumber("PoseGyro", m_pose2D.getRotation().getDegrees());
    SmartDashboard.putBoolean("limelightEnaged", limelightEngage);

    SmartDashboard.putNumber("PoseXerror", m_poseX - m_pose2D.getX());
    SmartDashboard.putNumber("PoseYerror", m_poseY - m_pose2D.getY());
    SmartDashboard.putNumber("Gyro Yaw error", getGyroAngle() - m_pose2D.getRotation().getDegrees());

    
  }

  

  private static SwerveDriveWheel initSwerveModule(double rP, double rI, double rD, double sP, double sI, double sD, int direction_drive, int rotation_drive, int coder_drive ) {
    TalonFX directionMotor = new TalonFX(direction_drive);
    TalonFX rotationMotor = new TalonFX(rotation_drive);
    CANCoder coder = new CANCoder(coder_drive);
    return new SwerveDriveWheel(rP, rI, rD, sP, sI, sD, rotationMotor, coder, directionMotor);
  }
}