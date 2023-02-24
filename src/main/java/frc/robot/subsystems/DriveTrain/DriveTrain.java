/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems.DriveTrain;

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
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.SerialPort.Port;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.OIConstants;
import frc.robot.Constants.PortConstants;
import frc.robot.Constants.SwerveConstants;
import frc.robot.subsystems.Vision.Limelight;


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
  //public Pose2d m_pose;
  public boolean limelightEngage;
  private PIDController limelightEngageController;
  private double gyroOffset;

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
    m_controller = new SwerveDriveController(m_frontLeftWheel, m_frontRightWheel, m_backLeftWheel, m_backRightWheel, m_gyro);
  
    limelightEngage = false;
    limelightEngageController = new PIDController(wklP, wklI, wklD);
    
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

  private double lastgyro = 0;
  
  @Override
  public void periodic() {
    SmartDashboard.putNumber("gyro velocity" ,Math.toRadians((getGyroAngle()-lastgyro)/0.02));
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
    SmartDashboard.putNumber("x", xAngle);
    SmartDashboard.putNumber("y", yAngle);
    SmartDashboard.putNumber("Angle", angle);
    SmartDashboard.putNumber("Mag", mag);
    SmartDashboard.putNumber("Turn", turn);
    SmartDashboard.putNumber("Gyro Yaw", getGyroAngle());
    SmartDashboard.putNumber("FLCoder", m_frontLeftWheel.getRotation());
    SmartDashboard.putNumber("FRCoder", m_frontRightWheel.getRotation());
    SmartDashboard.putNumber("BLCoder", m_backLeftWheel.getRotation());
    SmartDashboard.putNumber("BRCoder", m_backRightWheel.getRotation());
    double[] odom = m_controller.getSwerveOdometry(locToAngle(getGyroAngle()));
    double fwd = odom[0];
    double str = odom[1];
    double rot = odom[2];
    Translation2d translation = new Translation2d(fwd*SwerveConstants.PERIODIC_SPEED, str*SwerveConstants.PERIODIC_SPEED);
    Transform2d transform = new Transform2d(translation, Rotation2d.fromDegrees(0));
    m_poseX += fwd*SwerveConstants.PERIODIC_SPEED;
    m_poseY += str*SwerveConstants.PERIODIC_SPEED;

    SmartDashboard.putNumber("Pose2DY", m_poseX);
    SmartDashboard.putNumber("Pose2DX", m_poseY);
    SmartDashboard.putBoolean("limelightEnaged", limelightEngage);

    /*m_controller.m_pose = m_controller.m_odometry.update(gyroAngle,
            new SwerveModulePosition[] {
              m_controller.frontLeftWheel.getModule(),
              m_controller.frontRightWheel.getModule(),
              m_controller.backLeftWheel.getModule(),
              m_controller.backRightWheel.getModule()
            });
    SmartDashboard.putNumber("Pose2DY", m_controller.m_pose.getY());
    SmartDashboard.putNumber("Pose2DX", m_controller.m_pose.getX());
    SmartDashboard.putNumber("Pose2DRotation", m_controller.m_pose.getRotation().getDegrees());
    SmartDashboard.putNumber("Gyro Rotation2D", m_gyro.getRotation2d().getDegrees());*/
    SmartDashboard.putNumber("FR wheel vel", m_frontRightWheel.getVelocity());
    lastgyro = getGyroAngle();
  }

  

  private static SwerveDriveWheel initSwerveModule(double rP, double rI, double rD, double sP, double sI, double sD, int direction_drive, int rotation_drive, int coder_drive ) {
    TalonFX directionMotor = new TalonFX(direction_drive);
    TalonFX rotationMotor = new TalonFX(rotation_drive);
    CANCoder coder = new CANCoder(coder_drive);
    return new SwerveDriveWheel(rP, rI, rD, sP, sI, sD, rotationMotor, coder, directionMotor);
  }
}