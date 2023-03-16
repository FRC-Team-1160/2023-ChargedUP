// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.Vision;

import org.opencv.core.Mat;

import edu.wpi.first.apriltag.AprilTagDetection;
import edu.wpi.first.apriltag.AprilTagDetector;
import edu.wpi.first.apriltag.AprilTagPoseEstimate;
import edu.wpi.first.apriltag.AprilTagPoseEstimator;
import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.cscore.CvSink;
import edu.wpi.first.cscore.UsbCamera;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class AprilTag extends SubsystemBase {
  /** Creates a new AprilTag. */
  private static AprilTag m_instance;

  private AprilTagDetector detector;
  private AprilTagPoseEstimator estimator;
  private int iterations;
  private UsbCamera cam;
  
  public static AprilTag getInstance(){
    if (m_instance == null){
      m_instance = new AprilTag();
    }
    return m_instance;
  }
  public AprilTag() {
    detector.setConfig(null);
    estimator.setConfig(null);
    cam = CameraServer.startAutomaticCapture("april tag", 1);
    iterations = 50;
    
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    CvSink sink = CameraServer.getVideo("april tag");
    Mat img = new Mat();
    sink.grabFrame(img);
    AprilTagDetection[] results = detector.detect(img);
    AprilTagPoseEstimate poseEstimate = estimator.estimateOrthogonalIteration(results[0], iterations);
    Transform3d pose1 = poseEstimate.pose1;
    Transform3d pose2 = poseEstimate.pose2;
    SmartDashboard.putNumber("pose1 x", pose1.getX());
    SmartDashboard.putNumber("pose1 y", pose1.getY());
    SmartDashboard.putNumber("pose1 z", pose1.getZ());
    SmartDashboard.putNumber("pose2 x", pose2.getX());
    SmartDashboard.putNumber("pose2 y", pose2.getY());
    SmartDashboard.putNumber("pose2 z", pose2.getZ());
  }
}
