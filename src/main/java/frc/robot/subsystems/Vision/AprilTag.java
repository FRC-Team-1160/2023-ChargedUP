// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.Vision;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.cscore.UsbCamera;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.AprilTagConstants;
import frc.robot.Constants.VisionConstants;

import java.io.IOException;
import java.util.Optional;

import org.photonvision.*;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;

public class AprilTag extends SubsystemBase {
  /** Creates a new AprilTag. */
  private static AprilTag m_instance;

  private PhotonCamera photonCamera;
  private PhotonPoseEstimator photonPoseEstimator;

  
  public static AprilTag getInstance(){
    if (m_instance == null){
      m_instance = new AprilTag();
    };
    return m_instance;
  }

  public AprilTag() {
    photonCamera = new PhotonCamera("limelight");
    try {
        // Attempt to load the AprilTagFieldLayout that will tell us where the tags are on the field.
        AprilTagFieldLayout fieldLayout = AprilTagFields.k2023ChargedUp.loadAprilTagLayoutField();
        // Create pose estimator
        photonPoseEstimator =
                new PhotonPoseEstimator(
                        fieldLayout, PoseStrategy.MULTI_TAG_PNP, photonCamera, AprilTagConstants.ROBOT_TO_CAM);
        photonPoseEstimator.setMultiTagFallbackStrategy(PoseStrategy.LOWEST_AMBIGUITY);
    } catch (IOException e) {
        // The AprilTagFieldLayout failed to load. We won't be able to estimate poses if we don't know
        // where the tags are.
        DriverStation.reportError("Failed to load AprilTagFieldLayout", e.getStackTrace());
        photonPoseEstimator = null;
    }
  }
  /**
     * @param estimatedRobotPose The current best guess at robot pose
     * @return an EstimatedRobotPose with an estimated pose, the timestamp, and targets used to create
     *     the estimate
     */
    public Optional<EstimatedRobotPose> getEstimatedGlobalPose(Pose2d prevEstimatedRobotPose) {
      if (photonPoseEstimator == null) {
          // The field layout failed to load, so we cannot estimate poses.
          return Optional.empty();
      }
      photonPoseEstimator.setReferencePose(prevEstimatedRobotPose);
      return photonPoseEstimator.update();
  }

  @Override
  public void periodic() {
    SmartDashboard.putBoolean("has targets", photonCamera.getLatestResult().hasTargets());
  }
}
