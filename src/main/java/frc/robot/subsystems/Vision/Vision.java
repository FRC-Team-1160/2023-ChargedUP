// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.Vision;

import java.util.Optional;

import com.pathplanner.lib.PathConstraints;
import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.PathPoint;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.VisionConstants;
import frc.robot.subsystems.DriveTrain.DriveTrain;
import frc.robot.subsystems.DriveTrain.Pose;

public class Vision extends SubsystemBase {
  /** Creates a new Vision. */
  private static NetworkTable table;
  private static Vision m_instance;
  public PathPlannerTrajectory traj;
  public static Vision getInstance(){
    if (m_instance == null){
      m_instance = new Vision();
    }
    return m_instance;
  }
  private Vision() {
    table = NetworkTableInstance.getDefault().getTable("vision");
  }

  public double[] getRelativeObjectPose() {
    Number[] def = {-1,-1};
    double[] pose = {-1, -1};
    double u1 = table.getEntry("offset").getNumberArray(def)[0].doubleValue();
    double u2 = table.getEntry("distance").getNumberArray(def)[0].doubleValue();
    if (u2 == -1) {
      return pose;
    }

//these two are constants that we will set in Constants.java
    double camAngle = VisionConstants.CAM_ANGLE;
    double[][] camOffsetMatrix = VisionConstants.CAM_OFFSET; //horizontal, vertical

    double[][] rotMatrix = {{Math.cos(camAngle), -Math.sin(camAngle)}, {Math.sin(camAngle), Math.cos(camAngle)}};
    double[][] cMatrix = {{u1},{u2}};
    double[][] rMatrix = Matrix.add(camOffsetMatrix, Matrix.multiply(rotMatrix, cMatrix));
    pose[0] = rMatrix[1][0];
    pose[1] = rMatrix[0][0]; //flipped around because x is vertical and y is horizontal
    return pose;
    
  }

  public double[] getFieldObjectPose() {
    double[] rPose = getRelativeObjectPose();
    double[] pose = {-1, -1};
    if (rPose[0] == -1 && rPose[1] == -1) {
      return pose;
    }
    double[][] matrix = {{rPose[0]}, {rPose[1]}};
    double angle = DriveTrain.getInstance().getGyroAngle();
    double[][] rotMatrix = {{Math.cos(angle), -Math.sin(angle)}, {Math.sin(angle), Math.cos(angle)}};
    double execTime = table.getEntry("execTime").getNumber(-1).doubleValue();
    if (execTime == -1) {
      return pose;
    }
    DriveTrain m_drive = DriveTrain.getInstance();
    Pose pastPose = m_drive.getPastPose(execTime/1000);
    if (pastPose != null) {
    
      double driveX = pastPose.x; //THESE WILL BE FROM POSE HISTORY
      double driveY = pastPose.y; //THESE WILL BE FROM POSE HISTORY
      double[][] offsetMatrix = {{driveX}, {driveY}};
      double[][] rMatrix = Matrix.add(offsetMatrix, Matrix.multiply(rotMatrix, matrix));
      pose[0] = rMatrix[0][0];
      pose[1] = rMatrix[1][0];
    }
    return pose;
  }

  public PathPlannerTrajectory generatePathToObj(PathConstraints max) {
    double[] pos = getFieldObjectPose();
    double xPos = pos[0];
    double yPos = pos[1];
    double execTime = table.getEntry("execTime").getNumber(-1).doubleValue();
    if (execTime == -1) {
      return null;
    }
    DriveTrain m_drive = DriveTrain.getInstance();
    Pose pastPose = m_drive.getPastPose(execTime/1000);
    if (pastPose == null) {
      return null;
    } else {
    
    double driveX = pastPose.x; //THESE WILL BE FROM POSE HISTORY
    double driveY = pastPose.y; //THESE WILL BE FROM POSE HISTORY
    double xDiff = xPos - driveX;
    double yDiff = yPos - driveY;
    double angle = Math.atan((yDiff)/(xDiff));
    SmartDashboard.putNumber("objectGyroAngle", angle);
    PathPoint turnToObject = new PathPoint(new Translation2d(m_drive.m_poseX, m_drive.m_poseY), Rotation2d.fromRadians(angle), Rotation2d.fromRadians(angle));
    double d = Math.sqrt((xDiff*xDiff) + (yDiff*yDiff));
    double fX = xDiff-((VisionConstants.INTAKE_OFFSET*xDiff)/d);
    double fY = yDiff-((VisionConstants.INTAKE_OFFSET*yDiff)/d);
    PathPoint goToObject = new PathPoint(new Translation2d(driveX+fX, driveY+fY), Rotation2d.fromRadians(angle), Rotation2d.fromRadians(angle));
    return PathPlanner.generatePath(
      max,
      turnToObject,
      turnToObject
      );
    }
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    Number[] def = {-1,-1};
    SmartDashboard.putNumber("distance", table.getEntry("distance").getNumberArray(def)[0].doubleValue());
    SmartDashboard.putNumber("offset", table.getEntry("offset").getNumberArray(def)[0].doubleValue());
    SmartDashboard.putNumber("execTime", table.getEntry("execTime").getNumber(-1).doubleValue());
    SmartDashboard.putNumber("objRelativeXpos", getRelativeObjectPose()[0]);
    SmartDashboard.putNumber("objRelativeYpos", getRelativeObjectPose()[1]);
    SmartDashboard.putNumber("objFieldXpos", getFieldObjectPose()[0]);
    SmartDashboard.putNumber("objFieldYpos", getFieldObjectPose()[1]);
    if (DriveTrain.getInstance().getPastPose(table.getEntry("execTime").getNumber(-1).doubleValue()/1000) != null) {
      SmartDashboard.putNumber("past pose x", DriveTrain.getInstance().getPastPose(table.getEntry("execTime").getNumber(-1).doubleValue()/1000).x);
      traj = generatePathToObj(new PathConstraints(1, 1));
    } else {
      SmartDashboard.putNumber("past pose x", -1);
    }
    
  }
}