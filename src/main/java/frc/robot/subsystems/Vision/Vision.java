// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.Vision;

import com.pathplanner.lib.PathConstraints;
import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.VisionConstants;
import frc.robot.subsystems.DriveTrain.DriveTrain;

public class Vision extends SubsystemBase {
  /** Creates a new Vision. */
  private static NetworkTable table;
  private static Vision m_instance;
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
    double u1 = table.getEntry("offset").getNumberArray(def)[0].doubleValue();
    double u2 = table.getEntry("distance").getNumberArray(def)[0].doubleValue();
    if (u2 == -1) {
      return null;
    }

//these two are constants that we will set in Constants.java
    double camAngle = VisionConstants.CAM_ANGLE;
    double[][] camOffsetMatrix = VisionConstants.CAM_OFFSET; //horizontal, vertical

    double[][] rotMatrix = {{Math.cos(camAngle), -Math.sin(camAngle)}, {Math.sin(camAngle), Math.cos(camAngle)}};
    double[][] cMatrix = {{u1},{u2}};
    double[][] rMatrix = Matrix.add(camOffsetMatrix, Matrix.multiply(rotMatrix, cMatrix));
    double[] pose = {rMatrix[1][0], rMatrix[0][0]}; //flipped around because x is vertical and y is horizontal
    return pose;
    
  }

  public double[] getFieldObjectPose() {
    double[] rPose = getRelativeObjectPose();
    double[][] matrix = {{rPose[0]}, {rPose[0]}};
    double angle = -DriveTrain.getInstance().getGyroAngle();
    double[][] rotMatrix = {{Math.cos(angle), -Math.sin(angle)}, {Math.sin(angle), Math.cos(angle)}};
    double[][] offsetMatrix = {{DriveTrain.getInstance().m_poseX}, {DriveTrain.getInstance().m_poseY}};
    double[][] rMatrix = Matrix.add(offsetMatrix, Matrix.multiply(rotMatrix, matrix));
    double[] pose = {rMatrix[0][0], rMatrix[1][0]};
    return pose;
  }

  public PathPlannerTrajectory generatePathToObj(PathConstraints max) {
    double[] pos = getRelativeObjectPose();
    double xPos = pos[0];
    double yPos = pos[1];
    return PathPlanner.generatePath(
      max,
      null,
      null,
      null,
      null
      );
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    Number[] def = {-1,-1};
    SmartDashboard.putNumber("distance", table.getEntry("distance").getNumberArray(def)[0].doubleValue());
    SmartDashboard.putNumber("offset", table.getEntry("offset").getNumberArray(def)[0].doubleValue());
    SmartDashboard.putNumber("objRelativeXpos", getRelativeObjectPose()[0]);
    SmartDashboard.putNumber("objRelativeYpos", getRelativeObjectPose()[1]);
    SmartDashboard.putNumber("objFieldXpos", getRelativeObjectPose()[0]);
    SmartDashboard.putNumber("objFieldYpos", getRelativeObjectPose()[1]);
  }
}