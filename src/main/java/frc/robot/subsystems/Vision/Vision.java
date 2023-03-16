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

  public double[] getObjPosition() {
    String[] sdef = {"null"};
    Number[] def = {-1,-1};
    double[] ddef = {0, 0};
    if (table.getEntry("xPos").getStringArray(sdef)[0].equals("null")) {
      return ddef;
    }
    double xCoord = Double.parseDouble(table.getEntry("xPos").getStringArray(sdef)[0]);
    double distance = table.getEntry("distance").getNumberArray(def)[0].doubleValue();
    double robotAngle = DriveTrain.getInstance().getGyroAngle();
    
    double theta = Math.toRadians(VisionConstants.CAM_ANGLE) - (Math.toRadians(VisionConstants.CAM_HORIZONTAL_FOV) * (xCoord - 0.5*VisionConstants.STREAM_WIDTH_PIXELS)/(0.5*VisionConstants.STREAM_WIDTH_PIXELS));

    double frontObjDist = Math.sqrt(VisionConstants.CAM_OFFSET*VisionConstants.CAM_OFFSET + distance*distance - 2*VisionConstants.CAM_OFFSET*distance*Math.cos(theta));
    double frontObjAng = Math.asin(distance*Math.sin(theta)/frontObjDist) - Math.PI/2 + Math.toRadians(robotAngle);
    
    double objX = frontObjDist*Math.cos(frontObjAng);
    double objY = frontObjDist*Math.sin(frontObjAng);
    SmartDashboard.putNumber("objrelativeX", objX);
    
    double rX = VisionConstants.ROBOT_RADIUS*Math.cos(Math.toRadians(robotAngle));
    double rY = VisionConstants.ROBOT_RADIUS*Math.sin(Math.toRadians(-robotAngle));
    
    //this should be the final positions of the object on the field
    double xPos = DriveTrain.getInstance().m_poseX + objX + rX;
    double yPos = DriveTrain.getInstance().m_poseY + objY + rY;
    double[] pos = {xPos, yPos};
    return pos;
  }

  public PathPlannerTrajectory generatePathToObj(PathConstraints max) {
    double[] pos = getObjPosition();
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
    String[] sdef = {"null"};
    SmartDashboard.putNumber("vision", table.getEntry("distance").getNumberArray(def)[0].doubleValue());
    SmartDashboard.putString("obj", table.getEntry("obj").getStringArray(sdef)[0]);
    SmartDashboard.putString("objXPX", table.getEntry("xPos").getStringArray(sdef)[0]);
    SmartDashboard.putNumber("objXpos", getObjPosition()[0]);
    SmartDashboard.putNumber("objYpos", getObjPosition()[1]);
  }
}