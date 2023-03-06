// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.Vision;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Limelight extends SubsystemBase {


  /** Creates a new Limelight. */
  private static Limelight m_instance;

  private static NetworkTable table;
  public static Limelight getInstance(){
    if (m_instance == null){
      m_instance = new Limelight();
    }
    return m_instance;
  }
  private Limelight() {
    table = NetworkTableInstance.getDefault().getTable("limelight");
    table.getEntry("pipeline").setNumber(0);
  }

  public static void changePipeline(int pipeline) {
    table.getEntry("pipeline").setNumber(pipeline);
  }
  public static Number getPipeline() {
    return table.getEntry("pipeline").getNumber(0);
  }

  public static double getTx() {
    return table.getEntry("tx").getDouble(0);
  }

  public static boolean getTv() {
    return table.getEntry("tv").getInteger(0) == 1;
  }


  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    SmartDashboard.putNumber("tx", getTx());
    SmartDashboard.putNumber("Pipeline", getPipeline().intValue());
    SmartDashboard.putBoolean("tv", getTv());
  }
}
