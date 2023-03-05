// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.Vision;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

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
    //table = NetworkTableInstance.getDefault().getTable("vision");
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    Number[] def = {-1,-1};
    //SmartDashboard.putNumber("vision", table.getEntry("distance").getNumberArray(def)[0].doubleValue());
  }
}