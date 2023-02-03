// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.Arm;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Arm extends SubsystemBase {
  /** Creates a new Arm. */
  private static Arm m_instance;

  public static Arm getInstance(){
    if (m_instance == null){
      m_instance = new Arm();
    }
    return m_instance;
  }
  private Arm() {
    
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}