// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.Arm;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.PortConstants;

public class Arm extends SubsystemBase {
  /** Creates a new Arm. */
  private static Arm m_instance;
  private CANSparkMax m_armUp;
  private CANSparkMax m_armDown;
  private CANSparkMax m_claw;

  public static Arm getInstance(){
    if (m_instance == null){
      m_instance = new Arm();
    }
    return m_instance;
  }
  private Arm() {
    m_armUp = new CANSparkMax(PortConstants.ARM_UP, MotorType.kBrushless);
    m_armDown = new CANSparkMax(PortConstants.ARM_DOWN, MotorType.kBrushless);
    m_claw = new CANSparkMax(PortConstants.CLAW, MotorType.kBrushless);
  }

  public void armControl(double input) {
    m_armUp.setVoltage(input);
    m_armDown.setVoltage(-input);
  }

  public void clawControl(double input) {
    m_claw.setVoltage(input);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
