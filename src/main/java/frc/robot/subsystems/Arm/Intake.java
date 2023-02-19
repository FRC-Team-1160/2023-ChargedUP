// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.Arm;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.SparkMaxRelativeEncoder.Type;

import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj.PowerDistribution.ModuleType;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ArmConstants;
import frc.robot.Constants.PortConstants;
import frc.robot.Constants.ArmConstants.CLAW_STATE;

public class Intake extends SubsystemBase {
  /** Creates a new Claw. */
  private static Intake m_instance;
  private CANSparkMax m_intake;
  //private Compressor m_compressor;

  public static Intake getInstance(){
    if (m_instance == null){
      m_instance = new Intake();
    }
    return m_instance;
  }
  private Intake() {
    m_intake = new CANSparkMax(PortConstants.INTAKE, MotorType.kBrushless);
    //m_compressor = new Compressor(PneumaticsModuleType.CTREPCM);
    //m_compressor.enableDigital();
    
  }



  public void intakeControl(double input) {
    m_intake.setVoltage(input);
    /*if (Math.abs(input) > 0.1) {
      m_wrist.setVoltage(input);
    } else {
      m_wrist.stopMotor();
    }*/
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
