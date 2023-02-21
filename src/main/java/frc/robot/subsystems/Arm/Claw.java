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

public class Claw extends SubsystemBase {
  /** Creates a new Claw. */
  private static Claw m_instance;
  private DoubleSolenoid m_solenoid;
  private CANSparkMax m_wrist;
  private RelativeEncoder m_encoder;
  //private Compressor m_compressor;

  public static Claw getInstance(){
    if (m_instance == null){
      m_instance = new Claw();
    }
    return m_instance;
  }
  private Claw() {
    m_solenoid = new DoubleSolenoid(PneumaticsModuleType.CTREPCM, 0, 1);
    m_solenoid.set(Value.kReverse);
    m_wrist = new CANSparkMax(PortConstants.WRIST, MotorType.kBrushless);
    m_encoder = m_wrist.getEncoder(Type.kHallSensor, 42);
    //m_compressor = new Compressor(PneumaticsModuleType.CTREPCM);
    //m_compressor.enableDigital();
    
  }

  public void toggleClaw() {
    m_solenoid.toggle();
    
  }


  public void wristControl(double input) {
    m_wrist.setVoltage(input);
    /*if (Math.abs(input) > 0.1) {
      m_wrist.setVoltage(input);
    } else {
      m_wrist.stopMotor();
    }*/
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    if (m_solenoid.get() == Value.kForward) {
      SmartDashboard.putString("Claw", "Forward");
    } else if (m_solenoid.get() == Value.kReverse) {
      SmartDashboard.putString("Claw", "Reverse");
    } else if (m_solenoid.get() == Value.kOff) {
      SmartDashboard.putString("Claw", "Off");
    } else {
      SmartDashboard.putString("Claw", "No data");
    }
    SmartDashboard.putNumber("claw encoder", m_encoder.getPosition());
    /*
     * 0 is perpendicular to arm
     * -7.5 is parallel
     * -15 is perpendicular backwards
     * 5 is before it hits the motor
     */
  }
}
