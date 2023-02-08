// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.Arm;

import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj.PowerDistribution.ModuleType;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ArmConstants.CLAW_STATE;

public class Claw extends SubsystemBase {
  /** Creates a new Claw. */
  private CLAW_STATE m_state;
  private static Claw m_instance;
  private DoubleSolenoid m_solenoid;
  //private Compressor m_compressor;

  public static Claw getInstance(){
    if (m_instance == null){
      m_instance = new Claw();
    }
    return m_instance;
  }
  private Claw() {
    m_state = CLAW_STATE.Closed;
    m_solenoid = new DoubleSolenoid(PneumaticsModuleType.CTREPCM, 0, 0);
    m_solenoid.set(Value.kReverse);
    //m_compressor = new Compressor(PneumaticsModuleType.CTREPCM);
    
  }

  public void toggleClaw() {
    m_solenoid.toggle();
    
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
