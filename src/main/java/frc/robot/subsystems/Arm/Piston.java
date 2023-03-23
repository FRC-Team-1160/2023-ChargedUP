// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.Arm;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Piston extends SubsystemBase {
  /** Creates a new Piston. */
  private static Piston m_instance;
  private DoubleSolenoid m_solenoid;

  public static Piston getInstance(){
    if (m_instance == null){
      m_instance = new Piston();
    }
    return m_instance;
  }
  private Piston() {
    m_solenoid = new DoubleSolenoid(PneumaticsModuleType.CTREPCM, 0, 1);
  }


  public void togglePiston() {
    m_solenoid.toggle();
    
  }
  public void setPiston(Value input) {
    m_solenoid.set(input);
    
  }


  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    if (m_solenoid.get() == Value.kForward) {
      SmartDashboard.putString("Piston", "Forward");
    } else if (m_solenoid.get() == Value.kReverse) {
      SmartDashboard.putString("Piston", "Reverse");
    } else if (m_solenoid.get() == Value.kOff) {
      SmartDashboard.putString("Piston", "Off");
    } else {
      SmartDashboard.putString("Piston", "No data");
    }
  }
}
