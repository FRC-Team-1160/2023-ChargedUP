// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.Vision;

import edu.wpi.first.wpilibj.Relay;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class LED extends SubsystemBase {
  private static LED m_instance;
  private Relay spike;
  /** Creates a new LED. */
  public static LED getInstance(){
    if (m_instance == null){
      m_instance = new LED();
    }
    return m_instance;
  }
  private LED() {
    spike = new Relay(0);
  }

  public void setSpike(Relay.Value value) {
    spike.set(value);
  }
  

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    if (spike.get() == Relay.Value.kOn) {
      SmartDashboard.putString("Spike", "On");
    } else if (spike.get() == Relay.Value.kOff) {
      SmartDashboard.putString("Spike", "Off");
    } else if (spike.get() == Relay.Value.kForward){
      SmartDashboard.putString("Spike", "Cone");
    } else if (spike.get() == Relay.Value.kReverse) {
      SmartDashboard.putString("Spike", "Cube");
    }
  }
}
