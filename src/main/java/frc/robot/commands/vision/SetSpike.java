// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.vision;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.OIConstants;
import frc.robot.subsystems.Vision.LED;
import edu.wpi.first.wpilibj.Relay;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class SetSpike extends CommandBase {
  /** Creates a new SetSpike. */
  private LED m_LED;
  private Joystick m_rightPanel = new Joystick(OIConstants.controlPanelRightPort);
  public SetSpike(LED m_LED) {
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(m_LED);
    this.m_LED = m_LED;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (m_rightPanel.getRawButton(2)) {
      m_LED.setSpike(Relay.Value.kForward);
    } else {
      m_LED.setSpike(Relay.Value.kReverse);
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
