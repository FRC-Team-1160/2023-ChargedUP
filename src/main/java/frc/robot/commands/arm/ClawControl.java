// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.arm;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.OIConstants;
import frc.robot.subsystems.Arm.Piston;

import java.sql.Driver;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class ClawControl extends CommandBase {
  /** Creates a new ClawControl. */
  private Piston m_piston;
  private Joystick m_rightPanel = new Joystick(OIConstants.controlPanelRightPort);
  private boolean isSwitch;

  public ClawControl(Piston m_piston, boolean isSwitch) {
    addRequirements(m_piston);
    this.m_piston = m_piston;
    this.isSwitch = isSwitch;
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    if (!isSwitch || !DriverStation.isTeleopEnabled()) {
      m_piston.togglePiston();
    }
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (isSwitch && DriverStation.isTeleopEnabled()) {
      if (m_rightPanel.getRawButton(2)) {
        m_piston.setPiston(Value.kReverse);
      } else {
        m_piston.setPiston(Value.kForward);
      }
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
