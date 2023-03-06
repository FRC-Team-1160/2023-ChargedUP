// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.arm;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.ArmConstants;
import frc.robot.Constants.OIConstants;
import frc.robot.subsystems.Arm.Arm;
import frc.robot.subsystems.Arm.Claw;

public class ArmPID extends CommandBase {
  /** Creates a new ArmPID. */
  private Arm m_arm;
  private Claw m_claw;
  private double armSetpoint, wristSetpoint;
  private Joystick m_leftPanel = new Joystick(OIConstants.controlPanelLeftPort);
  private Joystick m_rightPanel = new Joystick(OIConstants.controlPanelRightPort);
  private boolean keptClawAngle;
  private boolean hitSwitch;
  public ArmPID(Arm m_arm, Claw m_claw, double armSetpoint, double wristSetpoint, boolean hitSwitch) {
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(m_arm);
    this.m_arm = m_arm;
    this.armSetpoint = armSetpoint;
    addRequirements(m_claw);
    this.m_claw = m_claw;
    this.wristSetpoint = wristSetpoint;
    this.keptClawAngle = false;
    this.hitSwitch = hitSwitch;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    keptClawAngle = m_claw.keepClawAngle;
    m_claw.keepClawAngle = false;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    SmartDashboard.putBoolean("ARM PID ACTIVE", true);
    if (m_arm.angle < ArmConstants.ARM_BUMPER_SAFETY && m_claw.wristAngle < ArmConstants.WRIST_BUMPER_SAFETY) {
      m_arm.armControl(0);
    } else {
      if (m_arm.m_armSwitch.get() || !hitSwitch) {
        m_arm.armPID(armSetpoint);
      } else {
        m_arm.armControl(0);
      }
      
    }
    if (m_claw.m_clawSwitch.get() || !hitSwitch) {
      m_claw.wristPID(wristSetpoint);
    } else {
      m_claw.wristControl(0);
    }
    
    
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_claw.keepClawAngle = keptClawAngle;
    SmartDashboard.putBoolean("ARM PID ACTIVE", false);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return Math.abs(m_leftPanel.getRawAxis(0)) > 0.1 || Math.abs(m_rightPanel.getRawAxis(0)) > 0.2;
  }
}
