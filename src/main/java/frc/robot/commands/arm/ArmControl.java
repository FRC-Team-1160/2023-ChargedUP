// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.arm;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Arm.Arm;
import frc.robot.subsystems.Arm.Claw;
import frc.robot.Constants.ArmConstants;
import frc.robot.Constants.OIConstants;

public class ArmControl extends CommandBase {
  /** Creates a new ArmControl. */
  private Arm m_arm;
  private double m_armInput, m_wristInput;
  private Joystick m_leftPanel = new Joystick(OIConstants.controlPanelLeftPort);
  private Joystick m_rightPanel = new Joystick(OIConstants.controlPanelRightPort);
  private double currentAngle;
  private Claw m_claw;
  private double currentClawAngle;
  private double currentWristAngle;
  public ArmControl(Arm m_arm, Claw m_claw, double armInput, double wristInput) {
    addRequirements(m_arm);
    this.m_arm = m_arm;
    m_armInput = armInput;
    if (m_armInput < 0) {
      m_armInput = 0;
    }
    m_wristInput = wristInput;
    currentAngle = m_arm.angle;
    this.m_claw = m_claw;
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    currentAngle = m_arm.angle;
    currentClawAngle = m_claw.angle;
    currentWristAngle = m_claw.wristAngle;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    SmartDashboard.putNumber("arm axis input", m_leftPanel.getRawAxis(0));

    if (m_arm.angle < ArmConstants.ARM_BUMPER_SAFETY+1 && m_claw.wristAngle < -22) {
      m_arm.armControl(0);
    } else {
      if (Math.abs(m_leftPanel.getRawAxis(0)) < 0.1) {
        m_arm.armPID(currentAngle);
      } else {
        m_arm.armControl(m_armInput*m_leftPanel.getRawAxis(0)*-1);
        currentAngle = m_arm.angle;
      }
    }
    SmartDashboard.putNumber("claw axis input", m_rightPanel.getRawAxis(5));
    SmartDashboard.putNumber("current claw angle", currentClawAngle);
    SmartDashboard.putNumber("current wrist angle", currentWristAngle);

    if (Math.abs(m_rightPanel.getRawAxis(0)) < 0.2) {
      if (m_arm.angle < ArmConstants.ARM_BUMPER_SAFETY+3 && currentClawAngle < ArmConstants.WRIST_BUMPER_SAFETY+m_arm.angle && m_claw.keepClawAngle) {
        currentClawAngle = m_claw.angle;
        currentWristAngle = m_claw.wristAngle;
      } else if (m_arm.angle < ArmConstants.ARM_BUMPER_SAFETY+3 && currentWristAngle < ArmConstants.WRIST_BUMPER_SAFETY && !m_claw.keepClawAngle) {
        currentClawAngle = m_claw.angle;
        currentWristAngle = m_claw.wristAngle;
      }
      if (m_claw.keepClawAngle) {
        if (m_claw.m_clawSwitch.get()){

          m_claw.wristPID(currentClawAngle);
        }
      } else {
        SmartDashboard.putBoolean("wrist PID active", true);
        m_claw.wristPID(currentWristAngle);
      }
    } else {
      m_claw.wristControl(m_wristInput*m_rightPanel.getRawAxis(0)*-1);
      SmartDashboard.putBoolean("wrist PID active", false);
      currentClawAngle = m_claw.angle;
      currentWristAngle = m_claw.wristAngle;
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
