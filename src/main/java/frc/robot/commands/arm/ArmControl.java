// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.arm;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Arm.Arm;
import frc.robot.subsystems.Arm.Claw;
import frc.robot.Constants.OIConstants;

public class ArmControl extends CommandBase {
  /** Creates a new ArmControl. */
  private Arm m_arm;
  private double m_armInput, m_wristInput;
  private Joystick m_firstStick = new Joystick(OIConstants.firstStickPort);
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
    SmartDashboard.putNumber("arm axis input", m_firstStick.getRawAxis(1)*-1);
    if (m_arm.angle < 11 && m_claw.wristAngle < -22) {
      m_arm.armControl(0);
    } else {
      if (Math.abs(m_firstStick.getRawAxis(1)) < 0.1) {
        m_arm.armPID(currentAngle);
      } else {
        m_arm.armControl(m_armInput*m_firstStick.getRawAxis(1)*-1);
        currentAngle = m_arm.angle;
      }
    }
    SmartDashboard.putNumber("claw axis input", m_firstStick.getRawAxis(5)*-1);
    SmartDashboard.putNumber("current claw angle", currentClawAngle);
    if (Math.abs(m_firstStick.getRawAxis(5)) < 0.2) {
      if (m_arm.angle < 14 && currentClawAngle < -20+m_arm.angle && m_claw.keepClawAngle) {
        currentClawAngle = m_claw.angle;
        currentWristAngle = m_claw.wristAngle;
      } else if (m_arm.angle < 14 && currentWristAngle < -20 && !m_claw.keepClawAngle) {
        currentClawAngle = m_claw.angle;
        currentWristAngle = m_claw.wristAngle;
      }
      if (m_claw.keepClawAngle) {
        if (m_claw.m_clawSwitch.get()){

          m_claw.wristPID(currentClawAngle);
        }
      } else {
        m_claw.wristPID(currentWristAngle);
      }
    } else {
      m_claw.wristControl(m_wristInput*m_firstStick.getRawAxis(5)*-1);
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
