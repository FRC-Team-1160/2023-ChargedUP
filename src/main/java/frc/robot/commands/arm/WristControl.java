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

public class WristControl extends CommandBase {
  /** Creates a new ArmControl. */
  private Claw m_claw;
  private double m_input;
  private Joystick m_firstStick = new Joystick(OIConstants.firstStickPort);
  private double currentClawAngle;
  private double currentWristAngle;
  private Arm m_arm;
  public WristControl(Claw m_claw, double input) {
    addRequirements(m_claw);
    this.m_claw = m_claw;
    m_input = input;
    currentClawAngle = m_claw.angle;
    currentWristAngle = m_claw.wristAngle;
    // Use addRequirements() here to declare subsystem dependencies.
    m_arm = Arm.getInstance();
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    currentClawAngle = m_claw.angle;
    currentWristAngle = m_claw.wristAngle;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    SmartDashboard.putNumber("claw axis input", m_firstStick.getRawAxis(5)*-1);
    SmartDashboard.putNumber("current claw angle", currentClawAngle);
    if (Math.abs(m_firstStick.getRawAxis(5)) < 0.2) {
      if (m_arm.angle < 12 && currentClawAngle < -21+m_arm.angle && m_claw.keepClawAngle) {
        currentClawAngle = m_claw.angle;
        currentWristAngle = m_claw.wristAngle;
      } else if (m_arm.angle < 12 && currentWristAngle < -21 && !m_claw.keepClawAngle) {
        currentClawAngle = m_claw.angle;
        currentWristAngle = m_claw.wristAngle;
      }
      if (m_claw.keepClawAngle) {
        if (m_claw.m_clawSwitch.get()){
          //STILL DOES WEIRD BREAK SO DO NOT RUN YET
          //MAYBE CHANGE THE PID CONTROL COMMAND SO THAT IT CAN NEVER HAVE A WRISTANGLE OF GREATER THAN 0
          m_claw.wristPID(currentClawAngle);
        }
      } else {
        m_claw.wristPID(currentWristAngle);
      }
    } else {
      m_claw.wristControl(m_input*m_firstStick.getRawAxis(5)*-1);
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
