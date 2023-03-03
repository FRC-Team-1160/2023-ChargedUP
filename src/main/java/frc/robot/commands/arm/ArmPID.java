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
  private Joystick m_firstStick = new Joystick(OIConstants.firstStickPort);
  private boolean keptClawAngle;
  public ArmPID(Arm m_arm, Claw m_claw, double armSetpoint, double wristSetpoint) {
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(m_arm);
    this.m_arm = m_arm;
    this.armSetpoint = armSetpoint;
    addRequirements(m_claw);
    this.m_claw = m_claw;
    this.wristSetpoint = wristSetpoint;
    this.keptClawAngle = false;
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
    if (m_arm.angle < ArmConstants.ARM_BUMPER_SAFETY && m_claw.wristAngle < ArmConstants.WRIST_BUMPER_SAFETY) {
      m_arm.armControl(0);
    } else {
      m_arm.armPID(armSetpoint);
    }
    m_claw.wristPID(wristSetpoint);
    
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_claw.keepClawAngle = keptClawAngle;
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return Math.abs(m_firstStick.getRawAxis(1)) > 0.1 || Math.abs(m_firstStick.getRawAxis(5)) > 0.2;
  }
}
