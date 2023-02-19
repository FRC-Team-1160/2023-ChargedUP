// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.arm;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Arm.Claw;

import frc.robot.Constants.OIConstants;

public class WristControl extends CommandBase {
  /** Creates a new ArmControl. */
  private Claw m_claw;
  private double m_input;
  private Joystick m_firstStick = new Joystick(OIConstants.firstStickPort);
  public WristControl(Claw m_claw, double input) {
    addRequirements(m_claw);
    this.m_claw = m_claw;
    m_input = input;
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    m_claw.wristControl(m_input*m_firstStick.getRawAxis(5)*-1);
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
