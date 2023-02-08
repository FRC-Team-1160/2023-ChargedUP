// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Arm.Arm;

import frc.robot.Constants.OIConstants;

public class WristControl extends CommandBase {
  /** Creates a new ArmControl. */
  private Arm m_arm;
  private double m_input;
  private Joystick m_mainStick = new Joystick(OIConstants.mainStickPort);
  public WristControl(Arm m_arm, double input) {
    addRequirements(m_arm);
    this.m_arm = m_arm;
    m_input = input;
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    m_arm.clawControl(m_input*m_mainStick.getRawAxis(5));
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
