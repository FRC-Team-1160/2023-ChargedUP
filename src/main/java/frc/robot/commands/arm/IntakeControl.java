// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.arm;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Arm.Intake;
import frc.robot.Constants.OIConstants;

public class IntakeControl extends CommandBase {
  /** Creates a new ArmControl. */
  private Intake m_intake;
  private double m_input;
  private Joystick m_firstStick = new Joystick(OIConstants.firstStickPort);
  public IntakeControl(Intake m_intake, double input) {
    addRequirements(m_intake);
    this.m_intake = m_intake;
    m_input = input;
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (m_firstStick.getRawAxis(2) > 0.0001 && m_firstStick.getRawAxis(3) > 0.0001) {
      m_intake.intakeControl(0);
      SmartDashboard.putNumber("intake input", 0);
    } else {
      m_intake.intakeControl(m_input*m_firstStick.getRawAxis(2) - m_input*m_firstStick.getRawAxis(3));
      SmartDashboard.putNumber("intake input", m_input*m_firstStick.getRawAxis(2) - m_input*m_firstStick.getRawAxis(3));
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
