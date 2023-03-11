// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.swerve;

import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj.Joystick;
import frc.robot.Constants.OIConstants;
import frc.robot.subsystems.DriveTrain.DriveTrain;

public class MoveUntilBalance extends CommandBase {
  private DriveTrain m_drive;
  private Joystick m_mainStick = new Joystick(OIConstants.mainStickPort);
  private double m_input;
  /** Creates a new MoveUntilBalance. */
  public MoveUntilBalance(DriveTrain m_drive, double m_input) {
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(m_drive);
    this.m_drive = m_drive;
    this.m_input = m_input;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    m_drive.m_controller.setSwerveDrive(false, m_input, 0, 0, m_drive.getGyroAngle());
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return Math.abs(m_drive.getGyroPitch()) > 11;
  }
}
