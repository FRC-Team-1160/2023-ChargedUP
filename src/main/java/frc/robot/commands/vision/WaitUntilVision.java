// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.vision;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Vision.Vision;

public class WaitUntilVision extends CommandBase {
  /** Creates a new WaitUntilVision. */
  Vision m_vision;
  public WaitUntilVision(Vision m_vision) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.m_vision = m_vision;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {}

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    if (m_vision.getDistance() > 0) {
      return true;
    }
    return false;
  }
}
