// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.arm;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Arm.Claw;

public class ToggleClawAngle extends CommandBase {
  /** Creates a new toggleClawAngle. */
  private Claw m_claw;
  public ToggleClawAngle(Claw m_claw) {
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(m_claw);
    this.m_claw = m_claw;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_claw.toggleClawAngle();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {}

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return true;
  }
}
