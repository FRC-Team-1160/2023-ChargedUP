// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController.Button;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.OIConstants;
import frc.robot.subsystems.Arm.Arm;

public class SetArm extends CommandBase {
  /** Creates a new SetArm. */
  private Arm m_arm;
  private double setpoint;
  private Joystick m_firstStick = new Joystick(OIConstants.firstStickPort);
  public SetArm(Arm m_arm, double setpoint) {
    addRequirements(m_arm);
    // Use addRequirements() here to declare subsystem dependencies.
    this.m_arm = m_arm;
    this.setpoint = setpoint;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    m_arm.armPID(setpoint);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    if (m_firstStick.getRawButtonPressed(Button.kY.value)) {
      return true;
    }
    return false;
  }
}
