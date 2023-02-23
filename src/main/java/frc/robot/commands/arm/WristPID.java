// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.arm;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.OIConstants;
import frc.robot.subsystems.Arm.Arm;
import frc.robot.subsystems.Arm.Claw;

public class WristPID extends CommandBase {
  /** Creates a new ArmPID. */
  private Claw m_claw;
  private double setpoint;
  private Joystick m_firstStick = new Joystick(OIConstants.firstStickPort);
  public WristPID(Claw m_claw, double setpoint) {
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(m_claw);
    this.m_claw = m_claw;
    this.setpoint = setpoint;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    m_claw.wristPID(setpoint);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    
    return Math.abs(m_firstStick.getRawAxis(5)) > 0.2;
  }
}
