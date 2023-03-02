// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.swerve;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DriveTrain.DriveTrain;

import frc.robot.Constants.OIConstants;

public class SwerveDrive extends CommandBase {
  /** Creates a new SwerveDrive. */
  private double fwd;
  private double str;
  private double spd;
  private double turnspd;
  DriveTrain m_drive;
  private Joystick m_mainStick = new Joystick(OIConstants.mainStickPort);
  public SwerveDrive(DriveTrain m_drive) {
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(m_drive);
    this.m_drive = m_drive;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    str = m_mainStick.getRawAxis(0);
    fwd = -m_mainStick.getRawAxis(1);
    spd = 0.35;
    turnspd = 0.32;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    
    double joystickX = m_mainStick.getRawAxis(0);
    double joystickY = -m_mainStick.getRawAxis(1);
    double joystickBrake = m_mainStick.getRawAxis(2);

    double mag = Math.sqrt(joystickX*joystickX + joystickY*joystickY);
    if (mag > 1) {
      mag = 1;
    }

    double rot = m_mainStick.getRawAxis(4);
    double gyroAngle = Math.toRadians(m_drive.getGyroAngle());
        //field oriented
    if (mag > 0.02) {
      str = m_mainStick.getRawAxis(0);
      fwd = -m_mainStick.getRawAxis(1);
      double temp = fwd * Math.cos(gyroAngle) + str*Math.sin(gyroAngle);
      str = -1*fwd * Math.sin(gyroAngle) + str*Math.cos(gyroAngle);
      fwd = temp;
      spd = 0.35;
    } else {
      spd = 0.0001;
    }
    if (m_drive.limelightEngage) {
      m_drive.m_controller.setSwerveDrive(true, spd*fwd, spd*str, -1*m_drive.limelightEngagePID(), gyroAngle);
    } else {
      m_drive.m_controller.setSwerveDrive(true, spd*fwd, spd*str, turnspd*rot, gyroAngle);
    }
    m_drive.m_controller.brake(joystickBrake);

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
