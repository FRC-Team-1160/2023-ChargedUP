// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.swerve;

import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.Timer;
import frc.robot.Constants.OIConstants;
import frc.robot.subsystems.DriveTrain.DriveTrain;

public class MoveTimed extends CommandBase {
  private DriveTrain m_drive;
  private Joystick m_mainStick = new Joystick(OIConstants.mainStickPort);
  private double m_fwd;
  private double m_str;
  private double time;
  private final Timer timer;
  /** Creates a new MoveUntilBalance. */
  public MoveTimed(DriveTrain m_drive, double m_fwd, double m_str){//, double time) {
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(m_drive);
    this.m_drive = m_drive;
    this.m_fwd = m_fwd;
    this.m_str = m_str;
    this.timer = new Timer();
    
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    //this.timer.reset();
    //this.timer.start();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    m_drive.m_controller.setSwerveDrive(false, m_fwd, m_str, 0, Math.toRadians(m_drive.getGyroAngle()));
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_drive.m_controller.setSwerveDrive(false, 0.1*m_fwd, 0.1*m_str, 0, Math.toRadians(m_drive.getGyroAngle()));
    //this.timer.stop();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;//this.timer.get() >= time;
  }
}
