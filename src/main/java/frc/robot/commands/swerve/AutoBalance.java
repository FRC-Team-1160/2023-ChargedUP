// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.swerve;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.OIConstants;
import frc.robot.subsystems.DriveTrain.DriveTrain;

public class AutoBalance extends CommandBase {
  /** Creates a new AutoBalance. */
  private DriveTrain m_drive;
  private Joystick m_mainStick = new Joystick(OIConstants.mainStickPort);
  private boolean dir;
  private final Timer timer = new Timer();
  public AutoBalance(DriveTrain m_drive) {
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(m_drive);
    this.m_drive = m_drive;
    dir = false;
    timer.reset();
    timer.stop();
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (m_drive.getGyroPitch() < -11) {
      timer.reset();
      timer.stop();
      m_drive.m_controller.setSwerveDrive(false, -0.5, 0, 0, m_drive.getGyroAngle());
      dir = false;
    } else if (m_drive.getGyroPitch() > 11) {
      timer.reset();
      timer.stop();
      m_drive.m_controller.setSwerveDrive(false, 0.5, 0, 0, m_drive.getGyroAngle());
      dir = true;
    } else {
      
      timer.start();
      if (timer.get() < 0.5) {
        if (dir) {
          m_drive.m_controller.setSwerveDrive(false, -0.25, 0, 0, m_drive.getGyroAngle());

        } else {
          m_drive.m_controller.setSwerveDrive(false, 0.25, 0, 0, m_drive.getGyroAngle());

        }
      } else {
        timer.stop();
        m_drive.m_controller.setSwerveDrive(false, 0.01, 0, 0, m_drive.getGyroAngle());

      }
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    if (Math.abs(m_mainStick.getRawAxis(1)) > 0.25) {
      return true;
    }
    return false;
  }
}
