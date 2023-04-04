// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.swerve;

import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants.OIConstants;
import frc.robot.subsystems.DriveTrain.DriveTrain;
import frc.robot.subsystems.Vision.Limelight;

public class LimelightAlign extends CommandBase {
  private DriveTrain m_drive;
  private Joystick m_mainStick = new Joystick(OIConstants.mainStickPort);
  private Limelight m_lime;
  PIDController m_fwdController, m_strController;
  private double fwd,str;
  /** Creates a new MoveUntilBalance. */
  public LimelightAlign(DriveTrain m_drive, Limelight m_limelight, double fwd, double str){//, double time) {
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(m_drive);
    this.m_drive = m_drive;
    this.m_lime = m_limelight;
    m_fwdController = new PIDController(1, 0.04, 0);
    m_strController = new PIDController(0.08, 0.0, 0);
    this.fwd = fwd;
    this.str = str;
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
    double outputStr = -m_strController.calculate(Limelight.getTx(), -12);
    double outputFwd = -m_fwdController.calculate(Limelight.getTy(), -5.45);
    SmartDashboard.putNumber("lime pid X", outputStr);
    SmartDashboard.putNumber("lime pid Y", outputFwd);
    if (Math.abs(outputStr) > 2) {
      outputStr = Math.signum(outputStr)*2;
    }
    if (Math.abs(outputFwd) > 1.5) {
      outputFwd = Math.signum(outputFwd)*2;
    }
    m_drive.m_controller.setSwerveDrive(false, outputFwd*fwd, outputStr*str, 0, Math.toRadians(m_drive.getGyroAngle()));
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_drive.m_controller.setSwerveDrive(false, -0.01, -0.01, 0, Math.toRadians(m_drive.getGyroAngle()));
    //this.timer.stop();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    if (fwd == 0) { //only moving left/right
      return Math.abs(Limelight.getTx() + 12) < 1;
    } else if (str == 0) { //only moving forward/backward
      return Math.abs(Limelight.getTy() + 5.45) < 0.2;
    }
    return false;//this.timer.get() >= time;
  }
}
