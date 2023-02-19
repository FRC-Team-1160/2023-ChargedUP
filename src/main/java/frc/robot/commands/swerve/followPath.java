// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.swerve;

import com.pathplanner.lib.PathConstraints;
import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.PathPlannerTrajectory.PathPlannerState;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.SwerveConstants;
import frc.robot.subsystems.DriveTrain.DriveTrain;

public class followPath extends CommandBase {
  /** Creates a new followPath. */
  private final Timer timer = new Timer();
  private PathPlannerTrajectory transformedTrajectory; 
  private PathPlannerTrajectory trajectory;
  private double poseX;
  private double poseY;
  private boolean mirrorIfRed;
  DriveTrain m_drive;
  private DriveController controller;
  private PathConstraints constraints;

  public followPath(PathPlannerTrajectory traj, double m_poseX, double m_poseY, PIDController xController, PIDController yController, PIDController rController, PathConstraints constraints, boolean mirrorIfRed, DriveTrain m_drive) {
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(m_drive);
    this.trajectory = traj;
    this.poseX = m_poseX;
    this.poseY = m_poseY;
    this.mirrorIfRed = mirrorIfRed;
    this.m_drive = m_drive;
    this.controller = new DriveController(xController, yController, rController);
    this.constraints = constraints;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    if (mirrorIfRed) {
      transformedTrajectory =
          PathPlannerTrajectory.transformTrajectoryForAlliance(
              trajectory, DriverStation.getAlliance());
    } else {
      transformedTrajectory = trajectory;
    }
    this.timer.reset();
    this.timer.start();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double currentTime = this.timer.get();
    PathPlannerState desiredState = (PathPlannerState) transformedTrajectory.sample(currentTime);

    poseX = m_drive.m_poseX;
    poseY = m_drive.m_poseY;
    SmartDashboard.putNumber("desired x", desiredState.poseMeters.getX());
    SmartDashboard.putNumber("desired y", desiredState.poseMeters.getY());


    double[] robotSpeeds = this.controller.calculate(poseX, poseY, locToAngle(m_drive.getGyroAngle()), desiredState);
    double fwd = robotSpeeds[0];
    double str = -robotSpeeds[1];
    double rot = robotSpeeds[2];
    double gyroAngle = Math.toRadians(m_drive.getGyroAngle());
    double temp = fwd * Math.cos(gyroAngle) + str*Math.sin(gyroAngle);
      str = -1*fwd * Math.sin(gyroAngle) + str*Math.cos(gyroAngle);
      fwd = temp;
    rot /= SwerveConstants.AUTO_ROTATION;
    SmartDashboard.putNumber("auto fwd", fwd);
    SmartDashboard.putNumber("auto str", str);
    SmartDashboard.putNumber("auto rot", rot);
    m_drive.m_controller.setSwerveDrive(false, fwd, str, rot, m_drive.getGyroAngle());
  }

  public double locToAngle(double ogLoc) {
    if (ogLoc > 180) {
      ogLoc -= 360;
    }
    return ogLoc;
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    this.timer.stop();
    if (interrupted) {
      m_drive.m_controller.setSwerveDrive(false, 0,0,0,m_drive.getGyroAngle());
    }
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return this.timer.hasElapsed(transformedTrajectory.getTotalTimeSeconds());
  }
}
