// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.vision;

import com.pathplanner.lib.PathConstraints;
import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.PathPoint;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Arm.Intake;
import frc.robot.subsystems.DriveTrain.DriveTrain;
import frc.robot.subsystems.Vision.Vision;
import frc.robot.commands.swerve.DriveController;
import frc.robot.commands.swerve.followPath;

public class generateAndFollow extends CommandBase {
  /** Creates a new generateAndFollow. */
  PathPlannerTrajectory traj;
  Vision m_vision;
  Command followPath;
  private double poseX;
  private double poseY;
  private boolean mirrorIfRed;
  DriveTrain m_drive;
  Intake m_intake;
  private PIDController xController;
  private PIDController yController;
  private PIDController rController;
  private PathConstraints constraints;
  private boolean findObj;
  public generateAndFollow(Intake m_intake, Vision m_vision, double m_poseX, double m_poseY, PIDController xController, PIDController yController, PIDController rController, PathConstraints constraints, boolean mirrorIfRed, DriveTrain m_drive, boolean findObj) {
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(m_drive);
    addRequirements(m_vision);
    addRequirements(m_intake);
    this.m_vision = m_vision;
    this.poseX = m_poseX;
    this.poseY = m_poseY;
    this.mirrorIfRed = mirrorIfRed;
    this.m_drive = m_drive;
    this.constraints = constraints;
    this.xController = xController;
    this.yController = yController;
    this.rController = rController;
    this.m_intake = m_intake;
    this.findObj = findObj;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    if (findObj) {
      traj = m_vision.generatePathToObj(constraints);
    } else {
      traj = PathPlanner.generatePath(
        constraints,
        new PathPoint(new Translation2d(3.79, 4.64), Rotation2d.fromDegrees(-176.52), Rotation2d.fromDegrees(160)),
        new PathPoint(new Translation2d(2.3, 4.57), Rotation2d.fromDegrees(-90), Rotation2d.fromDegrees(180)),
        new PathPoint(new Translation2d(2.3, 3.8), Rotation2d.fromDegrees(90), Rotation2d.fromDegrees(180))
        );
    }
    
    if (traj == null) {
      SmartDashboard.putBoolean("generateandfollow", false);
    } else {

      SmartDashboard.putBoolean("generateandfollow", true);
      followPath = new followPath(traj, poseX, poseY, xController, yController, rController, constraints, mirrorIfRed, m_drive);
      followPath.initialize();
    }
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (traj != null) {
      followPath.execute();
      if (findObj) {
        m_intake.intakeControl(-0.7*12);
      }
      
      
    }
    
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    if (traj == null) {
      return true;
    }
    return followPath.isFinished();
  }
}
