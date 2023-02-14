package frc.robot;

import com.pathplanner.lib.PathConstraints;
import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController.Button;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants.OIConstants;
import frc.robot.commands.ArmControl;
import frc.robot.commands.ClawControl;
import frc.robot.commands.WristControl;
import frc.robot.commands.Reset;
import frc.robot.commands.SwerveDrive;
import frc.robot.commands.TestWheelSpeed;
import frc.robot.commands.followPath;
import frc.robot.subsystems.Arm.Arm;
import frc.robot.subsystems.Arm.Claw;
import frc.robot.subsystems.DriveTrain.DriveTrain;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

// Commands


/**
 * This class is where the bulk of the robot should be declared. Since
 * Command-based is a "declarative" paradigm, very little robot logic should
 * actually be handled in the {@link Robot} periodic methods (other than the
 * scheduler calls). Instead, the structure of the robot (including subsystems,
 * commands, and button mappings) should be declared here.
 */
public class RobotContainer {

    // The robot's subsystems
    public final Arm m_arm = Arm.getInstance();
    public final DriveTrain m_driveTrain = DriveTrain.getInstance(); 
    public final Claw m_claw = Claw.getInstance();
  
    // Controllers`
    private Joystick m_mainStick = new Joystick(OIConstants.mainStickPort);
    private Joystick m_firstStick = new Joystick(OIConstants.firstStickPort);



    /**
     * The container for the robot.  Contains subsystems, OI devices, and commands.
     */
    public RobotContainer() {

      // Configure the button bindings
      configureButtonBindings();
  
      // Configure default commands
      m_driveTrain.setDefaultCommand(new SwerveDrive(m_driveTrain));
      m_arm.setDefaultCommand(new ArmControl(m_arm, 0.18 * 12)); //18
      m_claw.setDefaultCommand(new WristControl(m_claw, 0.25 * 12));

    }

      
  
    /**
     * Use this method to define your button->command mappings.  Buttons can be created by
     * instantiating a {@link GenericHID} or one of its subclasses ({@link
     * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a
     * {@link edu.wpi.first.wpilibj2.command.button.JoystickButton}.
     */
    
    private void configureButtonBindings() {
      Trigger yButton = new JoystickButton(m_firstStick, Button.kY.value);
      Trigger xButton = new JoystickButton(m_firstStick, Button.kX.value);
      Trigger lbButton = new JoystickButton(m_firstStick, Button.kRightBumper.value);
      Trigger aTrigger = new JoystickButton(m_mainStick, Button.kA.value);
      aTrigger.toggleOnTrue(new TestWheelSpeed(m_driveTrain));

      lbButton.onTrue(new ClawControl(m_claw));


      Trigger startButton = new JoystickButton(m_mainStick, Button.kStart.value);
      startButton.onTrue(new Reset(m_driveTrain));

      

    }

    /*
     * COMMANDS
     */
    public Command followTrajectoryCommand(PathPlannerTrajectory traj, boolean isFirstPath, PathConstraints maxSpd) {
      return new SequentialCommandGroup(
          new InstantCommand(() -> {
            // Reset odometry for the first path you run during auto
            if(isFirstPath){
                m_driveTrain.resetOdometry(traj.getInitialHolonomicPose());
            }
          }),
          new followPath(
              traj, 
              m_driveTrain.m_pose, // Pose supplier
              new PIDController(0.0001, 0.000001, 0),
              new PIDController(0.0001, 0.000001, 0),
              new PIDController(0.0001, 0.000001, 0),
              maxSpd,
              true, // Should the path be automatically mirrored depending on alliance color. Optional, defaults to true
              m_driveTrain // Requires this drive subsystem
          )
      );
    }


    /**
     * Use this to pass the autonomous command to the main {@link Robot} class.
     *
     * @return the command to run in autonomous
     */

    
    public Command getAutonomousCommand() {
        PathConstraints max = new PathConstraints(1, 1);
        PathPlannerTrajectory path = PathPlanner.loadPath("3m forward rotating", max);
        return followTrajectoryCommand(path, true, max);
        //return null;
    }
    
}