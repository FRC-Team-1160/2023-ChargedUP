package frc.robot;

import java.util.ArrayList;
import java.util.HashMap;
import java.util.List;

import com.pathplanner.lib.PathConstraints;
import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.PathPlannerTrajectory.StopEvent;
import com.pathplanner.lib.PathPlannerTrajectory.StopEvent.ExecutionBehavior;
import com.pathplanner.lib.commands.FollowPathWithEvents;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController.Button;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.FunctionalCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants.OIConstants;
import frc.robot.commands.arm.ArmControl;
import frc.robot.commands.arm.ArmPID;
import frc.robot.commands.arm.ClawControl;
import frc.robot.commands.arm.IntakeControl;
import frc.robot.commands.arm.ToggleClawAngle;
import frc.robot.commands.arm.WristControl;
import frc.robot.commands.arm.WristPID;
import frc.robot.commands.swerve.Reset;
import frc.robot.commands.swerve.SwerveDrive;
import frc.robot.commands.swerve.TestWheelSpeed;
import frc.robot.commands.swerve.followPath;
import frc.robot.commands.vision.LimelightEngage;
import frc.robot.commands.vision.TogglePipeline;
import frc.robot.subsystems.Arm.Arm;
import frc.robot.subsystems.Arm.Claw;
import frc.robot.subsystems.Arm.Intake;
import frc.robot.subsystems.DriveTrain.DriveTrain;
import frc.robot.subsystems.Vision.Limelight;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.POVButton;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.Subsystem;

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
    public final Intake m_intake = Intake.getInstance();
    public final Limelight m_limelight = Limelight.getInstance();
  
    // Controllers`
    private Joystick m_mainStick = new Joystick(OIConstants.mainStickPort);
    private Joystick m_firstStick = new Joystick(OIConstants.firstStickPort);

    private double xP,xI,xD,yP,yI,yD,rP,rI,rD;

    //Event map for auto
    private HashMap<String, Command> eventMap = new HashMap<>();
    //eventMap.put("intake", new Intake());



    /**
     * The container for the robot.  Contains subsystems, OI devices, and commands.
     */
    public RobotContainer() {

      xP = 0.0001;
      xI = 0.000001;
      xD = 0;
      yP = 0.0001;
      yI = 0.000001;
      yD = 0;
      rP = 0.75; //still needs to be tuned
      rI = 0.01; //still needs to be tuned
      rD = 0;

      /*
       * add commands to event map
       */
      eventMap.put("Intake Three", intake(0.5*12, 3));
      eventMap.put("Stow", stow());
      eventMap.put("Stop Intake", intake(0, 1));
      eventMap.put("Toggle Lime Pipe", new TogglePipeline());
      eventMap.put("Toggle Lime Engage", new LimelightEngage(m_driveTrain));
      eventMap.put("Toggle Claw", new ClawControl(m_claw));
      eventMap.put("Pickup", pickup());

      // Configure the button bindings
      configureButtonBindings();
  
      // Configure default commands
      m_driveTrain.setDefaultCommand(new SwerveDrive(m_driveTrain));
      m_arm.setDefaultCommand(new ArmControl(m_arm, 0.18 * 12)); //18
      m_claw.setDefaultCommand(new WristControl(m_claw, 0.25 * 12));
      m_intake.setDefaultCommand(new IntakeControl(m_intake, 0.5 * 12, true));
    }

      
  
    /**
     * Use this method to define your button->command mappings.  Buttons can be created by
     * instantiating a {@link GenericHID} or one of its subclasses ({@link
     * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a
     * {@link edu.wpi.first.wpilibj2.command.button.JoystickButton}.
     */
    
    private void configureButtonBindings() {
      //CO DRIVER
      Trigger yCoButton = new JoystickButton(m_firstStick, Button.kY.value);
      Trigger bCoButton = new JoystickButton(m_firstStick, Button.kB.value);
      Trigger aCoButton = new JoystickButton(m_firstStick, Button.kA.value);
      Trigger dTopCoButton = new POVButton(m_firstStick, 0);
      Trigger dLeftCoButton = new POVButton(m_firstStick, 270);
      Trigger dBottomCoButton = new POVButton(m_firstStick, 180);
      Trigger rbCoButton = new JoystickButton(m_firstStick, Button.kRightBumper.value);
      Trigger startCoButton = new JoystickButton(m_firstStick, Button.kStart.value);
      Trigger xCoButton = new JoystickButton(m_firstStick, Button.kX.value);
      Trigger backCoButton = new JoystickButton(m_firstStick, Button.kBack.value);
      Trigger lbCoButton = new JoystickButton(m_firstStick, Button.kLeftBumper.value);

      //MAIN DRIVER
      Trigger xButton = new JoystickButton(m_mainStick, Button.kX.value);
      Trigger aTrigger = new JoystickButton(m_mainStick, Button.kA.value);
      Trigger backButton = new JoystickButton(m_mainStick, Button.kBack.value);
      
      //CUBES
      yCoButton.onTrue(new ArmPID(m_arm, m_claw, 84));
      bCoButton.onTrue(new ArmPID(m_arm, m_claw, 70));
      aCoButton.onTrue(new ArmPID(m_arm, m_claw, 28));

      //CONES
      dTopCoButton.onTrue(new ArmPID(m_arm, m_claw, 92));
      dLeftCoButton.onTrue(new ArmPID(m_arm, m_claw, 82));
      dBottomCoButton.onTrue(new ArmPID(m_arm, m_claw, 28));

      //
      xCoButton.onTrue(new ToggleClawAngle(m_claw));

      aTrigger.toggleOnTrue(new TestWheelSpeed(m_driveTrain));

      //backCoButton.onTrue(new TogglePipeline());
      startCoButton.onTrue(new LimelightEngage(m_driveTrain));

      rbCoButton.onTrue(new ClawControl(m_claw));
      lbCoButton.onTrue(stow());


      Trigger startButton = new JoystickButton(m_mainStick, Button.kStart.value);
      startButton.onTrue(new Reset(m_driveTrain));

      //TESTERS
      backButton.onTrue(pickup());

      

    }

    /*
     * 
     * COMMANDS
     * for both auto and user
     * 
     */

    

    public Command intake(double input, double seconds) {
      return new IntakeControl(m_intake, input, false).withTimeout(seconds);
    }

    public Command stow() {
      return new ParallelCommandGroup(
        new ArmPID(m_arm, m_claw, 0),
        new WristPID(m_claw, 0)
      );
    }

    public Command pickup() {
      return new ParallelCommandGroup(
        new ArmPID(m_arm, m_claw, 16),
        new WristPID(m_claw, -69)
      );
    }

    public Command highCone() {
      return null;
    }

    public Command midCone() {
      return null;
    }

    public Command hybridCone() {
      return null;
    }

    public Command highCube() {
      return null;
    }

    public Command midCube() {
      return null;
    }

    public Command hybridCube() {
      return null;
    }



    /*
     * COMMANDS FOR USE BY AUTO COMMANDS
     * YOU DO NOT NEED TO RUN THESE
     */

    protected CommandBase resetPose(PathPlannerTrajectory trajectory) {
      return Commands.runOnce(
          () -> {
            PathPlannerTrajectory.PathPlannerState initialState = trajectory.getInitialState();
            initialState =
                PathPlannerTrajectory.transformStateForAlliance(
                    initialState, DriverStation.getAlliance());
            m_driveTrain.m_poseX = initialState.poseMeters.getX();
            m_driveTrain.m_poseY = initialState.poseMeters.getY();
            m_driveTrain.resetGyroToPosition(initialState.holonomicRotation.getDegrees());
          });
    }

    protected static CommandBase wrappedEventCommand(Command eventCommand) {
      return new FunctionalCommand(
          eventCommand::initialize,
          eventCommand::execute,
          eventCommand::end,
          eventCommand::isFinished,
          eventCommand.getRequirements().toArray(Subsystem[]::new));
    }

    protected CommandBase getStopEventCommands(StopEvent stopEvent) {
      List<CommandBase> commands = new ArrayList<>();

      int startIndex = stopEvent.executionBehavior == ExecutionBehavior.PARALLEL_DEADLINE ? 1 : 0;
      for (int i = startIndex; i < stopEvent.names.size(); i++) {
        String name = stopEvent.names.get(i);
        if (eventMap.containsKey(name)) {
          commands.add(wrappedEventCommand(eventMap.get(name)));
        }
      }

      switch (stopEvent.executionBehavior) {
        case SEQUENTIAL:
          return Commands.sequence(commands.toArray(CommandBase[]::new));
        case PARALLEL:
          return Commands.parallel(commands.toArray(CommandBase[]::new));
        case PARALLEL_DEADLINE:
          Command deadline =
              eventMap.containsKey(stopEvent.names.get(0))
                  ? wrappedEventCommand(eventMap.get(stopEvent.names.get(0)))
                  : Commands.none();
          return Commands.deadline(deadline, commands.toArray(CommandBase[]::new));
        default:
          throw new IllegalArgumentException(
              "Invalid stop event execution behavior: " + stopEvent.executionBehavior);
      }
    }

    protected CommandBase stopEventGroup(StopEvent stopEvent) {
      if (stopEvent.names.isEmpty()) {
        return Commands.waitSeconds(stopEvent.waitTime);
      }
  
      CommandBase eventCommands = getStopEventCommands(stopEvent);
  
      switch (stopEvent.waitBehavior) {
        case BEFORE:
          return Commands.sequence(Commands.waitSeconds(stopEvent.waitTime), eventCommands);
        case AFTER:
          return Commands.sequence(eventCommands, Commands.waitSeconds(stopEvent.waitTime));
        case DEADLINE:
          return Commands.deadline(Commands.waitSeconds(stopEvent.waitTime), eventCommands);
        case MINIMUM:
          return Commands.parallel(Commands.waitSeconds(stopEvent.waitTime), eventCommands);
        case NONE:
        default:
          return eventCommands;
      }
    }

    protected CommandBase followPathWithEvents(PathPlannerTrajectory trajectory, PathConstraints maxSpd) {
      Command path = new followPath(
        trajectory, 
        m_driveTrain.m_poseX,
        m_driveTrain.m_poseY, // Pose supplier
        new PIDController(xP, xI, xD),
        new PIDController(yP, yI, yD),
        new PIDController(rP, rI, rD),
        maxSpd,
        true, // Should the path be automatically mirrored depending on alliance color. Optional, defaults to true
        m_driveTrain // Requires this drive subsystem
    );
      return new FollowPathWithEvents(path, trajectory.getMarkers(), eventMap);
    }

    /*
     * 
     * AUTO COMMANDS
     * 
     */

    public Command fullAuto(PathPlannerTrajectory trajectory, PathConstraints maxSpd) {
      List<PathPlannerTrajectory> pathGroup = new ArrayList<>(List.of(trajectory));
      List<CommandBase> commands = new ArrayList<>();

      commands.add(resetPose(pathGroup.get(0)));

      for (PathPlannerTrajectory traj : pathGroup) {
        commands.add(stopEventGroup(traj.getStartStopEvent()));
        commands.add(followPathWithEvents(traj, maxSpd));
      }

      commands.add(stopEventGroup(pathGroup.get(pathGroup.size() - 1).getEndStopEvent()));

      return Commands.sequence(commands.toArray(CommandBase[]::new));
    }

    public Command followTrajectoryCommand(PathPlannerTrajectory traj, boolean isFirstPath, PathConstraints maxSpd) {
      return new SequentialCommandGroup(
          new InstantCommand(() -> {
            // Reset odometry for the first path you run during auto
            if(isFirstPath){
              
                m_driveTrain.resetOdometry(traj.getInitialHolonomicPose().getX(), traj.getInitialHolonomicPose().getY());
            }
          }),
          new followPath(
              traj, 
              m_driveTrain.m_poseX,
              m_driveTrain.m_poseY, // Pose supplier
              new PIDController(xP, xI, xD),
              new PIDController(yP, yI, yD),
              new PIDController(rP, rI, rD),
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
        PathConstraints max = new PathConstraints(1.5, 1.5);
        PathPlannerTrajectory path = PathPlanner.loadPath("3m forward intake", max);
        return fullAuto(path, max);
        //return null;
    }
    
}