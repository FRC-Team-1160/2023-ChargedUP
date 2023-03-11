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
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.XboxController.Button;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.FunctionalCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.Subsystem;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.POVButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants.OIConstants;
import frc.robot.commands.arm.ArmControl;
import frc.robot.commands.arm.ArmPID;
import frc.robot.commands.arm.ClawControl;
import frc.robot.commands.arm.IntakeControl;
import frc.robot.commands.arm.ToggleClawAngle;
import frc.robot.commands.swerve.AutoBalance;
import frc.robot.commands.swerve.MoveUntilBalance;
import frc.robot.commands.swerve.Reset;
import frc.robot.commands.swerve.SwerveDrive;
import frc.robot.commands.swerve.TestWheelSpeed;
import frc.robot.commands.swerve.followPath;
import frc.robot.commands.vision.LimelightEngage;
import frc.robot.commands.vision.SetSpike;
import frc.robot.commands.vision.TogglePipeline;
import frc.robot.subsystems.Arm.Arm;
import frc.robot.subsystems.Arm.Claw;
import frc.robot.subsystems.Arm.Intake;
import frc.robot.subsystems.DriveTrain.DriveTrain;
import frc.robot.subsystems.Vision.LED;
import frc.robot.subsystems.Vision.Limelight;
import frc.robot.subsystems.Vision.Vision;

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
    public final Vision m_vision = Vision.getInstance();
    public final LED m_LED = LED.getInstance();
  
    // Controllers`
    private Joystick m_mainStick = new Joystick(OIConstants.mainStickPort);
    private Joystick m_leftPanel = new Joystick(OIConstants.controlPanelLeftPort);
    private Joystick m_rightPanel = new Joystick(OIConstants.controlPanelRightPort);

    private double xP,xI,xD,yP,yI,yD,rP,rI,rD;

    //Event map for auto
    private HashMap<String, Command> eventMap = new HashMap<>();
    //eventMap.put("intake", new Intake());

    SendableChooser<Command> m_chooser = new SendableChooser<>();

    /**
     * The container for the robot.  Contains subsystems, OI devices, and commands.
     */
    public RobotContainer() {

      xP = 1.9;
      xI = 0;
      xD = 0.1;
      yP = 1.9;
      yI = 0;
      yD = 0.1;
      rP = 0.8; //still needs to be tuned
      rI = 0; //still needs to be tuned
      rD = 0;



      SmartDashboard.putData("autonomous", m_chooser);
      /*
       * add commands to event map
       */
      eventMap.put("Intake Three", intake(0.5*12, 3));
      eventMap.put("Stow", stow());
      eventMap.put("Stop Intake", intake(0, 1));
      eventMap.put("Toggle Lime Pipe", new TogglePipeline());
      eventMap.put("High Cube", highCube());
      eventMap.put("Toggle Claw", toggleClaw());
      eventMap.put("Toggle Lime Engage", new LimelightEngage(m_driveTrain));
      eventMap.put("Toggle Claw", new ClawControl(m_claw));
      eventMap.put("Pickup", pickup());
      eventMap.put("End", end());
      eventMap.put("High Cone", highCone());
      eventMap.put("Auto Balance", autoBalance());

      /*
       * AUTO ROUTINES
       */
      m_chooser.setDefaultOption("cube left", getPathCommand("cube left", 3.5, 2.5));
      m_chooser.addOption("cube middle", getPathCommand("cube middle", 3.5, 2.5));
      m_chooser.addOption("cube right", getPathCommand("cube right", 3.5, 2.5));
      m_chooser.addOption("cube left charge left", getPathCommand("cube left charge left", 3.5, 2.5));
      m_chooser.addOption("cube middle charge left", getPathCommand("cube middle charge left", 3.5, 2.5));
      m_chooser.addOption("cube middle charge right", getPathCommand("cube middle charge right", 3.5, 2.5));
      m_chooser.addOption("cube right charge right", getPathCommand("cube right charge right", 3.5, 2.5));
      m_chooser.addOption("cone left left", getPathCommand("cone left left", 3.5, 2.5));
      m_chooser.addOption("cone left right", getPathCommand("cone left right", 3.5, 2.5));
      m_chooser.addOption("cone middle right", getPathCommand("cone middle right", 3.5, 2.5));
      m_chooser.addOption("cone right right", getPathCommand("cone right right", 3.5, 2.5));
      m_chooser.addOption("cone middle right charge right", getPathCommand("cone middle right charge right", 3.5, 2.5));
      m_chooser.addOption("cone right right charge right", getPathCommand("cone right right charge right", 3.5, 2.5));
      m_chooser.addOption("cone left left charge left", getPathCommand("cone left left charge left", 3.5, 2.5));
      m_chooser.addOption("cube middle no drive", getPathCommand("cube middle no drive", 3.5, 2.5));

      // Configure the button bindings
      configureButtonBindings();
  
      // Configure default commands
      m_driveTrain.setDefaultCommand(new SwerveDrive(m_driveTrain));
      m_arm.setDefaultCommand(new ArmControl(m_arm, m_claw, 0.14 * 12, 0.17 * 12));
      m_intake.setDefaultCommand(new IntakeControl(m_intake, 0.45 * 12, true));
      m_LED.setDefaultCommand(new SetSpike(m_LED));
    }

      
  
    /**
     * Use this method to define your button->command mappings.  Buttons can be created by
     * instantiating a {@link GenericHID} or one of its subclasses ({@link
     * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a
     * {@link edu.wpi.first.wpilibj2.command.button.JoystickButton}.
     */
    
    private void configureButtonBindings() {
      //CO DRIVER

      //left panel
      Trigger leftTLButton = new JoystickButton(m_leftPanel, 10);
      Trigger leftMLButton = new JoystickButton(m_leftPanel, 11);
      Trigger leftBLButton = new JoystickButton(m_leftPanel, 12);
      Trigger leftTRButton = new JoystickButton(m_leftPanel, 7);
      Trigger leftMRButton = new JoystickButton(m_leftPanel, 8);
      Trigger leftBRButton = new JoystickButton(m_leftPanel, 9);

      //right (and top) panel
      Trigger headerLButton = new JoystickButton(m_rightPanel, 7);
      Trigger headerMButton = new JoystickButton(m_rightPanel, 8);
      Trigger headerRButton = new JoystickButton(m_rightPanel, 6);
      Trigger rightTLButton = new JoystickButton(m_rightPanel, 10);
      Trigger rightBLButton = new JoystickButton(m_rightPanel, 12);
      Trigger rightTRButton = new JoystickButton(m_rightPanel, 9);
      Trigger rightBRButton = new JoystickButton(m_rightPanel, 11);

      //MAIN DRIVER
      Trigger xButton = new JoystickButton(m_mainStick, Button.kX.value);
      Trigger aTrigger = new JoystickButton(m_mainStick, Button.kA.value);
      Trigger backButton = new JoystickButton(m_mainStick, Button.kBack.value);
      Trigger rBButton = new JoystickButton(m_mainStick, Button.kRightBumper.value);
      
      //CUBES
      leftTLButton.onTrue(highCube());
      leftMLButton.onTrue(midCube());
      leftBLButton.onTrue(hybrid());

      //CONES
      leftTRButton.onTrue(highCone());
      leftMRButton.onTrue(midCone());
      leftBRButton.onTrue(hybrid());

      //
      //xCoButton.onTrue(new ToggleClawAngle(m_claw));

      aTrigger.toggleOnTrue(new TestWheelSpeed(m_driveTrain));

      //backCoButton.onTrue(new TogglePipeline());
      headerLButton.toggleOnTrue(new AutoBalance(m_driveTrain));
      headerRButton.onTrue(new ArmPID(m_arm, m_claw, 82, -150, false));
      headerMButton.onTrue(pickup());

      rightTRButton.onTrue(new ClawControl(m_claw));
      rightBRButton.onTrue(stow());

      //rightTLButton.whileTrue(new IntakeControl(m_intake, -0.4 * 12, true));
      //rightBLButton.whileTrue(new IntakeControl(m_intake, 0.4 * 12, true));

      Trigger startButton = new JoystickButton(m_mainStick, Button.kStart.value);
      startButton.onTrue(new Reset(m_driveTrain));
      rBButton.onTrue(new Reset(m_driveTrain));

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
      return new ArmPID(m_arm, m_claw, -4, 2, true).withTimeout(2);
    }

    public Command pickup() {
      return new ArmPID(m_arm, m_claw, 17, -67, false).withTimeout(2);
    }

    public Command highCone() {
      return new SequentialCommandGroup(
        new ArmPID(m_arm, m_claw, 87, 0, false).withTimeout(1),
        new ArmPID(m_arm, m_claw, 87, -121, false).withTimeout(1)
      );
    }

    public Command midCone() {
      return new SequentialCommandGroup(
        new ArmPID(m_arm, m_claw, 69, 0, false).withTimeout(1),
        new ArmPID(m_arm, m_claw, 69, -90, false).withTimeout(1)
      );
    }

    public Command hybrid() {
      return new ArmPID(m_arm, m_claw, 28, -60, false);
    }

    public Command highCube() {
      return new SequentialCommandGroup(
        new ArmPID(m_arm, m_claw, 83, 0, false).withTimeout(1),
        new ArmPID(m_arm, m_claw, 83, -125, false).withTimeout(1)
      );
    }

    public Command midCube() {
      return new SequentialCommandGroup(
        new ArmPID(m_arm, m_claw, 75, 0, false).withTimeout(1),
        new ArmPID(m_arm, m_claw, 75, -120, false).withTimeout(1)
      );
    }

    public Command toggleClaw() {
      return new ClawControl(m_claw);
    }

    public Command autoBalance() {
      return new SequentialCommandGroup(
        new MoveUntilBalance(m_driveTrain, -1), //NEEDS TO BE TUNED
        new AutoBalance(m_driveTrain)
      );
    }

    public Command autoBalanceReverse() {
      return new SequentialCommandGroup(
        new MoveUntilBalance(m_driveTrain, 1), //NEEDS TO BE TUNED
        new AutoBalance(m_driveTrain)
      );
    }

    public Command end() {
      return new ParallelCommandGroup(
        new SwerveDrive(m_driveTrain).withTimeout(0.1),
        new ArmControl(m_arm, m_claw, 0, 0).withTimeout(0.1),
        new IntakeControl(m_intake, 0, false).withTimeout(0.1)
      );
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

    public Command fullAuto(List<PathPlannerTrajectory> pathGroup, PathConstraints maxSpd) {
      List<CommandBase> commands = new ArrayList<>();
      SmartDashboard.putNumber("size of trajectory", pathGroup.size());

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


    public Command getPathCommand(String name, double maxV, double maxA) {
      PathConstraints max = new PathConstraints(maxV, maxA);
      List<PathPlannerTrajectory> path = PathPlanner.loadPathGroup(name, max);
      return fullAuto(path, max);
    }
    /**
     * Use this to pass the autonomous command to the main {@link Robot} class.
     *
     * @return the command to run in autonomous
     */

    
    public Command getAutonomousCommand() {
        return m_chooser.getSelected();
    }

    
}