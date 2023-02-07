package frc.robot;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController.Button;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants.OIConstants;
import frc.robot.commands.ArmControl;
import frc.robot.commands.ClawControl;
import frc.robot.subsystems.Arm.Arm;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;

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
  
    // Controllers
    private Joystick m_mainStick = new Joystick(OIConstants.mainStickPort);



    /**
     * The container for the robot.  Contains subsystems, OI devices, and commands.
     */
    public RobotContainer() {

      // Configure the button bindings
      configureButtonBindings();
  
      // Configure default commands

    }

      
  
    /**
     * Use this method to define your button->command mappings.  Buttons can be created by
     * instantiating a {@link GenericHID} or one of its subclasses ({@link
     * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a
     * {@link edu.wpi.first.wpilibj2.command.button.JoystickButton}.
     */
    
    private void configureButtonBindings() {
      Trigger yButton = new JoystickButton(m_mainStick, 4);
      Trigger xButton = new JoystickButton(m_mainStick, 3);
      //yButton.onTrue(new ArmControl(m_arm, 0.5 * 12));
      xButton.onTrue(new ClawControl(m_arm, 0.1 * 12));

      

    }

    /**
     * Use this to pass the autonomous command to the main {@link Robot} class.
     *
     * @return the command to run in autonomous
     */

    
    public Command getAutonomousCommand() {
        return null;
    }
    
}