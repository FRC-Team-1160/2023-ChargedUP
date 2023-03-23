// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.Arm;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.SparkMaxRelativeEncoder.Type;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ArmConstants;
import frc.robot.Constants.PortConstants;
import frc.robot.Constants.SwerveConstants;
import frc.robot.Constants.OIConstants;

public class Claw extends SubsystemBase {
  /** Creates a new Claw. */
  private static Claw m_instance;
  private CANSparkMax m_wrist;
  private RelativeEncoder m_encoder;
  public DigitalInput m_clawSwitch;
  private double kP, kI, kD;
  private PIDController m_wristController;
  public double wristAngle, angle, encoderOffset;
  private Arm m_arm;
  public boolean keepClawAngle;
  public boolean lastFiveLimit;

  private final Timer timer = new Timer();
  private Joystick m_rightPanel = new Joystick(OIConstants.controlPanelRightPort);

  public static Claw getInstance(){
    if (m_instance == null){
      m_instance = new Claw();
    }
    return m_instance;
  }
  private Claw() {
    m_wrist = new CANSparkMax(PortConstants.WRIST, MotorType.kBrushless);
    m_encoder = m_wrist.getEncoder(Type.kHallSensor, 42);
    kP = 0.35;
    kI = 0.001;
    kD = 0.0;

    
    m_wristController = new PIDController(kP, kI, kD);
    m_clawSwitch = new DigitalInput(PortConstants.CLAW_SWITCH);
    wristAngle = 0;
    angle = 0;
    encoderOffset = 0;
    m_arm = Arm.getInstance();
    keepClawAngle = false;
    this.timer.reset();
    this.timer.start();
    lastFiveLimit = false;
  }

  public void toggleClawAngle() {
    keepClawAngle = !keepClawAngle;
  }

  public double getEncoderPosition() {
    return m_encoder.getPosition()/ArmConstants.WRIST_POSITION_CONVERSION;
  }


  public void wristControl(double input) {
    if (m_rightPanel.getRawButton(5)) {
      m_wrist.setVoltage(input);
    } else {
      if (wristAngle < ArmConstants.CLAW_LIMIT) {
        if (input < 0) {
          input = 0;
        }
      }
      if (!m_clawSwitch.get()) {
        if (input > 0) {
          input = 0;
        }
      }
      if (m_arm.angle < ArmConstants.ARM_BUMPER_SAFETY && wristAngle < ArmConstants.WRIST_BUMPER_SAFETY) {
        wristPID(ArmConstants.WRIST_BUMPER_SAFETY_SETPOINT);
      } else {
        m_wrist.setVoltage(input);
      }
    }
    
    /*if (Math.abs(input) > 0.1) {
      m_wrist.setVoltage(input);
    } else {
      m_wrist.stopMotor();
    }*/
    SmartDashboard.putNumber("wrist input", input);
  }

  public void wristPID(double setpoint) {
    double kV = -0.0015;
    double kA = 0;
    double kFF = 0;
    double currentAngle = wristAngle;
    if (keepClawAngle) {
      currentAngle = angle;
      double currentWristAngle = setpoint-m_arm.angle;
      if (currentWristAngle > ArmConstants.KEPT_CLAW_ANGLE_WRIST_SAFETY) {
        setpoint += ArmConstants.KEPT_CLAW_ANGLE_WRIST_SAFETY;
      }
    }
    if (m_arm.angle < ArmConstants.ARM_BUMPER_SAFETY && setpoint < ArmConstants.WRIST_BUMPER_SAFETY+m_arm.angle && keepClawAngle) {
      setpoint = ArmConstants.WRIST_BUMPER_SAFETY_SETPOINT+m_arm.angle;
    } else if (m_arm.angle < ArmConstants.ARM_BUMPER_SAFETY && setpoint < ArmConstants.WRIST_BUMPER_SAFETY && !keepClawAngle) {
      setpoint = ArmConstants.WRIST_BUMPER_SAFETY_SETPOINT;
    }
    double PIDoutput = m_wristController.calculate(currentAngle, setpoint);
    double output = PIDoutput + kV*setpoint;
    double max = 4;
    if (setpoint > currentAngle && setpoint-currentAngle > 50) {
      max = 7;
    }
    if (m_arm.angle < 10) {
      max = 3;
    }
    if (output > max) {
      output = max;
    }
    if (output < -max) {
      output = -max;
    }
    m_wrist.setVoltage(output);
    SmartDashboard.putNumber("wrist input", output);
  }

  public void resetTimer() {
    this.timer.reset();
    this.timer.start();
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    

    if (!m_clawSwitch.get()) {
      encoderOffset = getEncoderPosition();
      resetTimer();
    }
    wristAngle = getEncoderPosition() - encoderOffset;
    if (this.timer.get() < 3) {
      lastFiveLimit = true;
    } else {
      lastFiveLimit = false;
      m_arm.lastFiveLimit = false;
    }
    angle = wristAngle + m_arm.angle;
    SmartDashboard.putBoolean("lastFiveClawLimit", lastFiveLimit);
    SmartDashboard.putNumber("claw encoder", m_encoder.getPosition());
    SmartDashboard.putNumber("claw angle", angle);
    SmartDashboard.putNumber("wrist angle", wristAngle);
    /*
     * 0 is perpendicular to arm
     * -7.5 is parallel
     * -15 is perpendicular backwards
     * 5 is before it hits the motor
     */
    SmartDashboard.putBoolean("claw switch", !m_clawSwitch.get());
  }
}
