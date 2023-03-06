// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.Arm;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.SparkMaxRelativeEncoder.Type;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.ADXRS450_Gyro;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ArmConstants;
import frc.robot.Constants.OIConstants;
import frc.robot.Constants.PortConstants;

public class Arm extends SubsystemBase {
  /** Creates a new Arm. */
  private static Arm m_instance;
  private CANSparkMax m_armUp;
  private final Timer timer = new Timer();
  private CANSparkMax m_armDown;
  public boolean lastFiveLimit;

  public DigitalInput m_armSwitch;
  
  private PIDController m_armController;
  private RelativeEncoder m_upEncoder;
  public double angle;
  public double encoderOffset;
  private double kP, kI;

  public static Arm getInstance(){
    if (m_instance == null){
      m_instance = new Arm();
    }
    return m_instance;
  }
  private Arm() {
    m_armUp = new CANSparkMax(PortConstants.ARM_UP, MotorType.kBrushless);
    m_armDown = new CANSparkMax(PortConstants.ARM_DOWN, MotorType.kBrushless);
    kP = 0.2;
    kI = 0.009;
    
    m_armController = new PIDController(kP, kI, 0);
    m_upEncoder = m_armUp.getEncoder(Type.kHallSensor, 42);
    m_armSwitch = new DigitalInput(PortConstants.ARM_SWITCH);
    angle = 0;
    encoderOffset = 0;
    this.timer.reset();
    this.timer.start();
    lastFiveLimit = false;
  }

  public double getEncoderPosition() {
    return m_upEncoder.getPosition()/ArmConstants.ARM_POSITION_CONVERSION;
  }

  public void armControl(double input) {
    if (!m_armSwitch.get()) {
      if (input < 0) {
        input = 0;
      }
    }
    if (angle > ArmConstants.ARM_LIMIT) {
      if (input > 0) {
        input = 0;
      }
    }
    if (input < 0) {
      input *= 0.8;
    }
    m_armUp.setVoltage(input);
    m_armDown.setVoltage(-input);
    SmartDashboard.putNumber("arm input", input);
  }

  public void armPID(double setpoint) {
    double kV = 0.001;
    double kA = 0;
    double kFF = 0;
    double PIDoutput = m_armController.calculate(angle, setpoint);
    double output = PIDoutput + kV*setpoint;
    if (setpoint > 110) {
      setpoint = 110;
    }
    
    double max = 2;
    if (angle < 12) {
      max = 1;
    }
    if (setpoint > angle) {
      max = 3.5;
    }
    if (output > max) {
      output = max;
    }
    if (output < -max) {
      output = -max;
    }
    m_armUp.setVoltage(output);
    m_armDown.setVoltage(-output);
  }

  public void resetTimer() {
    this.timer.reset();
    this.timer.start();
  }

  @Override
  public void periodic() {
    
    // This method will be called once per scheduler run

    //update armPosition
    if (!m_armSwitch.get()) {
      encoderOffset = getEncoderPosition();
      resetTimer();
    }
    angle = getEncoderPosition() - encoderOffset;
    if (this.timer.get() < 3) {
      lastFiveLimit = true;
    } else {
      lastFiveLimit = false;
      Claw.getInstance().lastFiveLimit = false;
    }
    SmartDashboard.putBoolean("lastFiveArmLimit", lastFiveLimit);
    SmartDashboard.putNumber("arm angle", angle);
    SmartDashboard.putNumber("arm encoder reading", getEncoderPosition());
    SmartDashboard.putBoolean("arm switch", !m_armSwitch.get());
  }
}
