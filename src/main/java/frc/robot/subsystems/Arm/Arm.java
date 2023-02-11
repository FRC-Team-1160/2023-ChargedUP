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
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ArmConstants;
import frc.robot.Constants.PortConstants;

public class Arm extends SubsystemBase {
  /** Creates a new Arm. */
  private static Arm m_instance;
  private CANSparkMax m_armUp;
  private CANSparkMax m_armDown;
  
  private PIDController m_armController;
  private RelativeEncoder m_upEncoder;
  public ADXRS450_Gyro m_gyro;

  public static Arm getInstance(){
    if (m_instance == null){
      m_instance = new Arm();
    }
    return m_instance;
  }
  private Arm() {
    m_armUp = new CANSparkMax(PortConstants.ARM_UP, MotorType.kBrushless);
    m_armDown = new CANSparkMax(PortConstants.ARM_DOWN, MotorType.kBrushless);
    m_armController = new PIDController(0.00001, 0, 0);
    //m_upEncoder = m_armUp.getEncoder(Type.kHallSensor, 360*ArmConstants.ARM_GEAR_RATIO);
    m_gyro = new ADXRS450_Gyro();
    
  }

  public void armControl(double input) {
    m_armUp.setVoltage(input);
    m_armDown.setVoltage(-input);
  }

  public void armPID(double setpoint) {
    double kV = 0;
    double kA = 0;
    double kFF = 0;
    double PIDoutput = m_armController.calculate(m_gyro.getAngle(), setpoint);
    double output = PIDoutput + kV*setpoint;
    m_armUp.setVoltage(output);
    m_armDown.setVoltage(-output);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    //update armPosition
    SmartDashboard.putNumber("arm gyro", m_gyro.getAngle());
  }
}
