// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.Arm;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.SparkMaxRelativeEncoder.Type;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ArmConstants;
import frc.robot.Constants.PortConstants;

public class Arm extends SubsystemBase {
  /** Creates a new Arm. */
  private static Arm m_instance;
  private CANSparkMax m_armUp;
  private CANSparkMax m_armDown;
  private CANSparkMax m_claw;
  private PIDController m_armController;
  private RelativeEncoder m_upEncoder;
  private double armPosition; // arm position in degrees

  public static Arm getInstance(){
    if (m_instance == null){
      m_instance = new Arm();
    }
    return m_instance;
  }
  private Arm() {
    m_armUp = new CANSparkMax(PortConstants.ARM_UP, MotorType.kBrushless);
    m_armDown = new CANSparkMax(PortConstants.ARM_DOWN, MotorType.kBrushless);
    m_claw = new CANSparkMax(PortConstants.CLAW, MotorType.kBrushless);
    m_armController = new PIDController(0, 0, 0);
    m_upEncoder = m_armUp.getEncoder(Type.kQuadrature, 360*ArmConstants.ARM_GEAR_RATIO);
  }

  public void armControl(double input) {
    m_armUp.setVoltage(input);
    m_armDown.setVoltage(-input);
  }

  public void armPID(double setpoint) {
    double kV = 0;
    double kA = 0;
    double kFF = 0;
    double PIDoutput = m_armController.calculate(armPosition, setpoint);
    double output = PIDoutput + kV*setpoint;
  }

  public void clawControl(double input) {
    m_claw.setVoltage(input);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    //update armPosition
    armPosition = m_upEncoder.getPosition();
    SmartDashboard.putNumber("up encoder position", m_upEncoder.getPosition());
  }
}
