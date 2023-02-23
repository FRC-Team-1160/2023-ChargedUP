// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.Arm;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.SparkMaxRelativeEncoder.Type;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj.PowerDistribution.ModuleType;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ArmConstants;
import frc.robot.Constants.PortConstants;

public class Claw extends SubsystemBase {
  /** Creates a new Claw. */
  private static Claw m_instance;
  private DoubleSolenoid m_solenoid;
  private CANSparkMax m_wrist;
  private RelativeEncoder m_encoder;
  public DigitalInput m_clawSwitch;
  private double kP, kI;
  private PIDController m_wristController;
  public double wristAngle, angle, encoderOffset;
  private Arm m_arm;
  public boolean keepClawAngle;
  //private Compressor m_compressor;

  public static Claw getInstance(){
    if (m_instance == null){
      m_instance = new Claw();
    }
    return m_instance;
  }
  private Claw() {
    m_solenoid = new DoubleSolenoid(PneumaticsModuleType.CTREPCM, 0, 1);
    m_solenoid.set(Value.kReverse);
    m_wrist = new CANSparkMax(PortConstants.WRIST, MotorType.kBrushless);
    m_encoder = m_wrist.getEncoder(Type.kHallSensor, 42);
    kP = 0.3;
    kI = 0.001;
    
    m_wristController = new PIDController(kP, kI, 0);
    m_clawSwitch = new DigitalInput(PortConstants.CLAW_SWITCH);
    wristAngle = 0;
    angle = 0;
    encoderOffset = 0;
    m_arm = Arm.getInstance();
    keepClawAngle = false;
  }

  public void toggleClawAngle() {
    keepClawAngle = !keepClawAngle;
  }

  public void toggleClaw() {
    m_solenoid.toggle();
    
  }

  public double getEncoderPosition() {
    return m_encoder.getPosition()/ArmConstants.WRIST_POSITION_CONVERSION;
  }


  public void wristControl(double input) {
    m_wrist.setVoltage(input);
    /*if (Math.abs(input) > 0.1) {
      m_wrist.setVoltage(input);
    } else {
      m_wrist.stopMotor();
    }*/
  }

  public void wristPID(double setpoint) {
    double kV = -0.001;
    double kA = 0;
    double kFF = 0;
    double currentAngle = wristAngle;
    if (keepClawAngle) {
      currentAngle = angle;
    }
    double PIDoutput = m_wristController.calculate(currentAngle, setpoint);
    double output = PIDoutput + kV*setpoint;
    double max = 3;
    if (setpoint > currentAngle) {
      max = 4.5;
    }
    if (output > max) {
      output = max;
    }
    if (output < -max) {
      output = -max;
    }
    m_wrist.setVoltage(output);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    if (m_solenoid.get() == Value.kForward) {
      SmartDashboard.putString("Claw", "Forward");
    } else if (m_solenoid.get() == Value.kReverse) {
      SmartDashboard.putString("Claw", "Reverse");
    } else if (m_solenoid.get() == Value.kOff) {
      SmartDashboard.putString("Claw", "Off");
    } else {
      SmartDashboard.putString("Claw", "No data");
    }

    if (!m_clawSwitch.get()) {
      encoderOffset = getEncoderPosition();
    }
    wristAngle = getEncoderPosition() - encoderOffset;

    angle = wristAngle + m_arm.angle;

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
