package frc.robot.subsystems.DriveTrain;

import com.ctre.phoenix.motorcontrol.TalonFXControlMode;
import com.ctre.phoenix.motorcontrol.TalonFXSensorCollection;
import com.ctre.phoenix.motorcontrol.TalonFXSimCollection;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenix.sensors.CANCoder;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants.SwerveConstants;

public class SwerveDriveWheel
{
    public PIDController directionController;
    public TalonFX rotationMotor;
    public TalonFX speedMotor;
    public CANCoder rotationSensor;
    public double lastSetpointSpeed;
    double krP, krI, krD, ksP, ksI, ksD, rAccumulator, maxA, speedAccumulator, maxV;
    double currentOutputSpeed;
    double brakeConstant;
    public boolean unflipped;

    public SwerveDriveWheel(double rP, double rI, double rD, double sP, double sI, double sD, TalonFX rotationMotor, CANCoder rotationSensor, TalonFX speedMotor)
    {
        this.rotationSensor = rotationSensor;
        this.rotationMotor = rotationMotor;
        this.speedMotor = speedMotor;
        this.lastSetpointSpeed = 0;
        this.currentOutputSpeed = 0;
        this.brakeConstant = 1;
        
        //this.directionController = new PIDController(P, I, D);
        krP = rP;
        krI = rI;
        krD = rD;
        ksP = sP;
        ksI = sI;
        ksD = sD;
        unflipped = true;

        maxA = 100;
        rAccumulator = 0;
        maxV = SwerveConstants.MAX_WHEEL_ACCELERATION;
        speedAccumulator = 0;
    }

    public SwerveModulePosition getModule() {
        return new SwerveModulePosition(getPosition(), Rotation2d.fromDegrees(rotationSensor.getAbsolutePosition()));
    }

    public double getVelocity() {
        double v = speedMotor.getSelectedSensorVelocity()*10;
        v *= SwerveConstants.SWERVE_POSITION_RATIO;
        return v; //meters per second
    }
    
    public double getRelativeVelocity() {
        if (!unflipped) {
            return -getVelocity();
        }
        return getVelocity();
    }

    public double getPosition() {
        double pos = speedMotor.getSelectedSensorPosition();
        pos *= SwerveConstants.SWERVE_POSITION_RATIO;
        return pos; //meters
    
    }

    public double getRotation() {
        return rotationSensor.getAbsolutePosition();
    }

    public double getRelativeRotation() {
        if (unflipped) {
            return (getRotation()+180) % 360;
        }
        return getRotation();

    }

    public void resetPosition() {
        speedMotor.setSelectedSensorPosition(0);
    }

    public void setOutputSpeed(double spd) {
        rotationMotor.set(TalonFXControlMode.PercentOutput, 0);
        speedMotor.set(TalonFXControlMode.PercentOutput, spd * this.brakeConstant);
        this.currentOutputSpeed = spd;
    }

    public void setVelocity(double setpointAngle, double setpointSpeed) {
        
        double currentAngle = rotationSensor.getAbsolutePosition();
        double angle = (((setpointAngle - currentAngle) % 360) + 360 ) % 360;
        double error = getAngleError(currentAngle, setpointAngle);
        double speedError = setpointSpeed - getRelativeVelocity();
        if (error < 20) {
            rAccumulator += error;
        }
        if (rAccumulator > maxA) {
            rAccumulator = maxA;
        } else if (rAccumulator < -maxA) {
            rAccumulator = -maxA;
        }
        speedAccumulator += speedError;
        if (speedAccumulator > maxV) {
            speedAccumulator = maxV;
        } else if (speedAccumulator < -maxV) {
            speedAccumulator = -maxV;
        }
        double anglePID = (krP * error) + (krI*rAccumulator);
        double speedPID = ((ksP * speedError) + (ksI*speedAccumulator))*(1-this.brakeConstant);

        double kV = 1/(SwerveConstants.MAX_WHEEL_SPEED);
        //double kA = 0;//1/(SwerveConstants.MAX_WHEEL_ACCELERATION);

        //double setpointAcceleration = (setpointSpeed-lastSetpointSpeed)/0.02;
        SmartDashboard.putNumber("speed PID", speedPID/SwerveConstants.MAX_WHEEL_SPEED);
        //SmartDashboard.putNumber("accleration", kA*setpointAcceleration);

        double angleOutput = anglePID;
        double speedOutput = (speedPID/SwerveConstants.MAX_WHEEL_SPEED + kV*setpointSpeed);// + kA*setpointAcceleration);

        if (angle % 180 > 90) {
            angleOutput = -angleOutput;
        }
        if (angle < 90 || angle > 270) {
            speedOutput = -speedOutput;
            unflipped = false;
        } else {
            unflipped = true;
        }
        this.currentOutputSpeed = speedOutput;
        rotationMotor.set(TalonFXControlMode.PercentOutput, angleOutput);
        speedMotor.set(TalonFXControlMode.PercentOutput, speedOutput*this.brakeConstant);
        lastSetpointSpeed = setpointSpeed;
        //SmartDashboard.putNumber("setpoint accleration", setpointAcceleration);

    }

    public void brake(double power) {
        this.brakeConstant = -0.105*Math.pow(9, power) + 1.105;
    }

    public double getAngleError(double currentAngle, double setpoint) {
        double setpointAngle = (setpoint % 360) - (currentAngle % 360); //the %360 for some of these really should not do anything, as setpoint and current_angle are always positive.
        double setpointFAngle = ((setpoint+180)%360) - (currentAngle % 360);

        if (Math.abs(setpointAngle) > 180) {
            setpointAngle = -(Math.signum(setpointAngle) * 360.0) + setpointAngle;
        }
        if (Math.abs(setpointFAngle) > 180) {
            setpointFAngle = -(Math.signum(setpointFAngle) * 360.0) + setpointFAngle;
        }
        double error = Math.abs(Math.min(setpointAngle, setpointFAngle));
        if (error > 90) {
            error = 180 - error;
        }
        return error;
    }
    
}