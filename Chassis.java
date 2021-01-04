/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.DemandType;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import java.lang.Math;

import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.Sendable;
import edu.wpi.first.wpilibj.smartdashboard.SendableBuilder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.RobotContainer;
import frc.robot.commands.Drive_By_Xbox;
import com.ctre.phoenix.sensors.PigeonIMU;

public class Chassis extends SubsystemBase implements Sendable{
  private TalonSRX motor_right_1 = new TalonSRX(Constants.right_port_1);
  private TalonSRX motor_right_2 = new TalonSRX(Constants.right_port_2);
  private TalonSRX motor_left_1 = new TalonSRX(Constants.left_port_1);
  private TalonSRX motor_left_2 = new TalonSRX(Constants.left_port_2);
  private PigeonIMU gyro = new PigeonIMU(Constants.gyro_port);
   
  public Chassis() {
    super();
    motor_right_2.follow(motor_right_1);
    motor_left_2.follow(motor_left_1);
    // setDefaultCommand(new Drive_By_Xbox(RobotContainer.controller, this));
    motor_left_1.getSelectedSensorPosition(0);
    motor_right_1.getSelectedSensorPosition(0);
    motor_left_1.config_kP(0,Constants.kp);
    motor_left_1.config_kI(0,Constants.ki);
    motor_left_1.config_kD(0,Constants.kd);
    motor_right_1.config_kP(1,Constants.kp);
    motor_right_1.config_kI(1,Constants.ki);
    motor_left_1.config_kD(1,Constants.kd);

    SmartDashboard.putData(this);
  }
  @Override
  public void initSendable(SendableBuilder builder) {
    // TODO Auto-generated method stub
    super.initSendable(builder);
    builder.addDoubleProperty("Left Velocity", this::getVelocityLeft, null);
    builder.addDoubleProperty("Right Velocity", this::getVelocityRight, null);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  //public void Set_power(double power_left, double power_right){
  //  motor_right_1.set(ControlMode.PercentageOutput, power_right);
  //  motor_right_2.set(ControlMode.PercentageOutput, power_right);
  //  motor_left_1.set(ControlMode.PercentageOutput, power_left);
  //  motor_left_2.set(ControlMode.PercentageOutput, power_left);
  //}
  public double getAngle(){ // returns the raw angle of the robot
    return gyro.getFusedHeading();
  }

  public void Set_angle(double speed,double lastError,double gyroStartValue, double sumError){
    double angle = gyro.getFusedHeading();
    double error = angle - gyroStartValue;
    sumError += error;
    double p = error * Constants.kp;
    double i = sumError *Constants.ki;
    double d = (lastError - error) * Constants.kd;
    lastError = error;
    double corr = (p + i + d);
    motor_left_1.set(ControlMode.Velocity, ((speed * (1-corr))) / 10, DemandType.ArbitraryFeedForward,
    (Constants.kv + Constants.ks * speed + Constants.ka * 0) / 12);
    motor_left_1.set(ControlMode.Velocity, ((speed * (1+corr))) / 10, DemandType.ArbitraryFeedForward,
    (Constants.kv + Constants.ks * speed + Constants.ka * 0) / 12);
  }
  public void setConstantSpeed(double speed, String motor, double lastError, double sumError){
    double current = this.getVelocityRight();
    double error = speed - current;
    sumError += error;
    double p = error * Constants.kp;
    double i = sumError *Constants.ki;
    double d = (lastError - error) * Constants.kd;
    lastError = error;
    double corr = (p + i + d);
    
    if(motor == "right"){
      motor_right_1.set(ControlMode.Velocity, (Constants.pulses_in_meter * (speed * corr))/10, DemandType.ArbitraryFeedForward,
      (Constants.kv + Constants.ks * speed + Constants.ka * 0) / 12);// in volts so divide by 12  
    }
    if(motor == "left"){
      motor_left_1.set(ControlMode.Velocity, (Constants.pulses_in_meter * (speed * corr)) / 10, DemandType.ArbitraryFeedForward,
    (Constants.kv + Constants.ks * speed + Constants.ka * 0) / 12);
    }
  }

  public void setForwardSpeed(double speed, double sp, double lastError,double gyroStartValue, double sumError){
    double current = motor_right_1.getSelectedSensorPosition();
    //double sp = Constants.pulses_in_meter;
    double error = sp - current;
    sumError += error;
    double p = error * Constants.kp;
    double i = sumError *Constants.ki;
    double d = (lastError - error) * Constants.kd;
    lastError = error;
    double corr = (p + i + d);
    Set_angle(speed*corr,0,gyroStartValue,0);
  } 
  public double getVelocityRight(){
    double velocityRight = motor_right_1.getSelectedSensorVelocity();

    return (Constants.pulses_in_meter * velocityRight)/10;
  }
  public double getVelocityLeft(){
    
    double velocityLeft  = motor_left_1.getSelectedSensorVelocity();

    return (Constants.pulses_in_meter * velocityLeft)/10;
  }
  //public double setDirector(double speed, double lastError, double gyroStartValue){
  //  double angle = motor_left_1.Gyro.GetAngle();
  //  double error = angle - gyroStartValue;
  //  sumError += error;
  //  double p = error * Constants.kp;
  //  double i = sumError *Constants.ki;
  //  double d = (lastError - error) * Constants.kd;
  //  lastError = error;
  //  double corr = (p + i + d);
  //  motor_right_1.set(ControlMode.Velocity, (Constants.pulses_in_meter * speed)/10, DemandType.ArbitraryFeedForward,
  //  (Constants.kv + Constants.ks * speed + Constants.ka * 0) / 12);
  //  double leftSpeed = speed * (1 - corr);
  //  double rightSpeed = speed * (1 + corr);
  //}
  public double getEncoder(int id ){
    double result;
    if(id==1){
      result = motor_right_1.getSelectedSensorPosition();
    }
    else if(id==2){ 
      result = motor_left_1.getSelectedSensorPosition();
    }
    else{
      result = 666;
    }
    return result;
  } 
}
