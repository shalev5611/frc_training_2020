/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj.GenericHID.Hand;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj.XboxController;
import frc.robot.Constants;
import frc.robot.subsystems.Chassis;


public class Drive_By_Xbox extends CommandBase {
  /**
   * Creates a new Drive_By_Xbox.
   */
  XboxController controller;
  Chassis chassis;

  public Drive_By_Xbox(XboxController controller, Chassis chassis) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.chassis = chassis;
    this.controller = controller;
    addRequirements(chassis);
  }
  public double Thresh(double power){
    if(power > Constants.min_pwr&&power<Constants.max_pwr){
      return Constants.p1*(power-Constants.min_pwr)/(Constants.max_pwr-Constants.min_pwr);
    }
    else if(power<Constants.min_pwr){
      return 0;
    }
    else if(power < -Constants.min_pwr&&power>-Constants.max_pwr){
      return -Constants.p1*(power-Constants.min_pwr)/(Constants.max_pwr-Constants.min_pwr);
    }
    else if(power>-Constants.min_pwr){
      return 0;
    }
    else{
      return -Constants.p1+((power-Constants.max_pwr)/(1-Constants.max_pwr))*(1-Constants.p1);
    }
  }
    /*
    if(power < -Constants.min_pwr && power>-Constants.max_pwr){
      return -Constants.p1*(power-Constants.min_pwr)/(Constants.max_pwr-Constants.min_pwr);
    }
    else if(power>-Constants.min_pwr){
      return 0;
    }
    else{
      return -Constants.p1+((power-Constants.max_pwr)/(1-Constants.max_pwr))*(1-Constants.p1);
    }
  }
  */
  @Override
  public void initialize() {

  
  // Called every time the scheduler runs while the command is scheduled.

  
    double pos_Y_left= controller.getY(Hand.kLeft);
    double pos_Y_right = controller.getY(Hand.kRight);
    chassis.Set_power(Thresh(pos_Y_left), Thresh(pos_Y_right)); 
    System.out.println(Thresh(pos_Y_left));
    System.out.println(Thresh(pos_Y_right));    
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
