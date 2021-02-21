// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.SpeedController;


public class angleTwoCommand extends CommandBase {
  public SpeedController m_motor;
  public Encoder m_encoder;
  public double angle = m_encoder.getDistance();

  public angleTwoCommand() {
  }
  
  @Override
  public void execute() {
    if (angle != 5.449541) {
      m_motor.set(0.1);
    }
    if(angle == 5.449541) {
      m_motor.set(0);
    } 
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
