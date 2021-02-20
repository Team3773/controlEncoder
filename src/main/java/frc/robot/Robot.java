// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.SpeedController;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import frc.robot.angleOneCommand;

/**
 * This sample program shows how to control a motor using a joystick. In the
 * operator control part of the program, the joystick is read and the value is
 * written to the motor.
 *
 * <p>
 * Joystick analog values range from -1 to 1 and speed controller inputs also
 * range from -1 to 1 making it easy to work together.
 *
 * <p>
 * In addition, the encoder value of an encoder connected to ports 0 and 1 is
 * consistently sent to the Dashboard.
 */
public class Robot extends TimedRobot {
  private static final int kMotorPort = 4;
  private static final int kJoystickPort = 3;
  private static final int kEncoderPortA = 1;
  private static final int kEncoderPortB = 2;
  private JoystickButton button2;
  public SpeedController m_motor;
  private Joystick m_joystick;
  public Encoder m_encoder;


  @Override
  public void robotInit() {
    m_motor = new WPI_TalonSRX(kMotorPort);
    m_joystick = new Joystick(kJoystickPort);
    m_encoder = new Encoder(kEncoderPortA, kEncoderPortB);
    button2 = new JoystickButton(m_joystick, 2);

    // Use SetDistancePerPulse to set the multiplier for GetDistance
    // This is set up assuming a 2.75 inch wheel with a 360 CPR encoder.
    m_encoder.setDistancePerPulse((Math.PI * 2.75) / 360.0);
    m_encoder.reset();

  }

  /*
   * The RobotPeriodic function is called every control packet no matter the robot
   * mode.
   */
  @Override
  public void robotPeriodic() {
    SmartDashboard.putNumber("Encoder", m_encoder.getDistance());

    System.out.print(m_encoder.getDistance());

    button2.whenPressed(new angleOneCommand());    
  }

  @Override
  public void teleopPeriodic() {
   // m_motor.set(m_joystick.getY());
  }
}
