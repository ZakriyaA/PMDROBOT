// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.motorcontrol.MotorController;
import edu.wpi.first.wpilibj.motorcontrol.PWMSparkMax;
import edu.wpi.first.wpilibj.motorcontrol.Talon;

/**
 * This is a demo program showing the use of the DifferentialDrive class,
 * specifically it contains the code necessary to operate a robot with tank drive.
 */
public class Robot extends TimedRobot {
  // Declaration of robot drive system and joystick objects
  private DifferentialDrive m_myRobot;
  private Joystick m_leftStick;
  private Joystick m_rightStick;

  // Declaration of motor controller objects for the left and right drive motors
  private final MotorController m_leftMotor = new PWMSparkMax(0); // Left drive motor connected to PWM port 0
  private final MotorController m_rightMotor = new PWMSparkMax(1); // Right drive motor connected to PWM port 1

  // Declaration of motor controller objects for additional motors
  private final MotorController m_extraMotor1 = new Talon(8); // Extra motor 1 connected to PWM port 2
  private final MotorController m_extraMotor2 = new Talon(9); // Extra motor 2 connected to PWM port 3
  private final MotorController m_Lift1 = new Talon(5);
  private final MotorController m_Lift2 = new Talon(6);
    // Timer and flag for driving straight
  private final Timer m_timer = new Timer();
  private boolean m_driveStraight = false;
  @Override
  public void robotInit() {
    // Invert the right motor to ensure both motors move the robot forward
    m_rightMotor.setInverted(true);

    // Initialize the differential drive with the left and right motors
    m_myRobot = new DifferentialDrive(m_leftMotor, m_rightMotor);

    // Initialize the joysticks on ports 0 and 1
    m_leftStick = new Joystick(0);
    m_rightStick = new Joystick(1);
  }

  @Override
  public void teleopPeriodic() {
        // Drive straight for 5 seconds when button 3 on the right joystick is pressed
    if (m_rightStick.getRawButtonPressed(3)) {
      m_driveStraight = true;
      m_timer.reset();
      m_timer.start();
    }

    if (m_driveStraight) {
      if (m_timer.get() < 2.0) {
        m_myRobot.tankDrive(0.5, 0.5); // Drive straight at 50% speed
      } else {
        m_driveStraight = false;
        m_myRobot.tankDrive(0.0, 0.0); // Stop the robot
        m_timer.stop();
      }
      return; // Skip the rest of teleopPeriodic while driving straight
    // Control the robot using tank drive by getting the Y-axis values from the joysticks
    m_myRobot.tankDrive(-m_leftStick.getY(), -m_rightStick.getY());

    // Check if button 1 on the right joystick is pressed
    if (m_rightStick.getRawButton(1)) {

      // Set the extra motors to a certain speed when the button is pressed
      m_extraMotor1.set(1.0); // Set motor 1 to full speed
      m_extraMotor2.set(1.0); // Set motor 2 to full speed
    } else {
      // Stop the extra motors when the button is not pressed
      m_extraMotor1.stopMotor();
      m_extraMotor2.stopMotor();
    }

    // Check if button 1 on the left joystick is pressed
    if (m_leftStick.getRawButton(1)) {
      // Invert and set the extra motors to a certain speed when the button is pressed
      m_extraMotor1.setInverted(true);
      m_extraMotor2.setInverted(true);
      m_extraMotor1.set(1.0); // Set motor 1 to full speed
      m_extraMotor2.set(1.0); // Set motor 2 to full speed
    } else {
      // Stop the extra motors and reset inversion when the button is not pressed
      m_extraMotor1.setInverted(false);
      m_extraMotor2.setInverted(false);
      m_extraMotor1.stopMotor();
      m_extraMotor2.stopMotor();
    }

    // Check if button 2 on the left joystick is pressed
    if (m_leftStick.getRawButton(2)) {
      // Lower the lift motors when the button is pressed
      m_liftMotor1.set(-1.0);
      m_liftMotor2.set(-1.0);
    } else if (m_rightStick.getRawButton(2)) {
      // Raise the lift motors when the button is pressed
      m_liftMotor1.set(1.0);
      m_liftMotor2.set(1.0);
    } else {
      // Stop the lift motors when no button is pressed
      m_liftMotor1.stopMotor();
      m_liftMotor2.stopMotor();
    }
if (m_rightStick.getRawButton(2)) {
  // Set the extra motors to a certain speed when the button is pressed
  m_Lift1.set(1.0);
  m_Lift2.set(1.0);

} 

}
}}