/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import com.analog.adis16470.frc.ADIS16470_IMU;


import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.GenericHID.Hand;
import frc.robot.commands.ExampleCommand;
import frc.robot.commands.Swervedrive;
import frc.robot.subsystems.DriveTrainSwerve;
import frc.robot.subsystems.ExampleSubsystem;
import edu.wpi.first.wpilibj2.command.Command;
//import edu.wpi.first.wpilibj2.command.CommandScheduler;

/**
 * This class is where the bulk of the robot should be declared.  Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls).  Instead, the structure of the robot
 * (including subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems and commands are defined here...
  private ExampleSubsystem m_exampleSubsystem = new ExampleSubsystem();
  private DriveTrainSwerve m_DriveTrainSwerve;

  private ExampleCommand m_autoCommand = new ExampleCommand(m_exampleSubsystem);
  private Swervedrive m_swervecommand = new Swervedrive(m_DriveTrainSwerve);

  private ADIS16470_IMU m_gyro;
  public static XboxController driveController = new XboxController(Constants.USB_driveController);

  
  /**
   * The container for the robot.  Contains subsystems, OI devices, and commands.
   */
  public RobotContainer() 
  {
    
    m_gyro = new ADIS16470_IMU();
    m_gyro.calibrate();
    m_DriveTrainSwerve = new DriveTrainSwerve(m_gyro);
    m_DriveTrainSwerve.setDefaultCommand(m_swervecommand);

    // Configure the button bindings
    configureButtonBindings();
  }
  public double getGyroAngle(){
    return m_gyro.getAngle();
  }
  public static double getDriveRightXAxis() {
    return driveController.getX(Hand.kRight);
  }

  public static double getDriveLeftYAxis() {
    return driveController.getY(Hand.kLeft);
  }

  public static double getDriveLeftXAxis() {
    return driveController.getX(Hand.kLeft);
  }

  /**
   * Use this method to define your button->command mappings.  Buttons can be created by
   * instantiating a {@link GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a
   * {@link edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private void configureButtonBindings() {
  }


  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // An ExampleCommand will run in autonomous
    return m_autoCommand;
  }
}
