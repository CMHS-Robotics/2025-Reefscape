// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.subsystems.DriveAugments;
import frc.robot.subsystems.Elevator;


public class Robot extends TimedRobot {
  private Command m_autonomousCommand;

  private final RobotContainer m_robotContainer;
  
  XboxController Driver = new XboxController(0);
  XboxController Manipulator = new XboxController(1);
  Elevator Elevator = new Elevator(Manipulator);
  DriveAugments Augment = new DriveAugments(Driver);


  public Robot() {
    m_robotContainer = new RobotContainer();
  }

  @Override
  public void robotPeriodic() {
    CommandScheduler.getInstance().run(); 
  }

  @Override
  public void disabledInit() {}

  @Override
  public void disabledPeriodic() {}

  @Override
  public void disabledExit() {}

  @Override
  public void autonomousInit() {
    m_autonomousCommand = m_robotContainer.getAutonomousCommand();
    if (m_autonomousCommand != null) {
      m_autonomousCommand.schedule();
    }
    }

  @Override
  public void autonomousPeriodic() {}

  @Override
  public void autonomousExit() {}

  @Override
  public void teleopInit() {
    if (m_autonomousCommand != null) {
      m_autonomousCommand.cancel();
    }
  }

  @Override
  public void teleopPeriodic() {

  Augment.checkInput();

  //Elevator.checkInput();


    // m_robotContainer.SpeedMultiplier -= 0.1 * Xbox.getLeftTriggerAxis();
    // m_robotContainer.SpeedMultiplier += 0.1 * Xbox.getRightTriggerAxis();

    // if(m_robotContainer.SpeedMultiplier >= 1){
    //   m_robotContainer.SpeedMultiplier = 1;
    // }
    // if(m_robotContainer.SpeedMultiplier <= 0.01){
    //   m_robotContainer.SpeedMultiplier = 0.01;
    // }
  }

  @Override
  public void teleopExit() {}

  @Override
  public void testInit() {
    CommandScheduler.getInstance().cancelAll();
  }

  @Override
  public void testPeriodic() {}

  @Override
  public void testExit() {}

  @Override
  public void simulationPeriodic() {}
}
