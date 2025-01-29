package frc.robot.subsystems;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Subsystem;
import frc.robot.RobotContainer;

/**
 * Class that extends the Phoenix 6 SwerveDrivetrain class and implements
 * Subsystem so it can easily be used in command-based projects.
 */
public class DriveAugments implements Subsystem {
    XboxController Driver;

    public DriveAugments(XboxController bruh){

        Driver = bruh;
    }   




public void checkInput(){
    if(Driver.getLeftTriggerAxis()>0){
      RobotContainer.SpeedMultiplier = 0.25;
  }else if (Driver.getRightTriggerAxis()>0) {
      RobotContainer.SpeedMultiplier = 0.5;
  }else{
      RobotContainer.SpeedMultiplier = 1;
  }
    }
}



