package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.Subsystem;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.RobotContainer;

/**
 * Class that extends the Phoenix 6 SwerveDrivetrain class and implements
 * Subsystem so it can easily be used in command-based projects.
 */
public class DriveAugments implements Subsystem {
    CommandXboxController Driver;
    Trigger leftTrigger = Driver.leftTrigger();
    Trigger rightTrigger = Driver.rightTrigger();

    

    public DriveAugments(CommandXboxController bruh){
        Driver = bruh;
        leftTrigger.whileTrue(slowMoCommand());
        rightTrigger.whileTrue(ultraSlowMoCommand());
    }   



    public final Command slowMoCommand(){

        return Commands.run(() -> 
        {   
          RobotContainer.SpeedMultiplier = 0.5;
        }
        );
        }
    public final Command ultraSlowMoCommand(){

        return Commands.run(() -> 
        {   
            RobotContainer.SpeedMultiplier = 0.25;
        }
        );
        }
}



