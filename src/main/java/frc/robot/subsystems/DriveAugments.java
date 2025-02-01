package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.Subsystem;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.commands.ResetSpeedMultiplierCommand;
import frc.robot.commands.SlowModeCommand;
import frc.robot.commands.UltraSlowModeCommand;

/**
 * Class that extends the Phoenix 6 SwerveDrivetrain class and implements
 * Subsystem so it can easily be used in command-based projects.
 */
public class DriveAugments implements Subsystem {
    CommandXboxController Driver;
    SlowModeCommand slowMode;
    ResetSpeedMultiplierCommand resetSpeed;
    UltraSlowModeCommand ultraSlowMode;

    public DriveAugments(CommandXboxController bruh){
        Driver = bruh;
        Trigger leftTrigger = Driver.leftTrigger();
        Trigger rightTrigger = Driver.rightTrigger();
        slowMode = new SlowModeCommand(this);
        ultraSlowMode = new UltraSlowModeCommand(this);
        leftTrigger.whileTrue(slowMode);
        rightTrigger.whileTrue(ultraSlowMode);
        resetSpeed = new ResetSpeedMultiplierCommand(this);

        this.setDefaultCommand(resetSpeed);
    }   

    @Override
    public void periodic(){
    
    }

    // public final Command slowMoCommand(){

    //     return Commands.run(() -> 
    //     {   
    //       RobotContainer.SpeedMultiplier = 0.5;
    //     }
    //     );
    // }

    // public final Command ultraSlowMoCommand(){

    //     return Commands.run(() -> 
    //     {   
    //         RobotContainer.SpeedMultiplier = 0.25;
    //     }
    //     );
    // }
}



