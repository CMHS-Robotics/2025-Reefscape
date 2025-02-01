package frc.robot.commands;

import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.RobotContainer;
import frc.robot.subsystems.DriveAugments;

public class SlowModeCommand extends Command {

    DriveAugments Augments;
    public SlowModeCommand(DriveAugments d){
        Augments = d;
        addRequirements(Augments);
      
   }

    @Override
    public void execute(){
        RobotContainer.SpeedMultiplier = 0.5;
        Augments.Driver.setRumble(RumbleType.kBothRumble,0.3);
    }
    
    @Override
    public void initialize(){
    }

    public void end(){
        Augments.Driver.setRumble(RumbleType.kBothRumble,0);
    }

    @Override
    public boolean isFinished() {
       return false;
    }

}
