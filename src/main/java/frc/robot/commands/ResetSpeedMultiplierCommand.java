package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.RobotContainer;
import frc.robot.subsystems.DriveAugments;

public class ResetSpeedMultiplierCommand extends Command {

    DriveAugments Augments;

    public ResetSpeedMultiplierCommand(DriveAugments d){
        Augments = d;
        addRequirements(d);
   }

    @Override
    public void execute(){
        RobotContainer.SpeedMultiplier = 1;
    }
    
    @Override
    public void initialize(){
    }

    @Override
    public boolean isFinished() {
       return false;
    }

}
