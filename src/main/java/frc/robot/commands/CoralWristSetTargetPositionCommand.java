package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.CoralWristV2;
public class CoralWristSetTargetPositionCommand extends Command {
   CoralWristV2 wrist;
   
   int level;

   public CoralWristSetTargetPositionCommand(CoralWristV2 w,int a){
      wrist = w;
      level = a;
   }

   @Override
   public void initialize(){
      wrist.setTargetLevel(level);
      
   }


    @Override
    public void execute(){
    }
    
    public boolean end(){
      return wrist.hasReachedTarget();
    }

    @Override
    public boolean isFinished() {
       return end();
    }


    }
