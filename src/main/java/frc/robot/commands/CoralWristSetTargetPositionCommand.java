package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.CoralWristV2;
public class CoralWristSetTargetPositionCommand extends Command {
   CoralWristV2 wrist;
   
   int level;
   double targetPosition;
   boolean usePosition = false;

   public CoralWristSetTargetPositionCommand(CoralWristV2 w,int a){
      wrist = w;
      usePosition = false;
      level = a;
   }
   
   public CoralWristSetTargetPositionCommand(CoralWristV2 w,double a){
      wrist = w;
      usePosition = true;
      targetPosition = a;
   }

   @Override
   public void initialize(){
      if(usePosition){
         wrist.setTarget(targetPosition);
      }
         else{
            wrist.setTargetLevel(level);};

      
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
