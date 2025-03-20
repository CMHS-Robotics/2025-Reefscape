package frc.robot.commands;

import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.CoralWristV2;
import frc.robot.tools.PID;

public class CoralWristTargetPositionCommand extends Command {
   CoralWristV2 wrist;

   TalonFX wristMotor;
   PID pid;
   double target;

   public CoralWristTargetPositionCommand(CoralWristV2 e,PID p){
      wrist = e;
      pid=p;
      target = 0.0;
      wristMotor = wrist.CoralWrist;
      addRequirements(wrist);
   }

   @Override
   public void initialize(){
      
   }


    @Override
    public void execute(){

        target = wrist.getTarget();

        pid.setSetPoint(target);

        wristMotor.set(pid.updatePID(wristMotor.getPosition().getValueAsDouble())); 
     
    }
    
    @Override
    public boolean isFinished() {
       return false;
    }


    }
