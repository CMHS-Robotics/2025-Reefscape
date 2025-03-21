package frc.robot.commands;

import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.subsystems.Elevator;
import frc.robot.tools.PID;

public class ElevatorTargetPositionCommand extends Command {
   Elevator Elevator;

   TalonFX ElevatorLeft,ElevatorRight;
   CommandXboxController Manipulator;
   PID pid;
   double target;

   public ElevatorTargetPositionCommand(Elevator e,PID p){
      Elevator = e;
      ElevatorLeft = Elevator.ElevatorLeft;
      ElevatorRight = Elevator.ElevatorRight;
      Manipulator = Elevator.Manipulator;
      pid=p;
      target = 0.0;
      addRequirements(Elevator);
   }

   @Override
   public void initialize(){
      
   }


    @Override
    public void execute(){

      target = Elevator.getTargetPosition();

      pid.setSetPoint(target);
      
      pid.updatePID(ElevatorLeft.getPosition().getValueAsDouble());
            if(target <= 1 && ElevatorLeft.getPosition().getValueAsDouble() <= 1){
                ElevatorLeft.set(0);
                ElevatorRight.set(0);
            }else{
            ElevatorLeft.set(pid.getResult());
            ElevatorRight.set(ElevatorLeft.get());
            }
            if(Math.abs(Manipulator.getLeftTriggerAxis()) > 0.1){

                target += -Manipulator.getLeftTriggerAxis() * 0.05;

            }

            if(target > 17){
                target = 17;
            }
            if(target < 1){
                target = 1;
            }
      
     
    }
    
    @Override
    public boolean isFinished() {
       return false;
    }


    }
