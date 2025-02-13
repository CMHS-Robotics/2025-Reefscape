package frc.robot.commands;

import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Elevator;

public class ElevatorToStageCommand extends Command {
   Elevator Elevator;

   TalonFX ElevatorLeft,ElevatorRight;
   Angle position;
   MotionMagicVoltage positionControl;
   int level;
  
   public ElevatorToStageCommand(Elevator e, Angle a, int l){
      Elevator = e;
      ElevatorLeft = Elevator.ElevatorLeft;
      ElevatorRight = Elevator.ElevatorRight;
      position = a;
      level = l;
      positionControl = new MotionMagicVoltage(0);
      
      addRequirements(Elevator);  
   }

    @Override
    public void initialize(){

    }

    @Override
    public void execute(){

      SmartDashboard.putString("Command Running:","ToStage");
      Elevator.motorPosition = position;
      Elevator.targetPosition = position.magnitude();
      Elevator.stageLevel = level;
      // ElevatorLeft.setControl(positionControl.withPosition(position).withSlot(0));
      // ElevatorRight.setControl(positionControl.withPosition(position).withSlot(0));
      
    }
    
    public void end(){
    }

    @Override
    public boolean isFinished() {
       return targetReached();
    }

    public boolean targetReached(){
      boolean target = (Math.abs(ElevatorLeft.getPosition().getValueAsDouble() - position.baseUnitMagnitude()) < 5);
      
      
      return target;
    }
   }