package frc.robot.commands;

import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.subsystems.Elevator;

public class ElevatorToStageCommand extends Command {
   Elevator Elevator;

   TalonFX ElevatorLeft,ElevatorRight;
   CommandXboxController Manipulator;
   Angle position;
   final MotionMagicVoltage positionControl;

    public ElevatorToStageCommand(Elevator e, Angle a, int level){
    Elevator = e;
    ElevatorLeft = Elevator.ElevatorLeft;
    ElevatorRight = Elevator.ElevatorRight;
    Manipulator = Elevator.Manipulator;
    position = a;
    positionControl = new MotionMagicVoltage(position).withSlot(0);
    Elevator.stageLevel = level;
    addRequirements(Elevator);

    
      
   }

    @Override
    public void initialize(){

    }

    @Override
    public void execute(){

      ElevatorLeft.setControl(positionControl);

      //ElevatorLeft.setControl(pos.withPosition(position));
      
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