package frc.robot.commands;

import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.subsystems.Elevator;

public class ElevatorHoldPositionCommand extends Command {
   Elevator Elevator;

   TalonFX ElevatorLeft,ElevatorRight;
   CommandXboxController Manipulator;
   MotionMagicVoltage positionControl;
   Angle position;
   Angle motPos;

    public ElevatorHoldPositionCommand(Elevator e){
    Elevator = e;
    ElevatorLeft = Elevator.ElevatorLeft;
    ElevatorRight = Elevator.ElevatorRight;
    Manipulator = Elevator.Manipulator;
    position = Elevator.stages[Elevator.stageLevel];
    motPos = Elevator.motorPosition.getValue();
    positionControl = new MotionMagicVoltage(motPos).withSlot(0);
    addRequirements(Elevator);
      
   }

    @Override
    public void execute(){

      ElevatorLeft.set(0);
      ElevatorRight.set(0);
      ElevatorLeft.setControl(positionControl);
      ElevatorRight.setControl(positionControl);

    }
    
    @Override
    public boolean isFinished() {
       return false;
    }

    }
