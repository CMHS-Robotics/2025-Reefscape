package frc.robot.commands;

import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;

public class MoveElevator extends Command {
    TalonFX ElevatorLeft;
    TalonFX ElevatorRight;  
    XboxController Manipulator;

    public MoveElevator(XboxController M, TalonFX L, TalonFX R){
        Manipulator = M;
        ElevatorLeft = L;
        ElevatorRight = R;
    }

    @Override
    public void execute(){

        if(Manipulator.getPOV() > -1){
            if(Manipulator.getPOV() == 0){
                
                ElevatorLeft.set(1);
                ElevatorRight.set(1);
            }else if(Manipulator.getPOV() == 180){
    
                ElevatorLeft.set(-1);
                ElevatorRight.set(1);
            }
            }else{
                ElevatorLeft.set(0);
                ElevatorRight.set(1);
    
            }
        }

    }
