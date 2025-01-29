package frc.robot.commands;

import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;

public class MoveElevator extends Command {
    
    private static final int elevatorMotorLeftId = 13;
    private static final int elevatorMotorRightId = 14;

    TalonFX ElevatorLeft = new TalonFX(elevatorMotorLeftId);   
    TalonFX ElevatorRight = new TalonFX(elevatorMotorRightId);   
    XboxController Manipulator;

    public MoveElevator(XboxController M){
        //Stuff to set up a motor to be able to spin - replace elevatorLeft with motor name

        ElevatorLeft.getConfigurator().apply(new TalonFXConfiguration());

        Manipulator = M;

        
        var currentConfiguration = new CurrentLimitsConfigs();
        currentConfiguration.StatorCurrentLimit = 80;
        currentConfiguration.StatorCurrentLimitEnable = true;
        ElevatorLeft.getConfigurator().refresh(currentConfiguration);
        ElevatorLeft.getConfigurator().apply(currentConfiguration);
        ElevatorLeft.setNeutralMode(NeutralModeValue.Brake);
    
        ElevatorRight.getConfigurator().refresh(currentConfiguration);
        ElevatorRight.getConfigurator().apply(currentConfiguration);
        ElevatorRight.setNeutralMode(NeutralModeValue.Brake);

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
