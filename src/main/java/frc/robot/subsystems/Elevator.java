package frc.robot.subsystems;

import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Subsystem;

/**
 * Class that extends the Phoenix 6 SwerveDrivetrain class and implements
 * Subsystem so it can easily be used in command-based projects.
 */
public class Elevator implements Subsystem {
    
    private static final int elevatorMotorLeftId = 13;
    private static final int elevatorMotorRightId = 14;

    TalonFX ElevatorLeft = new TalonFX(elevatorMotorLeftId);   
    XboxController Manipulator;

    public Elevator(XboxController bruh){
        
        ElevatorLeft.getConfigurator().apply(new TalonFXConfiguration());

        Manipulator = bruh;
        var currentConfiguration = new CurrentLimitsConfigs();
        currentConfiguration.StatorCurrentLimit = 80;
        currentConfiguration.StatorCurrentLimitEnable = true;
        ElevatorLeft.getConfigurator().refresh(currentConfiguration);
        ElevatorLeft.getConfigurator().apply(currentConfiguration);
        ElevatorLeft.setNeutralMode(NeutralModeValue.Brake);
    }   




public void checkInput(){
    if(Manipulator.getPOV() > 0){
        if(Manipulator.getPOV() == 0){
            
            ElevatorLeft.set(1);
        }else if(Manipulator.getPOV() == 180){

            ElevatorLeft.set(-1);
        }
        }
    }
}



