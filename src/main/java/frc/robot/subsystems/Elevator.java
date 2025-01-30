package frc.robot.subsystems;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Subsystem;

/**
 * Class that extends the Phoenix 6 SwerveDrivetrain class and implements
 * Subsystem so it can easily be used in command-based projects.
 */
public class Elevator implements Subsystem {
    private final int elevatorMotorLeftId = 13;
    private final int elevatorMotorRightId = 14;

    private final double Stage1 = CmToDegrees(12);
    private final double Stage2 = CmToDegrees(40);
    private final double Stage3 = CmToDegrees(30);
    private final double IntakeStage = CmToDegrees(0);

    private int stageLevel = 0;

    TalonFX ElevatorLeft = new TalonFX(elevatorMotorLeftId);   
    TalonFX ElevatorRight = new TalonFX(elevatorMotorRightId);   
    XboxController Manipulator;
    MotionMagicVoltage pos;

    @Override
    public void periodic(){

    }

    public Elevator(XboxController bruh){
        
        //Stuff to set up a motor to be able to spin - replace elevatorLeft with motor name

        ElevatorLeft.getConfigurator().apply(new TalonFXConfiguration());

        Manipulator = bruh;

        
        var config = new TalonFXConfiguration();
        config.MotorOutput.NeutralMode = NeutralModeValue.Brake;
        var slot0configs = config.Slot0;
        slot0configs.kP = 1;
        slot0configs.kI = 0;
        slot0configs.kD = 0.1;
        var motionMagicConfigs = config.MotionMagic;
        motionMagicConfigs.MotionMagicCruiseVelocity = 60;
        motionMagicConfigs.MotionMagicAcceleration = 180;
        motionMagicConfigs.MotionMagicJerk = 0;

        ElevatorLeft.getConfigurator().refresh(config);
        ElevatorLeft.getConfigurator().apply(config);
        ElevatorLeft.setPosition(0);
    
        ElevatorRight.getConfigurator().refresh(config);
        ElevatorRight.getConfigurator().apply(config);
        ElevatorRight.setPosition(0);

        pos = new MotionMagicVoltage(0);
    }   

public void checkInput(){
    levels();

    ElevatorLeft.set(0.2 * Manipulator.getRightY());
    ElevatorRight.set(0.2 * Manipulator.getRightY());

}


    public double CmToDegrees(double Cm){
        double bruh = Cm;

        bruh *= 122;
        
        return bruh;
    }

    public void levels(){
        if(Manipulator.getPOV() == 270){
            ElevatorLeft.setControl(pos.withPosition(Stage1));
            ElevatorRight.setControl(pos.withPosition(Stage1));
        }
        if(Manipulator.getPOV() == 0){
            ElevatorLeft.setControl(pos.withPosition(Stage2));
            ElevatorRight.setControl(pos.withPosition(Stage2));
        }
        if(Manipulator.getPOV() == 90){
            ElevatorLeft.setControl(pos.withPosition(Stage3));
            ElevatorRight.setControl(pos.withPosition(Stage3));
        }
        if(Manipulator.getPOV() == 180){
            ElevatorLeft.setControl(pos.withPosition(IntakeStage));
            ElevatorRight.setControl(pos.withPosition(IntakeStage));
        }


    }


}



