package frc.robot.subsystems;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.units.Units;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.Subsystem;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;

/**
 * Class that extends the Phoenix 6 SwerveDrivetrain class and implements
 * Subsystem so it can easily be used in command-based projects.
 */
public class Elevator implements Subsystem {
    private final int elevatorMotorLeftId = 13;
    private final int elevatorMotorRightId = 14;

    Angle Stage1 = cmToDegrees(30);
    Angle Stage2 = cmToDegrees(30);
    Angle Stage3 = cmToDegrees(30);
    Angle IntakeStage = cmToDegrees(0);

    private int stageLevel = 0;

    Angle[] stages = {IntakeStage,Stage1,Stage2,Stage3};

    TalonFX ElevatorLeft = new TalonFX(elevatorMotorLeftId);   
    TalonFX ElevatorRight = new TalonFX(elevatorMotorRightId);   
    CommandXboxController Manipulator;
    MotionMagicVoltage pos;
    Trigger povUp = Manipulator.povUp();
    Trigger povLeft = Manipulator.povLeft();
    Trigger povRight = Manipulator.povRight();
    Trigger povDown = Manipulator.povDown();

    

    public Elevator(CommandXboxController bruh){
        
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

        this.setDefaultCommand(freeMove());

        povLeft.onTrue(stage1Command());
        povUp.onTrue(stage2Command());
        povRight.onTrue(stage3Command());
        povDown.onTrue(intakeStageCommand());

    }
    
    public final Command freeMove(){

        return Commands.run(() ->

            {
                ElevatorLeft.set(0.2 * Manipulator.getRightY());
                ElevatorRight.set(0.2 * Manipulator.getRightY());

            }
        );

    }
    
    @Override
    public void periodic(){

    }



    // public void checkInput(){
    
    
    //     if (Manipulator.a().getAsBoolean()) {   
    //     worseLevels();
    //     }else{
    //     levels();
    //     }

    // }


    public Angle cmToDegrees(int Cm){
        Angle bruh = Angle.ofBaseUnits(Cm, Units.Degrees);

       bruh.times(10);
        
        return bruh;
    }

    public Command stage1Command(){
        return Commands.runOnce(
        () -> {

            ElevatorLeft.setControl(pos.withPosition(Stage1));
            ElevatorRight.setControl(pos.withPosition(Stage1));

        }

        );
    }
    public Command stage2Command(){
        return Commands.runOnce(
        () -> {

            ElevatorLeft.setControl(pos.withPosition(Stage2));
            ElevatorRight.setControl(pos.withPosition(Stage2));

        }

        );
    }
    public Command stage3Command(){
        return Commands.runOnce(
        () -> {

            ElevatorLeft.setControl(pos.withPosition(Stage3));
            ElevatorRight.setControl(pos.withPosition(Stage3));

        }

        );
    }
    public Command intakeStageCommand(){
        return Commands.runOnce(
        () -> {

            ElevatorLeft.setControl(pos.withPosition(IntakeStage));
            ElevatorRight.setControl(pos.withPosition(IntakeStage));

        }

        );
    }

    // public void levels(){
    //     if(Manipulator.getPOV() == 270){
    //         ElevatorLeft.setControl(pos.withPosition(Stage1));
    //         ElevatorRight.setControl(pos.withPosition(Stage1));
    //     }
    //     if(Manipulator.getPOV() == 0){
    //         ElevatorLeft.setControl(pos.withPosition(Stage2));
    //         ElevatorRight.setControl(pos.withPosition(Stage2));
    //     }
    //     if(Manipulator.getPOV() == 90){
    //         ElevatorLeft.setControl(pos.withPosition(Stage3));
    //         ElevatorRight.setControl(pos.withPosition(Stage3));
    //     }
    //     if(Manipulator.getPOV() == 180){
    //         ElevatorLeft.setControl(pos.withPosition(IntakeStage));
    //         ElevatorRight.setControl(pos.withPosition(IntakeStage));
    //     }


    // }

    // public void worseLevels(){
    //     if(Manipulator.getPOV() == 270){
    //         stageLevel = 1;
    //         ElevatorLeft.setPosition(Stage1);
    //     }
    //     if(Manipulator.getPOV() == 0){
    //         stageLevel = 2;
    //         ElevatorLeft.setPosition(Stage2);
    //     }
    //     if(Manipulator.getPOV() == 90){
    //         stageLevel = 3;
    //         ElevatorLeft.setPosition(Stage3);
    //     }
    //     if(Manipulator.getPOV() == 180){
    //         stageLevel = 0;
    //         ElevatorLeft.setPosition(IntakeStage);
    //     }

    //     if(ElevatorLeft.getPosition() != stages[stageLevel]){
            
    //     }



    // }


}



