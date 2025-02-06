package frc.robot.subsystems;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import static edu.wpi.first.units.Units.Radian;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj2.command.Subsystem;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.commands.ElevatorFreeMoveCommand;
import frc.robot.commands.ElevatorToStageCommand;

/**
 * Class that extends the Phoenix 6 SwerveDrivetrain class and implements
 * Subsystem so it can easily be used in command-based projects.
 */
public class Elevator implements Subsystem {
    private final int elevatorMotorLeftId = 13;
    private final int elevatorMotorRightId = 14;

    public Angle Stage1 = distanceToMotorRot(10);
    public Angle Stage2 = distanceToMotorRot(20);
    public Angle Stage3 = distanceToMotorRot(30);
    public Angle IntakeStage = distanceToMotorRot(0);

    public int stageLevel = 0;

    public Angle[] stages = {IntakeStage,Stage1,Stage2,Stage3};

    
    // ElevatorToStageCommand Stage1Command;
    // ElevatorToStageCommand Stage2Command;
    // ElevatorToStageCommand Stage3Command;
    // ElevatorToStageCommand IntakeStageCommand;
    ElevatorFreeMoveCommand freeMoveCommand;
    

    public TalonFX ElevatorLeft = new TalonFX(elevatorMotorLeftId);   
    public TalonFX ElevatorRight = new TalonFX(elevatorMotorRightId);   
    public CommandXboxController Manipulator;
 
    //ElevatorFreeMoveCommand freeMove;

    public Elevator(CommandXboxController x){
        
        //Stuff to set up a motor to be able to spin - replace elevatorLeft with motor name

        ElevatorLeft.getConfigurator().apply(new TalonFXConfiguration());

        Manipulator = x;

        

        
        var config = new TalonFXConfiguration();
        config.MotorOutput.NeutralMode = NeutralModeValue.Brake;
        config.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
        var slot0configs = config.Slot0;
        slot0configs.kP = 30;
        slot0configs.kI = 0;
        slot0configs.kD = 3;
        var motionMagicConfigs = config.MotionMagic;
        motionMagicConfigs.MotionMagicCruiseVelocity = 60;
        motionMagicConfigs.MotionMagicAcceleration = 20;
        motionMagicConfigs.MotionMagicJerk = 0;

        ElevatorLeft.getConfigurator().refresh(config);
        ElevatorLeft.getConfigurator().apply(config);
        ElevatorLeft.setPosition(0);

        ElevatorRight.getConfigurator().refresh(config);
        ElevatorRight.getConfigurator().apply(config);
        ElevatorRight.setPosition(0);
        
        // Stage1Command = new ElevatorToStageCommand(this, Stage1);
        // Stage2Command = new ElevatorToStageCommand(this, Stage2);
        // Stage3Command = new ElevatorToStageCommand(this, Stage3);
        // IntakeStageCommand = new ElevatorToStageCommand(this, IntakeStage);
    
        freeMoveCommand = new ElevatorFreeMoveCommand(this);

        this.setDefaultCommand(new ElevatorFreeMoveCommand(this));


        Trigger povUp = Manipulator.povUp();
        Trigger povLeft = Manipulator.povLeft();
        Trigger povRight = Manipulator.povRight();
        Trigger povDown = Manipulator.povDown();

        Trigger rightStick = Manipulator.rightStick();

        povLeft.onTrue(new ElevatorToStageCommand(this, Stage1,1));
        povUp.onTrue(new ElevatorToStageCommand(this, Stage2,2));
        povRight.onTrue(new ElevatorToStageCommand(this, Stage3,3));
        povDown.onTrue(new ElevatorToStageCommand(this, IntakeStage,0));

        //rightStick.whileTrue(freeMoveCommand);
    
    }
        
    public final Angle distanceToMotorRot(double distance){
                double r2 = 0.289;
                double r3 = 1.6;
                double r5 = 1.2;
        
                double finalRad = 100 * distance / (2*Math.PI*r2*r5)/r3;
        
                return Radian.of(finalRad);
    }
    
    @Override
    public void periodic(){
    }

    // public final Command freeMove(){

    //     return freeMove;

    //     // return Commands.run(() ->

    //     //     {
    //     //         ElevatorLeft.set(0.2 * Manipulator.getRightY());
    //     //         ElevatorRight.set(0.2 * Manipulator.getRightY());

    //     //     }
    //     // );

    // }
    

    
//     public final Command stage1Command(){
//         return Commands.run(
//         () -> {
//             Manipulator.setRumble(RumbleType.kBothRumble, 100);
//             //ElevatorLeft.setControl(pos.withPosition(Stage1));
//             //ElevatorRight.setControl(pos.withPosition(Stage1));

//         }

//         );
//     }

//     public final Command stage2Command(){
//         return Commands.runOnce(
//         () -> {




             //ElevatorLeft.setControl(pos.withPosition(Stage2));
//             //ElevatorRight.setControl(pos.withPosition(Stage2));

//         }

//         );
//     }

//     public final Command stage3Command(){
//         return Commands.runOnce(
//         () -> {

//             //ElevatorLeft.setControl(pos.withPosition(Stage3));
//             //ElevatorRight.setControl(pos.withPosition(Stage3));

//         }

//         );
//     }

//     public final Command intakeStageCommand(){
//         return Commands.runOnce(
//         () -> {

//             //ElevatorLeft.setControl(pos.withPosition(IntakeStage));
//             //ElevatorRight.setControl(pos.withPosition(IntakeStage));

//         }

//         );
//     }


//     // public void checkInput(){
    
    
//     //     if (Manipulator.a().getAsBoolean()) {   
//     //     worseLevels();
//     //     }else{
//     //     levels();
//     //     }

//     // }

//     // public void levels(){
//     //     if(Manipulator.getPOV() == 270){
//     //         ElevatorLeft.setControl(pos.withPosition(Stage1));
//     //         ElevatorRight.setControl(pos.withPosition(Stage1));
//     //     }
//     //     if(Manipulator.getPOV() == 0){
//     //         ElevatorLeft.setControl(pos.withPosition(Stage2));
//     //         ElevatorRight.setControl(pos.withPosition(Stage2));
//     //     }
//     //     if(Manipulator.getPOV() == 90){
//     //         ElevatorLeft.setControl(pos.withPosition(Stage3));
//     //         ElevatorRight.setControl(pos.withPosition(Stage3));
//     //     }
//     //     if(Manipulator.getPOV() == 180){
//     //         ElevatorLeft.setControl(pos.withPosition(IntakeStage));
//     //         ElevatorRight.setControl(pos.withPosition(IntakeStage));
//     //     }


//     // }

//     // public void worseLevels(){
//     //     if(Manipulator.getPOV() == 270){
//     //         stageLevel = 1;
//     //         ElevatorLeft.setPosition(Stage1);
//     //     }
//     //     if(Manipulator.getPOV() == 0){
//     //         stageLevel = 2;
//     //         ElevatorLeft.setPosition(Stage2);
//     //     }
//     //     if(Manipulator.getPOV() == 90){
//     //         stageLevel = 3;
//     //         ElevatorLeft.setPosition(Stage3);
//     //     }
//     //     if(Manipulator.getPOV() == 180){
//     //         stageLevel = 0;
//     //         ElevatorLeft.setPosition(IntakeStage);
//     //     }

//     //     if(ElevatorLeft.getPosition() != stages[stageLevel]){
            
//     //     }



//     // }


 }



