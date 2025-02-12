package frc.robot.subsystems;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import static edu.wpi.first.units.Units.Rotations;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.Subsystem;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.commands.ElevatorFreeMoveCommand;
import frc.robot.commands.ElevatorHoldPositionCommand;
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

    public Angle motorPosition;
    
    // ElevatorToStageCommand Stage1Command;
    // ElevatorToStageCommand Stage2Command;
    // ElevatorToStageCommand Stage3Command;
    // ElevatorToStageCommand IntakeStageCommand;

    ElevatorFreeMoveCommand freeMoveCommand;
    ElevatorHoldPositionCommand holdPositionCommand;

    public TalonFX ElevatorLeft = new TalonFX(elevatorMotorLeftId);   
    public TalonFX ElevatorRight = new TalonFX(elevatorMotorRightId);   
    public CommandXboxController Manipulator;

 
    //ElevatorFreeMoveCommand freeMove;

    public Elevator(CommandXboxController x){
        

        
        Stage1 = Rotations.of(2);
        Stage2 = Rotations.of(4);
        Stage3 = Rotations.of(6);
        IntakeStage = Rotations.of(0);

        Manipulator = x;
        
        var config = new TalonFXConfiguration();
        config.MotorOutput.NeutralMode = NeutralModeValue.Brake;
        config.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
        config.Feedback.FeedbackSensorSource = FeedbackSensorSourceValue.RotorSensor;
        config.Feedback.SensorToMechanismRatio = 1;
        config.Voltage.PeakForwardVoltage = 3;
        config.Voltage.PeakReverseVoltage = 3;

        var softwarelimit = config.SoftwareLimitSwitch;
        softwarelimit.ForwardSoftLimitEnable = true;
        softwarelimit.ReverseSoftLimitEnable = true;
        softwarelimit.ReverseSoftLimitThreshold = 0;
        softwarelimit.ForwardSoftLimitThreshold = 10;

        var slot0configs = config.Slot0;
        slot0configs.kP = 10;
        slot0configs.kI = 0;
        slot0configs.kD = 0.5;
        slot0configs.kV = 0.1;
        slot0configs.kS = 0.025;
        //slot0configs.kG = 1;
        slot0configs.kA = 0.05;
        //slot0configs.GravityType = GravityTypeValue.Elevator_Static;

        var motionMagicConfigs = config.MotionMagic;
        motionMagicConfigs.MotionMagicCruiseVelocity = 1;
        motionMagicConfigs.MotionMagicAcceleration = .1;
        motionMagicConfigs.MotionMagicJerk = 0.01;
        ElevatorLeft.getConfigurator().apply(config);
        ElevatorLeft.setPosition(0);

        ElevatorRight.getConfigurator().apply(config);
        ElevatorRight.setPosition(0);
        
        
        ElevatorLeft.set(0);
        ElevatorRight.set(0);

        SmartDashboard.putString("Initialize Config",
        config.toString());
        

        motorPosition = Rotations.of(0);

        // Stage1Command = new ElevatorToStageCommand(this, stages[1],1);
        // Stage2Command = new ElevatorToStageCommand(this, stages[2],2);
        // Stage3Command = new ElevatorToStageCommand(this, stages[3],3);
        // IntakeStageCommand = new ElevatorToStageCommand(this, stages[0],0);
    
        freeMoveCommand = new ElevatorFreeMoveCommand(this);

        holdPositionCommand = new ElevatorHoldPositionCommand(this);

        this.setDefaultCommand(holdPositionCommand);


        Trigger povUp = Manipulator.povUp();
        Trigger povLeft = Manipulator.povLeft();
        Trigger povRight = Manipulator.povRight();
        Trigger povDown = Manipulator.povDown();

        Trigger rightAxis = Manipulator.rightTrigger();

        Trigger b = Manipulator.b();

        povLeft.onTrue(new ElevatorToStageCommand(this, Stage1,1));
        povUp.onTrue(new ElevatorToStageCommand(this, Stage2,2));
        povRight.onTrue(new ElevatorToStageCommand(this, Stage3,3));
        povDown.onTrue(new ElevatorToStageCommand(this, IntakeStage,0));

        rightAxis.whileTrue(freeMoveCommand);

        b.onTrue(Commands.runOnce(()->{
                ElevatorLeft.setPosition(Rotations.of(0));
                ElevatorRight.setPosition(Rotations.of(0));
                motorPosition = Rotations.of(0);
        },this));
        
        ElevatorRight.setPosition(0,1);
        ElevatorLeft.setPosition(0,1);
    }
        
    public final Angle distanceToMotorRot(double distance){
                double r2 = 0.289;
                double r3 = 1.6;
                double r5 = 1.2;
        
                double finalRad = 100 * distance / (r2*r5)/r3;
        
                return Rotations.of(finalRad);
    }
    
    @Override
    public void periodic(){
        SmartDashboard.putNumber("Motor Pos", ElevatorLeft.getPosition().getValueAsDouble());
        SmartDashboard.putNumber("Motor Pos var", motorPosition.baseUnitMagnitude());
        SmartDashboard.putNumber("Stage Level", stageLevel);
        SmartDashboard.putString("Motor Request",ElevatorLeft.getAppliedControl().toString());
        SmartDashboard.putString("Motor Config",ElevatorLeft.getConfigurator().toString());
        SmartDashboard.putString("Motor Description",ElevatorLeft.getDescription());
        SmartDashboard.putNumber("Joystick???: ", Manipulator.getRightY());


    }

    // public final Command freeMove(){


    //     return Commands.run(() ->
    //         {
    //             ElevatorLeft.set(0.2 * Manipulator.getRightY());
    //             ElevatorRight.set(0.2 * Manipulator.getRightY());
    //         }
    //     );

    // }
    

    
    public final Command stage1Command(){
        return Commands.run(
        () -> {
            //Manipulator.setRumble(RumbleType.kBothRumble, 100);
            //ElevatorLeft.setControl(pos.withPosition(Stage1));
            //ElevatorRight.setControl(pos.withPosition(Stage1));

        }

        );
    }

    public final Command stage2Command(){
        return Commands.runOnce(
        () -> {




             //ElevatorLeft.setControl(pos.withPosition(Stage2));
            //ElevatorRight.setControl(pos.withPosition(Stage2));

        }

        );
    }

    public final Command stage3Command(){
        return Commands.runOnce(
        () -> {

            //ElevatorLeft.setControl(pos.withPosition(Stage3));
            //ElevatorRight.setControl(pos.withPosition(Stage3));

        }

        );
    }

    public final Command intakeStageCommand(){
        return Commands.runOnce(
        () -> {

            //ElevatorLeft.setControl(pos.withPosition(IntakeStage));
            //ElevatorRight.setControl(pos.withPosition(IntakeStage));

        }

        );
    }


    // public void checkInput(){
    
    
    //     if (Manipulator.a().getAsBoolean()) {   
    //     worseLevels();
    //     }else{
    //     levels();
    //     }

    // }

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



