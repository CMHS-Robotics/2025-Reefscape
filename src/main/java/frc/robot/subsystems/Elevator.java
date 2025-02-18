package frc.robot.subsystems;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.GravityTypeValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.signals.StaticFeedforwardSignValue;    

import static edu.wpi.first.units.Units.Rotations;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.Subsystem;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.commands.ElevatorFreeMoveCommand;
import frc.robot.commands.ElevatorHoldPositionCommand;
import frc.robot.tools.PID;
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
    public double[] stagesDouble = new double[4];

    public Angle motorPosition;

    public double targetPosition = 0;

    private PID elevatorPID;
    
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
        
        //set PID
        elevatorPID = new PID(0.1,0,0.3);

        elevatorPID.setMaxOutput(0.2);
        elevatorPID.setMinOutput(-0.2);
        elevatorPID.setGravity(0.035);

        stagesDouble[0]= 0;
        stagesDouble[1]= 7;
        stagesDouble[2]= 12;
        stagesDouble[3]= 15;


        //set stage levels
        Stage1 = Rotations.of(2);
        Stage2 = Rotations.of(5);
        Stage3 = Rotations.of(10);
        IntakeStage = Rotations.of(0);


        Manipulator = x;

        //ElevatorRight.setControl(new Follower(13,false));
        //ElevatorRight.set(0);

        //Make new config
        var config = new TalonFXConfiguration();
        config.MotorOutput.NeutralMode = NeutralModeValue.Brake;
        config.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
        config.Feedback.FeedbackSensorSource = FeedbackSensorSourceValue.RotorSensor;
        config.Feedback.SensorToMechanismRatio = 1;
        config.Voltage.PeakForwardVoltage = 3;
        config.Voltage.PeakReverseVoltage = 3;

        //elevator limits
        var softwarelimit = config.SoftwareLimitSwitch;
        softwarelimit.ForwardSoftLimitEnable = true;
        softwarelimit.ReverseSoftLimitEnable = true;
        softwarelimit.ReverseSoftLimitThreshold = 1;
        softwarelimit.ForwardSoftLimitThreshold = 18;

        
        //pid gains
        var slot0configs = config.Slot0;
        slot0configs.kP = 1;
        slot0configs.kI = 0;
        slot0configs.kD = 0;
        slot0configs.kV = 0;
        slot0configs.kS = 0;
        slot0configs.kG = 0;
        slot0configs.kA = 0;
        slot0configs.StaticFeedforwardSign = StaticFeedforwardSignValue.UseClosedLoopSign;
        slot0configs.GravityType = GravityTypeValue.Elevator_Static;

        //motion magic config
        var motionMagicConfigs = config.MotionMagic;
        motionMagicConfigs.MotionMagicCruiseVelocity = 40;
        motionMagicConfigs.MotionMagicAcceleration = 80;
        motionMagicConfigs.MotionMagicJerk = 800;

        //apply configs and reset motors
        ElevatorLeft.getConfigurator().apply(config);
        ElevatorLeft.setPosition(0);

        ElevatorRight.getConfigurator().apply(config);
        ElevatorRight.setPosition(0);
        
        ElevatorLeft.setPosition(Rotations.of(0));
        ElevatorRight.setPosition(Rotations.of(0));
        motorPosition = Rotations.of(0);

        ElevatorLeft.set(0);
        ElevatorRight.set(0);

        SmartDashboard.putString("Initialize Config",
        config.toString());
        
        motorPosition = Rotations.of(0);
        
        //define commands

        // Stage1Command = new ElevatorToStageCommand(this, stages[1],1);
        // Stage2Command = new ElevatorToStageCommand(this, stages[2],2);
        // Stage3Command = new ElevatorToStageCommand(this, stages[3],3);
        // IntakeStageCommand = new ElevatorToStageCommand(this, stages[0],0);

        freeMoveCommand = new ElevatorFreeMoveCommand(this);

        // holdPositionCommand = new ElevatorHoldPositionCommand(this);

        //this.setDefaultCommand(holdPositionCommand);


        //testing out motion magic with new default command
        // MotionMagicDutyCycle req = new MotionMagicDutyCycle(Rotations.of(0));
        // req.Slot = 0;

        this.setDefaultCommand(Commands.run(()->
        
        {
            SmartDashboard.putString("Command Running", "pid target");
            
            // pos = 10;
            // if(Manipulator.a().getAsBoolean()){
            //     ElevatorLeft.setControl(req.withSlot(0).withPosition(Rotations.of(pos)));
            //     ElevatorRight.setControl(new Follower(13,false));
            // }
            elevatorPID.setSetPoint(targetPosition);
            if(targetPosition <= 1 && ElevatorLeft.getPosition().getValueAsDouble() <= 1){
                ElevatorLeft.set(0);
                ElevatorRight.set(0);
            }else{
            ElevatorLeft.set(elevatorPID.updatePID(ElevatorLeft.getPosition().getValueAsDouble()));
            ElevatorRight.set(ElevatorLeft.get());
            }
            if(Math.abs(Manipulator.getLeftTriggerAxis()) > 0.1){

                targetPosition += -Manipulator.getLeftTriggerAxis() * 0.05;

            }

            if(targetPosition > 17){
                targetPosition = 17;
            }
            if(targetPosition < 1){
                targetPosition = 1;
            }

            //scuffed solution for holding
            // ElevatorLeft.set(0.05);
            // ElevatorRight.set(0.05);
            //ElevatorRight.setControl(new Follower(13,false));

        //     if(Math.abs(ElevatorLeft.getPosition().getValueAsDouble() - targetPosition) <= 0.2){
        //         ElevatorLeft.set(0.05);
        //         ElevatorRight.setControl(new Follower(13,false));
        
        //     }else{
        //         ElevatorLeft.set((motorPosition.magnitude() - ElevatorLeft.getPosition().getValueAsDouble())/4);
        //         ElevatorRight.setControl(new Follower(13,false));
        
        // }

            
        }
        
        ,this)); 


        //define triggers
        Trigger povUp = Manipulator.povUp();
        Trigger povLeft = Manipulator.povLeft();
        Trigger povRight = Manipulator.povRight();
        Trigger povDown = Manipulator.povDown();

        Trigger rightTrigger = Manipulator.rightTrigger();

        Trigger b = Manipulator.b();

        //bind commands to triggers
        // povLeft.onTrue(new ElevatorToStageCommand(this, Stage1,1));
        // povUp.onTrue(new ElevatorToStageCommand(this, Stage2,2));
        // povRight.onTrue(new ElevatorToStageCommand(this, Stage3,3));
        // povDown.onTrue(new ElevatorToStageCommand(this, IntakeStage,0));


        povLeft.onTrue(Commands.runOnce(()->{

            targetPosition = stagesDouble[1];
            stageLevel = 1;

        }));
        povUp.onTrue(Commands.runOnce(()->{

            targetPosition = stagesDouble[2];
            stageLevel = 2;

        }));
        povRight.onTrue(Commands.runOnce(()->{

            targetPosition = stagesDouble[3];
            stageLevel = 3;

        }));
        povDown.onTrue(Commands.runOnce(()->{

            targetPosition = stagesDouble[0];
            stageLevel = 0;

        }));

        rightTrigger.whileTrue(freeMoveCommand);

        //reset motor position
        b.onTrue(Commands.runOnce(()->{
            ElevatorLeft.setPosition(Rotations.of(0));
            ElevatorRight.setPosition(Rotations.of(0));
            motorPosition = Rotations.of(0);
    }));

        //resetting motors again just to bet sure
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
        //pos = (-Manipulator.getLeftY() + 1.0) * 7.0;
        SmartDashboard.putNumber("Left Motor Pos", ElevatorLeft.getPosition().getValueAsDouble());
        SmartDashboard.putNumber("Right Motor Pos", ElevatorRight.getPosition().getValueAsDouble());
        SmartDashboard.putNumber("PID Target Position",targetPosition);
        SmartDashboard.putNumber("Stage Level", stageLevel);
        SmartDashboard.putString("Left Motor Request",ElevatorLeft.getAppliedControl().toString());
        SmartDashboard.putString("Right Motor Request",ElevatorRight.getAppliedControl().toString());
        SmartDashboard.putNumber("Joystick Right Y: ", Manipulator.getRightY());
        SmartDashboard.putNumber("Joystick Left Y: ", Manipulator.getRightY());
        SmartDashboard.putString("PID control",elevatorPID.toString());
        SmartDashboard.updateValues();

    }

    // public final Command freeMove(){


    //     return Commands.run(() ->
    //         {
    //             ElevatorLeft.set(0.2 * Manipulator.getRightY());
    //             ElevatorRight.set(0.2 * Manipulator.getRightY());
    //         }
    //     );

    // }
    

    
    // public final Command stage1Command(){
    //     return Commands.run(
    //     () -> {
    //         //Manipulator.setRumble(RumbleType.kBothRumble, 100);
    //         //ElevatorLeft.setControl(pos.withPosition(Stage1));
    //         //ElevatorRight.setControl(pos.withPosition(Stage1));

    //     }

    //     );
    // }

    // public final Command stage2Command(){
    //     return Commands.runOnce(
    //     () -> {




    //          //ElevatorLeft.setControl(pos.withPosition(Stage2));
    //         //ElevatorRight.setControl(pos.withPosition(Stage2));

    //     }

    //     );
    // }

    // public final Command stage3Command(){
    //     return Commands.runOnce(
    //     () -> {

    //         //ElevatorLeft.setControl(pos.withPosition(Stage3));
    //         //ElevatorRight.setControl(pos.withPosition(Stage3));

    //     }

    //     );
    // }

    // public final Command intakeStageCommand(){
    //     return Commands.runOnce(
    //     () -> {

    //         //ElevatorLeft.setControl(pos.withPosition(IntakeStage));
    //         //ElevatorRight.setControl(pos.withPosition(IntakeStage));

    //     }

    //     );
    // }


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



