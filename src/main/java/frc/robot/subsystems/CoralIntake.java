package frc.robot.subsystems;

import com.revrobotics.spark.SparkFlex;

import edu.wpi.first.wpilibj.motorcontrol.Spark;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.Subsystem;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.tools.PID;

public class CoralIntake implements Subsystem {
    CommandXboxController Manipulator;
    int CoralSpinMotorId = 15;
    int CoralWristMotorId = 16;
    Spark CoralSpin = new Spark(CoralSpinMotorId);
    SparkFlex CoralWrist = new SparkFlex(CoralWristMotorId,SparkFlex.MotorType.kBrushed);
    double wristTarget;
    PID coralWristPID;
    Elevator elevator;
    int elevatorStage = 0;
    double[] wristPositions = new double[4];

    

    public CoralIntake (CommandXboxController c,Elevator e){
        Manipulator = c;
        elevator = e;

        //set wrist positions
        wristPositions[0] = 0;
        wristPositions[1] = 0;
        wristPositions[2] = 0;
        wristPositions[3] = 0;


        //set pid
        coralWristPID = new PID(0.1,0,0.01);
        coralWristPID.setContinuous(true);
        coralWristPID.setMaxOutput(0.1);
        coralWristPID.setMinOutput(-0.1);



        //motor configs



        //triggers
        Trigger leftBumper = Manipulator.leftBumper();
        Trigger rightBumper = Manipulator.rightBumper();
        Trigger a = Manipulator.a();
        Trigger b = Manipulator.b();
        Trigger x = Manipulator.x();


        //default command
        this.setDefaultCommand(Commands.run(()->{

            elevatorStage = elevator.getStageLevel();

            wristTarget = switch (elevatorStage) {
                case 0 -> wristPositions[0];
                case 1 -> wristPositions[0];
                case 2 -> wristPositions[1];
                case 3 -> wristPositions[1];
                case 4 -> wristPositions[2];
                default -> wristPositions[0];
            };

            coralWristPID.setSetPoint(wristTarget);

            if(a.getAsBoolean()){
                CoralWrist.set(coralWristPID.updatePID(CoralWrist.getAbsoluteEncoder().getPosition()));                
            }else{
                CoralWrist.set(0);
            }
            if(x.getAsBoolean()){
                CoralWrist.set(-Manipulator.getRightTriggerAxis());
            }

        },this));   


        //spinning commands
        leftBumper.whileTrue(Commands.run(()->{
            CoralSpin.set(0.5);
        }));
        leftBumper.whileTrue(Commands.run(()->{
                CoralSpin.set(-0.5);
        }));
        leftBumper.and(rightBumper.whileFalse(Commands.run(()->{
            CoralSpin.set(0);
        })));
        b.onTrue(Commands.run(()->{
            
        }));

    }


    public void smartDashboard(){
        SmartDashboard.putNumber("Wrist Position",CoralWrist.getAbsoluteEncoder().getPosition());
        SmartDashboard.putNumber("Wrist Output",CoralWrist.get());
        SmartDashboard.updateValues();
    }

    @Override
    public void periodic(){
        smartDashboard();
    }



}
