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
    

    public CoralIntake (CommandXboxController x){
        Manipulator = x;

        //set pid
        coralWristPID = new PID(0.1,0,0.01);
        coralWristPID.setMaxOutput(0.2);
        coralWristPID.setMinOutput(-0.2);



        //motor configs
        CoralWrist.getAbsoluteEncoder().getPosition();


        //triggers
        Trigger leftBumper = Manipulator.leftBumper();
        Trigger rightBumper = Manipulator.rightBumper();
        Trigger a = Manipulator.a();


        this.setDefaultCommand(Commands.run(()->{




            if(leftBumper.getAsBoolean()){
                CoralSpin.set(0.5);
            }else if(rightBumper.getAsBoolean()){
                CoralSpin.set(-0.5);
            }else{
                CoralSpin.set(0);
            }




        },this));   
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
