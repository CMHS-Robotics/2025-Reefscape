package frc.robot.subsystems;

import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.wpilibj.motorcontrol.Spark;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.Subsystem;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;

public class CoralIntake implements Subsystem {
    CommandXboxController Manipulator;
    int CoralSpinMotorId = 15;
    int CoralWristMotorId = 16;
    Spark CoralSpin = new Spark(CoralSpinMotorId);
    Spark CoralWrist = new Spark(CoralWristMotorId);
    

    public CoralIntake (CommandXboxController x){
        Manipulator = x;

        var config = new SparkMaxConfig();
        Trigger leftBumper = Manipulator.leftBumper();
        Trigger rightBumper = Manipulator.rightBumper();


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
}
