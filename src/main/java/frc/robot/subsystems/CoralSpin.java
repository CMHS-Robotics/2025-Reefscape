package frc.robot.subsystems;

import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.Subsystem;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;

public class CoralSpin implements Subsystem{
    
CommandXboxController Manipulator;
    int CoralSpinMotorId = 16;
    public SparkMax CoralSpin = new SparkMax(CoralSpinMotorId,SparkMax.MotorType.kBrushless);
    

    public CoralSpin (CommandXboxController c){
        Manipulator = c;
        
        //motor configs
        SparkMaxConfig spinConfig = new SparkMaxConfig();
        spinConfig
        .inverted(false)
        .idleMode(IdleMode.kCoast);

        CoralSpin.configure(spinConfig,ResetMode.kResetSafeParameters,PersistMode.kPersistParameters);


        //triggers
        Trigger leftBumper = Manipulator.leftBumper();
        Trigger rightBumper = Manipulator.rightBumper();


        //default command
        this.setDefaultCommand(Commands.run(()->{

            CoralSpin.set(0);
    
        },this));   


        //spinning commands
        leftBumper.whileTrue(Commands.run(()->{
            CoralIn();
        },this));
        rightBumper.whileTrue(Commands.run(()->{
            CoralOut(); 
        },this));
    }



    public void CoralIn(){
        CoralSpin.set(-0.3);

    }
    public void CoralOut(){
        CoralSpin.set(0.3);
    }

    public void smartDashboard(){
        SmartDashboard.putNumber("Coral Spin Output",CoralSpin.get());
        SmartDashboard.updateValues();
    }

    @Override
    public void periodic(){
        smartDashboard();
    }



}

