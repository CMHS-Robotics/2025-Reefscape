package frc.robot.subsystems;

import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Subsystem;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.commands.CoralSetSpinSpeedCommand;

public class CoralSpin implements Subsystem{
    
CommandXboxController Manipulator;
    int CoralSpinMotorId = 16;
    public SparkMax CoralSpin = new SparkMax(CoralSpinMotorId,SparkMax.MotorType.kBrushless);
    //public TalonFX CoralSpin = new TalonFX(CoralSpinMotorId);
    CoralSetSpinSpeedCommand NoSpin;
    CoralSetSpinSpeedCommand InSpin;
    CoralSetSpinSpeedCommand OutSpin;
    

    public CoralSpin (CommandXboxController c){
        Manipulator = c;
        
        //motor configs
        SparkMaxConfig spinConfig = new SparkMaxConfig();
        spinConfig
        .inverted(false)
        .idleMode(IdleMode.kBrake);

        CoralSpin.configure(spinConfig,ResetMode.kResetSafeParameters,PersistMode.kPersistParameters);

        // var config = new TalonFXConfiguration();
        // config.MotorOutput.NeutralMode = NeutralModeValue.Brake;

        // CoralSpin.getConfigurator().apply(config);

        //triggers
        Trigger leftBumper = Manipulator.leftBumper();
        Trigger rightBumper = Manipulator.rightBumper();


        // //default command
        // this.setDefaultCommand(Commands.run(()->{

        //     CoralSpin.set(0);
    
        // },this));   

    
        // //spinning commands
        // leftBumper.whileTrue(Commands.run(()->{
        //     CoralIn();
        // },this));
        // rightBumper.whileTrue(Commands.run(()->{
        //     CoralOut(); 
        // },this));


        //define commands
        NoSpin = new CoralSetSpinSpeedCommand(this,0);
        InSpin = new CoralSetSpinSpeedCommand(this,-0.3);
        OutSpin = new CoralSetSpinSpeedCommand(this,0.3);


        //default command
        this.setDefaultCommand(NoSpin);   


        //spinning commands
        leftBumper.whileTrue(InSpin);
        rightBumper.whileTrue(OutSpin);


    }



    public void CoralIn(){
        CoralSpin.set(-0.3);

    }
    public void CoralOut(){
        CoralSpin.set(0.3);
    }

    public void smartDashboard(){
        SmartDashboard.putNumber("Coral Spin Output",CoralSpin.get());
        SmartDashboard.putNumber("coralspin applied output",CoralSpin.getAppliedOutput());
        SmartDashboard.putNumber("coralspin bus voltage",CoralSpin.getBusVoltage());
        SmartDashboard.putNumber("coralspin temperature",CoralSpin.getMotorTemperature());
        SmartDashboard.putNumber("coralspin current",CoralSpin.getOutputCurrent());
        SmartDashboard.updateValues();
    }

    @Override
    public void periodic(){
        smartDashboard();
    }



}

