package frc.robot.commands;

import com.revrobotics.spark.SparkMax;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.CoralSpin;

public class CoralSetSpinSpeedCommand extends Command{

    SparkMax coralSpin;
    CoralSpin SpinSystem;
    double speed;
    double wait = 0;

    public CoralSetSpinSpeedCommand(CoralSpin c,double s){
        SpinSystem = c;
        coralSpin = SpinSystem.CoralSpin;
        speed = s;
        addRequirements(SpinSystem);
    }

    public CoralSetSpinSpeedCommand(CoralSpin c,double s,double w){
        SpinSystem = c;
        coralSpin = SpinSystem.CoralSpin;
        speed = s;
        addRequirements(SpinSystem);
    }
    

    public void Initialize(){
        Timer timer = new Timer();
        timer.start(); 
    }

    @Override
    public void execute(){
        coralSpin.set(speed);
    }

    @Override
    public boolean isFinished(){
        if(wait != 0){
        return Timer.getTimestamp() >= wait;
        }else{
            return false;
        }
    }

}
