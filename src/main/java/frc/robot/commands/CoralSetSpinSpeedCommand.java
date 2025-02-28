package frc.robot.commands;

import com.revrobotics.spark.SparkMax;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.CoralSpin;

public class CoralSetSpinSpeedCommand extends Command{

    SparkMax coralSpin;
    CoralSpin SpinSystem;
    double speed;

    public CoralSetSpinSpeedCommand(CoralSpin c,double s){
        SpinSystem = c;
        coralSpin = SpinSystem.CoralSpin;
        speed = s;
        addRequirements(SpinSystem);
    }
    

    public void Initialize(){

    }

    @Override
    public void execute(){
        coralSpin.set(speed);
    }



}
