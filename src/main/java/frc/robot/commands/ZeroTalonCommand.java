package frc.robot.commands;

import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.wpilibj2.command.Command;

public class ZeroTalonCommand extends Command {
   TalonFX motor;

    public ZeroTalonCommand(TalonFX m){
      motor = m;
      
   }

   @Override
   public void initialize(){
      motor.setPosition(0);
   }

    @Override
    public void execute(){

    }
    @Override
    public boolean isFinished() {
       return true;
    }

    }
