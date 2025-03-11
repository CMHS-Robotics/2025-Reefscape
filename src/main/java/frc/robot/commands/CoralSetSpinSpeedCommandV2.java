package frc.robot.commands;

import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.CoralSpinV2;
public class CoralSetSpinSpeedCommandV2 extends Command {
    CoralSpinV2 CoralIntakeV2;

    TalonFX CoralIntakeLeft,CoralIntakeRight;
    
    double speed;
    public CoralSetSpinSpeedCommandV2(CoralSpinV2 c, double s){
        CoralIntakeV2 = c;
        speed = s;
        CoralIntakeRight = CoralIntakeV2.CoralIntakeRight;
        CoralIntakeLeft = CoralIntakeV2.CoralIntakeLeft;
        addRequirements(CoralIntakeV2);
    }

    @Override
    public void execute(){
        CoralIntakeLeft.set(-speed);
        CoralIntakeRight.set(speed);
    }

    @Override
    public void end(boolean interrupted){
        CoralIntakeLeft.set(0);
        CoralIntakeRight.set(0);
    }
}
