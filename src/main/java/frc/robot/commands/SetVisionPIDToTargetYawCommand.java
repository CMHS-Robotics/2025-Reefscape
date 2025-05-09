package frc.robot.commands;

import org.photonvision.targeting.PhotonTrackedTarget;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Vision;
import frc.robot.subsystems.Vision.CAMERA;
import org.photonvision.utils.*;

public class SetVisionPIDToTargetYawCommand extends Command {
    
    Vision vision;
    int id;
    boolean idTarget = false;
    PhotonTrackedTarget target;
    double targetYaw = 0;
    boolean skip = false;

    public SetVisionPIDToTargetYawCommand( Vision v){
        vision = v;
        idTarget = false;
        addRequirements(vision);
    }
    
    public SetVisionPIDToTargetYawCommand(Vision v, int id){
        vision = v;
        idTarget = true;
        this.id = id;
        addRequirements(vision);
    }

    @Override
    public void initialize(){

    }

    @Override
    public void execute(){

        skip = (idTarget)?!vision.hasTarget(CAMERA.FRONT,id) :!vision.hasTarget(CAMERA.FRONT);

        if(!skip){

            if(idTarget){


            }
            else{
            target = vision.getTarget(CAMERA.FRONT);
            targetYaw = target.getYaw();
           }
           vision.turnTrackingPID.setSetPoint(targetYaw);
        }
    }

    public double getYaw(){
        return targetYaw;
    }
}
