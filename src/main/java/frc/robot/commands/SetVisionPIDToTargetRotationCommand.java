package frc.robot.commands;

import org.photonvision.targeting.PhotonTrackedTarget;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Vision;
import frc.robot.subsystems.Vision.CAMERA;

public class SetVisionPIDToTargetRotationCommand extends Command {
    
    Vision vision;
    int id;
    boolean idTarget = false;
    PhotonTrackedTarget target;
    double targetYaw = 0;
    boolean skip = false;

    public SetVisionPIDToTargetRotationCommand( Vision v){
        vision = v;
        idTarget = false;
        addRequirements(vision);
    }
    
    public SetVisionPIDToTargetRotationCommand(Vision v, int id){
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
           }
           vision.turnTrackingPID.setSetPoint(AprilTagFieldLayout.loadField(AprilTagFields.k2025ReefscapeWelded).getTags().get(target.getFiducialId()).pose.getRotation().toRotation2d().getDegrees());
        }
    }
}
