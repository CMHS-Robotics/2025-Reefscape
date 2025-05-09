package frc.robot.commands;

import java.util.Vector;

import org.photonvision.PhotonUtils;
import org.photonvision.targeting.PhotonTrackedTarget;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.MatBuilder;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.Nat;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N2;
import edu.wpi.first.units.Unit;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Vision;
import frc.robot.subsystems.Vision.CAMERA;

public class SetVisionPIDToTargetRotationAndCenterCommand extends Command {
    
    Vision vision;
    int id;
    boolean idTarget = false;
    PhotonTrackedTarget target;
    double targetX = 0;
    double targetY = 0;
    double targetRot = 0;
    double targetSlope = 0;
    double driveSlope = 0;
    Pose3d targetPose;
    Pose2d robotPose;
    boolean skip = false;
    double intersectX = 0;
    double intersectY = 0;

    public SetVisionPIDToTargetRotationAndCenterCommand( Vision v){
        vision = v;
        idTarget = false;
        addRequirements(vision);
    }
    
    public SetVisionPIDToTargetRotationAndCenterCommand(Vision v, int id){
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
            targetRot = target.getBestCameraToTarget().getRotation().toRotation2d().getDegrees();
            targetX = target.getBestCameraToTarget().getX();
            targetY = target.getBestCameraToTarget().getY();
            targetPose = AprilTagFieldLayout.loadField(AprilTagFields.k2025ReefscapeWelded).getTags().get(target.getFiducialId()).pose;
            targetSlope = Math.tan(targetPose.getRotation().toRotation2d().getRadians());
            driveSlope = -1/targetSlope;
            robotPose = vision.getRobotPose();
            
            //matrix: row 1 is target pos, row 2 is robot pos
           Matrix<N2,N2> coeffmatrix = MatBuilder.fill(Nat.N2(),Nat.N2(),
           
           -targetSlope,1.0,
            -driveSlope,1.0
           
           );

           Matrix<N2,N1> ansmatrix = MatBuilder.fill(Nat.N2(),Nat.N1(),
           
           -targetSlope*targetPose.getX()+targetPose.getY(),
           -driveSlope*robotPose.getX()+robotPose.getY()
           
           );

           Matrix<N2,N1> solmatrix = coeffmatrix.inv().times(ansmatrix);

           intersectX = solmatrix.get(0,0);
           intersectY = solmatrix.get(1,0);

            vision.turnTrackingPID.setSetPoint(targetRot);
            vision.xTranslatePID.setSetPoint(intersectX - robotPose.getX());
            vision.yTranslatePID.setSetPoint(intersectY - robotPose.getY());
}
    }
}
