package frc.robot.subsystems;
//20 deg angle 

import java.io.Console;
import java.lang.reflect.Array;

import org.photonvision.PhotonCamera;
import org.photonvision.targeting.PhotonTrackedTarget;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.tools.DashboardSuite;
import frc.robot.subsystems.Elevator;




public class VisionV2 extends SubsystemBase {
    PhotonCamera FrontLeftCamera = new PhotonCamera("FrontLeftCamera");
    PhotonCamera FrontRightCamera = new PhotonCamera("FrontRightCamera");
    PhotonTrackedTarget targetL;
    PhotonTrackedTarget targetR;
    int targetIDR;
    int targetIDL;
    boolean targetsMatch;
    public VisionV2(CommandSwerveDrivetrain s, CommandXboxController d){
        CommandSwerveDrivetrain swerve = s;
        CommandXboxController driver = d;
    }
    public int GetCameraTarget(){
        var resultL = FrontLeftCamera.getLatestResult();
        var resultR = FrontRightCamera.getLatestResult();

        if(resultL.hasTargets() || resultR.hasTargets()){
            PhotonTrackedTarget targetL = resultL.getBestTarget();
            PhotonTrackedTarget targetR = resultR.getBestTarget();
            int targetIDR = targetR.getFiducialId();
            int targetIDL = targetL.getFiducialId();
            
        }
        if(targetIDL == targetIDR){
            targetsMatch=true;
        }else{
            targetIDL = -1;
            targetsMatch=false;
        }
        
        return targetIDL;
        
    }

    @Override
    public void periodic(){
        GetCameraTarget();
        SmartDashboard.putBoolean("HasSameTargetId", targetsMatch);
        SmartDashboard.putNumber("SameTargetId", targetIDR);
    }
}