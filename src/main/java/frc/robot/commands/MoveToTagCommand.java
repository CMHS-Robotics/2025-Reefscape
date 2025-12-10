package frc.robot.commands;

import com.ctre.phoenix6.swerve.SwerveRequest;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.VisionV2;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;

import java.util.Optional;

public class MoveToTagCommand extends Command {

    private final CommandSwerveDrivetrain swerve;
    private final VisionV2 vision;

    private final SwerveRequest zero = new SwerveRequest.FieldCentric()
            .withVelocityX(0)
            .withVelocityY(0)
            .withRotationalRate(0);

    private final PIDController KPForward = new PIDController(0.3, 0.0, 0.00);
    private final PIDController KPSideways = new PIDController(0.7, 0.0, 0.00);
    private final PIDController KPRotation = new PIDController(0.7, 0.0, 0.0);

    private int framesWithoutTag = 0;
    private static final int MAX_FRAME_LOSS = 5;

    private final SendableChooser<Integer> tagChooser;

    public MoveToTagCommand(
            CommandSwerveDrivetrain swerve, 
            VisionV2 vision,
            SendableChooser<Integer> tagChooser
    ) {
        this.swerve = swerve;
        this.vision = vision;
        this.tagChooser = tagChooser;

        addRequirements(swerve);

        // Tell the vision system which tag we want to navigate to

        // PID tolerances
        KPForward.setTolerance(0.05);
        KPSideways.setTolerance(0.05);
        KPRotation.setTolerance(0.05);
    }

    @Override
    public void initialize() {
    // 1. Get the selected tag ID from the chooser
    int selectedTag = tagChooser.getSelected();
    
    // 2. Pass that selected ID to the vision subsystem
    vision.setDestinationTag(selectedTag);

    // Reset PIDs and frame loss
    KPForward.reset();
    KPSideways.reset();
    KPRotation.reset();
    framesWithoutTag = 0;
}

    @Override
    public void execute() {
        Optional<Transform3d> transformOpt = vision.getBestTransform();

        if (transformOpt.isEmpty()) {
            framesWithoutTag++;
            if (framesWithoutTag >= MAX_FRAME_LOSS) swerve.setControl(zero);
            return;
        }

        framesWithoutTag = 0;
        Transform3d transform = transformOpt.get();

        double forwardError = -transform.getX();
        double sidewaysError = -transform.getY();
        double yawError = -(transform.getRotation().toRotation2d().getRadians());

        double vx = KPForward.calculate(forwardError, 0);
        double vy = KPSideways.calculate(sidewaysError, 0);
        double omega = KPRotation.calculate(yawError, 0);

        SwerveRequest.FieldCentric request = new SwerveRequest.FieldCentric()
                .withVelocityX(vx)
                .withVelocityY(vy)
                .withRotationalRate(omega);

        swerve.setControl(request);
    }

    @Override
    public boolean isFinished() {
        Optional<Transform3d> transformOpt = vision.getBestTransform();
        if (transformOpt.isEmpty()) return false;

        Transform3d t = transformOpt.get();

        boolean closeEnough =
                Math.abs(t.getX()) < 0.1 &&
                Math.abs(t.getY()) < 0.05;

        boolean pidSettled =
                KPForward.atSetpoint() &&
                KPSideways.atSetpoint() &&
                KPRotation.atSetpoint();

        return closeEnough && pidSettled;
    }

    @Override
    public void end(boolean interrupted) {
        swerve.setControl(zero);
    }
}
