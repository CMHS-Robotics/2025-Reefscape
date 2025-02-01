package frc.robot;

import static edu.wpi.first.units.Units.*;

import edu.wpi.first.units.measure.Angle;

public class elevatorconversiontemp {
    public static final Angle distanceToMotorRot(double distance){
        double r2 = 0.289;
        double r3 = 1.6;
        double r5 = 1.2;

        double finalRad = distance * (2*Math.PI*r2*r5)/r3;

        return Radian.ofBaseUnits(finalRad);
    }
}
