package frc.robot.targeting.providers.limelight;

import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;

public class LimelightDistanceLookup {
    // key is ty (in degrees), value is distance in feet
    private final InterpolatingDoubleTreeMap mapDistance = new InterpolatingDoubleTreeMap();
    
    public LimelightDistanceLookup() {
        this.mapDistance.put(22.27, 4.5);
        this.mapDistance.put(19.25, 5.0);
        this.mapDistance.put(14.62, 6.0);
        this.mapDistance.put(10.87, 7.0);
        this.mapDistance.put(7.58, 8.0);
        this.mapDistance.put(5.29, 9.0);
        this.mapDistance.put(3.14, 10.0);
        this.mapDistance.put(0.92, 11.0);
        this.mapDistance.put(-0.68, 12.0);
        this.mapDistance.put(-1.81, 13.0);
        this.mapDistance.put(-2.75, 14.0);
        this.mapDistance.put(-3.72, 15.0);
        this.mapDistance.put(-5.06, 16.0);
        this.mapDistance.put(-5.98, 17.0);
        this.mapDistance.put(-6.98, 18.0);
        this.mapDistance.put(-7.90, 19.0);
        this.mapDistance.put(-8.21, 20.0);
        this.mapDistance.put(-8.70, 21.0);
        this.mapDistance.put(-9.27, 22.0);
        this.mapDistance.put(-9.95, 23.0);
        this.mapDistance.put(-10.27, 24.0);
        this.mapDistance.put(-10.50, 25.0);
        this.mapDistance.put(-10.69, 26.0);
        this.mapDistance.put(-10.77, 27.0);
    }

    public double GetDistance_ft(double ty) {
        return this.mapDistance.get(ty);
    }
}
