package frc.robot.targeting;

import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;

public class ShooterLookupTable {
    private final InterpolatingDoubleTreeMap mapVelocity = new InterpolatingDoubleTreeMap();
    private final InterpolatingDoubleTreeMap mapAngle = new InterpolatingDoubleTreeMap();

    public ShooterLookupTable() {
        
        this.AddPoint(3.0, 1800, 58.7);
        this.AddPoint(4.0, 2000, 50.0);
        this.AddPoint(5.0, 2200, 44.7);
        this.AddPoint(6.0, 2400, 41.0);
        this.AddPoint(7.0, 2500, 38.0);
        this.AddPoint(8.0, 2600, 34.3);
        this.AddPoint(9.0, 2650, 32.9);
        this.AddPoint(10.0, 2700, 30.8);
        this.AddPoint(11.0, 2750, 29.0);
        this.AddPoint(12.0, 2750, 27.0);
        this.AddPoint(13.0, 2700, 27.0);
        this.AddPoint(14.0, 2700, 26.0);
        this.AddPoint(15.0, 2700, 24.5);
        this.AddPoint(16.0, 2700, 24.2);
        this.AddPoint(17.0, 3000, 21.3);
        this.AddPoint(18.0, 3050, 21.1);
        this.AddPoint(19.0, 3100, 19.8);
        this.AddPoint(20.0, 3175, 19.2);
        this.AddPoint(21.0, 3250, 18.9);
        this.AddPoint(22.0, 3400, 18.3);
        this.AddPoint(23.0, 3500, 17.8);
        this.AddPoint(24.0, 3500, 17.5);
        this.AddPoint(25.0, 3500, 17.4);
        this.AddPoint(26.0, 3500, 17.3);
        this.AddPoint(27.0, 3500, 17.3); // can't quite shoot from here
    }

    public void AddPoint(double distanceFeet, double rpm, double angle_deg) {
        this.mapVelocity.put(distanceFeet, rpm);
        this.mapAngle.put(distanceFeet, angle_deg);
    }

    public double GetAngle_deg(double distance_ft) {
        return this.mapAngle.get(distance_ft);
    }

    public double GetSpeed_rpm(double distance_ft) {
        return this.mapVelocity.get(distance_ft);
    }
}
