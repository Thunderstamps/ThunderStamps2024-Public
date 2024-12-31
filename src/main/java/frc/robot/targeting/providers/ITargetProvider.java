package frc.robot.targeting.providers;

import java.util.OptionalDouble;

public interface ITargetProvider {

    // Returns angle to target, in degrees
    OptionalDouble getTheta_rad();

    // Returns distance to target, in feet
    OptionalDouble getDistance_ft();

}
