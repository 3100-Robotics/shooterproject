package frc.robot;

import edu.wpi.first.math.util.Units;

public class Constants {
    public class Shooter {
        public static double MOI = 
        (1f/2f)*Units.lbsToKilograms(1.543)*(Math.pow(Units.inchesToMeters(3.95)/2,2f)) + // SDS Flywheel
        2*( (1f/2f)*Units.lbsToKilograms(0.22)*(Math.pow(Units.inchesToMeters(3.965)/2,2f)) ) // 2x am-2647
        ;
    }

    public class Indexer {
        public static double MOI = 
        (1f/2f)*Units.lbsToKilograms(0.1)*(Math.pow(Units.inchesToMeters(1.965)/2,2f)); // am-3155
    }
}
