package frc.pathfinder;

import com.ctre.phoenix.motion.TrajectoryPoint;

/**
 * Add your docs here.
 */
public class Trapezoidal {
    Cord middle;

    TrajectoryPoint p1;
    TrajectoryPoint p2;

    public Trapezoidal(Cord start, Cord end) {
        this.middle = new Cord(
            (start.x + end.x) / 2,
            (start.y + end.y) / 2
        );

        this.p1 = new TrajectoryPoint();
        this.p1.position = 1;
        this.p1.auxiliaryPos = 1;
        this.p2 = new TrajectoryPoint();
        this.p1.position = 2;
        this.p2.auxiliaryPos = 1;
    }
}
