package frc.robot.subsystems.DriveTrain;

public class Pose {
    public double x;
    public double y;
    public double angle;
    public double time;

    public Pose(double x, double y, double angle, double time) {
        this.x = x;
        this.y = y;
        this.angle = angle;
        this.time = time;
    }

    public void updatePose(double x, double y, double angle, double time) {
        this.x = x;
        this.y = y;
        this.angle = angle;
        this.time = time;
    }

    public static Pose getPoseBetweenPoses(Pose pose1, Pose pose2, double pos) {
        double x1 = pose1.x;
        double x2 = pose2.x;
        double y1 = pose1.y;
        double y2 = pose2.y;
        double a1 = pose1.angle;
        double a2 = pose2.angle;
        double t1 = pose1.time;
        double t2 = pose2.time;

        double x3 = x1*(1-pos) + x2*(pos);
        double y3 = y1*(1-pos) + y2*(pos);
        double t3 = t1*(1-pos) + t2*(pos);
        double a3;
        double diff = Math.abs(a1-a2);
        double angleDiff = Math.min(360-diff, diff);
        if (a1 < a2) {
            a3 = a1 + angleDiff*pos;
        } else {
            a3 = a1 - angleDiff*pos;
        }
        return new Pose(x3, y3, a3, t3);
    }
}
