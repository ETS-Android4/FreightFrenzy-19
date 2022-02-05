package org.firstinspires.ftc.teamcode.PurePursuit;

import org.opencv.core.Point;

import java.util.ArrayList;

import static org.firstinspires.ftc.teamcode.PurePursuit.MovementVars.*;
import static org.firstinspires.ftc.teamcode.PurePursuit.Robot.*;
import static org.firstinspires.ftc.teamcode.PurePursuit.MathFunctions.AngleWrap;
import static org.firstinspires.ftc.teamcode.PurePursuit.MathFunctions.lineCircleIntersection;
/*
public class RobotMovement {

    public static void followCurve(ArrayList<CurvePoint> allPoints, double followAngle){
        for(int a = 0; a < allPoints.size()-1; a++){
            ComputerDebugging.sendLine(new FloatPoint(allPoints.get(a).x,allPoints.get(a).y),new FloatPoint(allPoints.get(a+1).x,allPoints.get(a+1).y));
        }
        CurvePoint followMe = getFollowPointPath(allPoints,new Point(worldXPosition,worldYPosition),allPoints.get(0).followDistance);
        ComputerDebugging.sendKeyPoint(new FloatPoint(followMe.x,followMe.y));
        goToPosition(followMe.x,followMe.y,followMe.moveSpeed,followAngle,followMe.turnSpeed);
    }

    public static CurvePoint getFollowPointPath(ArrayList<CurvePoint> pathPoints, Point robotLocation, double followRadius){

        CurvePoint followMe = new CurvePoint(pathPoints.get(0));
        for(int a = 0; a < pathPoints.size()-1; a++){
            CurvePoint startLine = pathPoints.get(a);
            CurvePoint endLine = pathPoints.get(a+1);

            ArrayList<Point> intersections = lineCircleIntersection(robotLocation,followRadius,startLine.toPoint(),endLine.toPoint());
            double closestAngle = 10000;
            int count = 0;

            for(Point thisIntersection:intersections){
                double angle = Math.atan2(thisIntersection.y-worldYPosition,thisIntersection.x-worldXPosition);
                double deltaAngle = Math.abs(MathFunctions.AngleWrap(angle-worldAngle_rad));
                if(deltaAngle<closestAngle){
                    closestAngle = deltaAngle;
                    followMe.setPoint(thisIntersection);
                }
            }
        }
        return followMe;

    }

    public static void goToPosition (double x, double y, double movementSpeed, double preferredAngle, double turnSpeed){

        double distanceToTarget = Math.hypot(x-worldXPosition,y-worldYPosition);
        double absoluteAngleToTarget = Math.atan2(y-worldYPosition, x-worldXPosition);
        double relativeAngleToPoint = AngleWrap(absoluteAngleToTarget- (worldAngle_rad-Math.toRadians(90)));
        double relativeXToPoint = Math.cos(relativeAngleToPoint)*distanceToTarget;
        double relativeYToPoint = Math.sin(relativeAngleToPoint)*distanceToTarget;
        double movementXPower = relativeXToPoint/(Math.abs(relativeXToPoint)+Math.abs(relativeYToPoint));
        double movementYPower = relativeYToPoint/(Math.abs(relativeXToPoint)+Math.abs(relativeYToPoint));
        movement_x = movementXPower*movementSpeed;
        movement_y = movementYPower*movementSpeed;
        double relativeTurnAngle = relativeAngleToPoint-Math.toRadians(180)+ preferredAngle;
        movement_turn = Range.clip(relativeTurnAngle/Math.toRadians(30),-1,1)*turnSpeed;
        if(distanceToTarget<10){
            movement_turn=0;
        }

    }

}

 */
