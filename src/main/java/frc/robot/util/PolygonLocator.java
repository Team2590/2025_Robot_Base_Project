package frc.robot.util;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import frc.robot.Constants;
import java.awt.geom.Rectangle2D;
import java.util.List;
import org.littletonrobotics.junction.Logger;

public class PolygonLocator {

  private QuadTree quadtree;

  public PolygonLocator(List<FRCPolygon> polygons, Rectangle2D bounds) {
    quadtree = new QuadTree(bounds);
    for (FRCPolygon polygon : polygons) {
      polygon.flipSide(Constants.flipside);
      quadtree.insert(polygon, polygon.getBoundingRect());
    }
  }

  public FRCPolygon findContainingPolygon(Translation2d robotPosition) {
    List<FRCPolygon> candidates =
        quadtree.query(new Rectangle2D.Double(robotPosition.getX(), robotPosition.getY(), 0, 0));

    for (FRCPolygon polygon : candidates) {
      if (polygon.contains(robotPosition)) {
        return polygon;
      }
    }
    return null;
  }

  public String getZoneOfField(Pose2d robotPose) {
    if (Constants.locator.findContainingPolygon(robotPose.getTranslation()) != null) {
      String zone = Constants.locator.findContainingPolygon(robotPose.getTranslation()).getName();

      // Logger.recordOutput("FieldZone", zone);
      return zone;
    }
    return "General Field";
  }
}
