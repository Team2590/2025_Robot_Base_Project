package frc.robot.util;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import java.awt.geom.Rectangle2D;
import java.util.List;
import java.util.Optional;

public class PolygonLocator {

  private QuadTree quadtree;
  private List<FRCPolygon> polygons;
  private Rectangle2D bounds;

  public PolygonLocator(List<FRCPolygon> polygons, Rectangle2D bounds) {
    this.bounds = bounds;
    this.polygons = polygons;
    quadtree = new QuadTree(bounds);
    for (FRCPolygon polygon : polygons) {
      polygon.flipSide(
          DriverStation.getAlliance().isPresent()
              && DriverStation.getAlliance().get() == Alliance.Red);
      quadtree.insert(polygon, polygon.getBoundingRect());
    }
  }

  public FRCPolygon findContainingPolygon(
      Translation2d robotPosition, Optional<Alliance> alliance) {
    List<FRCPolygon> candidates =
        quadtree.query(new Rectangle2D.Double(robotPosition.getX(), robotPosition.getY(), 0, 0));

    for (FRCPolygon polygon : candidates) {
      if (polygon.contains(robotPosition)) {
        return polygon;
      }
    }

    quadtree = new QuadTree(bounds);
    for (FRCPolygon polygon : polygons) {
      if (alliance.isPresent()) {
        polygon.flipSide(alliance.get().equals(Alliance.Red));
      }
      quadtree.insert(polygon, polygon.getBoundingRect());
    }
    return null;
  }

  // public String getZoneOfField(Pose2d robotPose) {
  //   if (findContainingPolygon(robotPose.getTranslation()) != null) {
  //     String zone = findContainingPolygon(robotPose.getTranslation()).getName();

  //     // Logger.recordOutput("FieldZone", zone);
  //     return zone;
  //   }
  //   return "General Field";
  // }
}
