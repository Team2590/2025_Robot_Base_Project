package frc.robot.util;

import java.awt.geom.Rectangle2D;
import java.util.ArrayList;
import java.util.List;

public class QuadTree {

  private static final int MAX_OBJECTS = 10;
  private static final int MAX_LEVELS = 5;

  private int level;
  private List<FRCPolygon> objects;
  private Rectangle2D bounds;
  private QuadTree[] nodes;

  public QuadTree(Rectangle2D bounds) {
    this(0, bounds);
  }

  private QuadTree(int level, Rectangle2D bounds) {
    this.level = level;
    this.objects = new ArrayList<>();
    this.bounds = bounds;
    this.nodes = new QuadTree[4];
  }

  public void insert(FRCPolygon object, Rectangle2D objectBounds) {
    if (nodes[0] != null) {
      int index = getIndex(objectBounds);
      if (index != -1) {
        nodes[index].insert(object, objectBounds);
        return;
      }
    }

    objects.add(object);

    if (objects.size() > MAX_OBJECTS && level < MAX_LEVELS) {
      if (nodes[0] == null) {
        split();
      }

      int i = 0;
      while (i < objects.size()) {
        int index = getIndex(objectBounds);
        if (index != -1) {
          nodes[index].insert(objects.remove(i), objectBounds);
        } else {
          i++;
        }
      }
    }
  }

  public List<FRCPolygon> query(Rectangle2D searchBounds) {
    List<FRCPolygon> result = new ArrayList<>();
    getIndex(searchBounds, result);
    return result;
  }

  private void split() {
    double subWidth = bounds.getWidth() / 2;
    double subHeight = bounds.getHeight() / 2;
    double x = bounds.getX();
    double y = bounds.getY();

    QuadTree[] newNodes = new QuadTree[4];
    newNodes[0] =
        new QuadTree(level + 1, new Rectangle2D.Double(x + subWidth, y, subWidth, subHeight));
    newNodes[1] = new QuadTree(level + 1, new Rectangle2D.Double(x, y, subWidth, subHeight));
    newNodes[2] =
        new QuadTree(level + 1, new Rectangle2D.Double(x, y + subHeight, subWidth, subHeight));
    newNodes[3] =
        new QuadTree(
            level + 1, new Rectangle2D.Double(x + subWidth, y + subHeight, subWidth, subHeight));

    nodes = newNodes;
  }

  private int getIndex(Rectangle2D objectBounds) {
    int index = -1;
    double verticalMidpoint = bounds.getX() + (bounds.getWidth() / 2);
    double horizontalMidpoint = bounds.getY() + (bounds.getHeight() / 2);

    boolean topQuadrant =
        (objectBounds.getY() < horizontalMidpoint
            && objectBounds.getY() + objectBounds.getHeight() < horizontalMidpoint);
    boolean bottomQuadrant = (objectBounds.getY() > horizontalMidpoint);

    if (objectBounds.getX() < verticalMidpoint
        && objectBounds.getX() + objectBounds.getWidth() < verticalMidpoint) {
      if (topQuadrant) {
        index = 1;
      } else if (bottomQuadrant) {
        index = 2;
      }
    } else if (objectBounds.getX() > verticalMidpoint) {
      if (topQuadrant) {
        index = 0;
      } else if (bottomQuadrant) {
        index = 3;
      }
    }

    return index;
  }

  private void getIndex(Rectangle2D searchBounds, List<FRCPolygon> result) {
    int index = getIndex(searchBounds);
    if (index != -1 && nodes[0] != null) {
      nodes[index].getIndex(searchBounds, result);
    }

    result.addAll(objects);
  }
}
