from shapely.geometry import Point
from shapely.geometry.polygon import Polygon
from descartes import PolygonPatch
from matplotlib import pyplot as plt

if __name__ == "__main__" :
    point = Point(3, 0.5)
    polygon = Polygon([(0, 0), (0, 1), (1, 1), (1, 0)])

    fig = plt.figure(1, figsize=(5,5), dpi=90)

    ax = fig.add_subplot(111)
    polygon_patch = PolygonPatch(polygon)
    ax.add_patch(polygon_patch)

    print(polygon.contains(point))

    plt.show()
