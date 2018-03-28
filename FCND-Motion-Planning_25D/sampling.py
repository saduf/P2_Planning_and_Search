import numpy as np
from sklearn.neighbors import KDTree
from shapely.geometry import Polygon, Point

class Poly:

    def __init__(self, coords, height):
        self._polygon = Polygon(coords)
        self._height = height

    @property
    def height(self):
        return self._height

    @property
    def coords(self):
        return list(self._polygon.exterior.coords)[:-1]
    
    @property
    def area(self):
        return self._polygon.area

    @property
    def center(self):
        return (self._polygon.centroid.x, self._polygon.centroid.y)

    def contains(self, point):
        point = Point(point)
        return self._polygon.contains(point)

    def crosses(self, other):
        return self._polygon.crosses(other)



def extract_polygons(data):

    polygons = []
    # Center offset for grid
    north_min_center = np.min(data[:, 0])
    east_min_center = np.min(data[:, 1])
    SAFETY_DISTANCE = 7
    
    for i in range(data.shape[0]):
        north, east, alt, d_north, d_east, d_alt = data[i, :]
        
        obstacle = [north - d_north-north_min_center - SAFETY_DISTANCE,
                    north + d_north-north_min_center + SAFETY_DISTANCE,
                    east - d_east-east_min_center - SAFETY_DISTANCE,
                    east + d_east-east_min_center + SAFETY_DISTANCE]
        corners = [(obstacle[0], obstacle[2]), (obstacle[0], obstacle[3]), (obstacle[1], obstacle[3]), (obstacle[1], obstacle[2])]
        
        # TODO: Compute the height of the polygon
        height = alt + d_alt + SAFETY_DISTANCE

        p = Poly(corners, height)
        polygons.append(p)

    return polygons

    
class Sampler:

    def __init__(self, data):
        self._polygons = extract_polygons(data)
        
        north_min_center = np.floor(np.min(data[:, 0] - data[:, 3]))
        east_min_center = np.floor(np.min(data[:, 1] - data[:, 4]))
        
        self._xmin = np.floor(np.min(data[:, 0] - data[:, 3])-north_min_center)
        self._xmax = np.ceil(np.max(data[:, 0] + data[:, 3])-north_min_center)

        self._ymin = np.floor(np.min(data[:, 1] - data[:, 4])-east_min_center)
        self._ymax = np.ceil(np.max(data[:, 1] + data[:, 4])-east_min_center)
        
       

        self._zmin = 3
        # limit z-axis
        self._zmax = 20

        centers = np.array([p.center for p in self._polygons])
        self._tree = KDTree(centers, metric='euclidean')

    def sample(self, num_samples):
        """Implemented with a k-d tree for efficiency."""
        xvals = np.random.uniform(self._xmin, self._xmax, num_samples)
        yvals = np.random.uniform(self._ymin, self._ymax, num_samples)
        zvals = np.random.uniform(self._zmin, self._zmax, num_samples)
        
        xvals [xvals < 0] = 0
        yvals [yvals < 0] = 0
        
        samples = list(zip(xvals, yvals, zvals))

        pts = []
        for s in samples:
            _, idx = self._tree.query(np.array([s[0], s[1]]).reshape(1, -1))
            p = self._polygons[int(idx)]
            if (not p.contains(s)) or p.height < s[2]:
                pts.append(s)
        return pts

    @property
    def polygons(self):
        return self._polygons
