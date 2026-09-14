"""Local, continuous random walkers; no creation or removal within sensor reach."""
from dataclasses import dataclass
import math
import random
import xml.etree.ElementTree as ET

from shapely.geometry import Point, LineString, Polygon
from shapely.ops import unary_union
from shapely.prepared import prep


@dataclass
class Walker:
    name: str
    x: float
    y: float
    z: float
    heading: float
    speed: float


class Ground:
    def __init__(self, world):
        self.grid = world['grid']
        shapes = [Polygon(o['poly']).buffer(.3) for o in world.get('obstacles', [])
                  if o.get('poly')]
        shapes += [Point(o['x'], o['y']).buffer(o['r']+.3)
                   for o in world.get('obstacles', []) if o.get('type') == 'circle']
        shapes += [LineString(o['pts']).buffer(o.get('r', 0)+.3)
                   for o in world.get('obstacles', []) if o.get('type') == 'segment']
        shapes += [Point(t['x'], t['y']).buffer(t.get('trunkRadius', .3)+.3)
                   for t in world.get('trees', [])]
        self.blocked = prep(unary_union(shapes))

    def height(self, x, y):
        g = self.grid
        u, v = (x-g['ox'])/g['res'], (y-g['oy'])/g['res']
        if not (0 <= u < g['w']-1 and 0 <= v < g['h']-1):
            return None
        i, j = int(u), int(v)
        a, b = u-i, v-j
        h00, h10 = g['height'][j*g['w']+i:j*g['w']+i+2]
        h01, h11 = g['height'][(j+1)*g['w']+i:(j+1)*g['w']+i+2]
        # Same triangle diagonal as export_world.cjs.
        return h00+a*(h10-h00)+b*(h11-h10) if a >= b else h00+b*(h01-h00)+a*(h11-h01)

    def valid(self, x, y, previous=None):
        z = self.height(x, y)
        if z is None or not math.isfinite(z):
            return False
        shape = Point(x, y) if previous is None else LineString([previous, (x, y)])
        if self.blocked.intersects(shape):
            return False
        if previous is not None:
            old = self.height(*previous)
            if old is None or abs(z-old) > .5*math.dist(previous, (x, y))+.01:
                return False
        return True


def sensor_reach(filename):
    root = ET.parse(filename)
    robot = root.find(".//model[@name='icart_mini']")
    ranges = []
    for link in robot.findall('link'):
        lp = [float(v) for v in link.findtext('pose', '0 0 0 0 0 0').split()]
        for sensor in link.findall('sensor'):
            maximum = sensor.findtext('lidar/range/max')
            if maximum is not None:
                sp = [float(v) for v in sensor.findtext('pose', '0 0 0 0 0 0').split()]
                ranges.append(float(maximum)+math.sqrt(sum(v*v for v in lp[:3]))
                              +math.sqrt(sum(v*v for v in sp[:3])))
    if not ranges:
        raise ValueError('icart_mini LiDAR range missing')
    return root.find('world').get('name'), max(ranges)


class Crowd:
    def __init__(self, ground, radius, density=.1, seed=42, limit=200):
        self.ground, self.radius, self.limit = ground, radius, limit
        self.random = random.Random(seed)
        self.walkers = {}
        self.serial = 0
        self.set_density(density)

    def set_density(self, density):
        if not math.isfinite(density) or density < 0:
            raise ValueError('density must be finite and nonnegative (people / 100 m²)')
        self.density = density

    @property
    def target(self):
        return min(self.limit, round(self.density*math.pi*(self.radius+3)**2/100))

    def step(self, robot, dt):
        """Return births and exits. Caller commits births after successful creation."""
        if not 0 < dt <= .25:
            return [], []
        exits = []
        for w in self.walkers.values():
            distance = math.dist((w.x, w.y), robot)
            if distance > self.radius+6 or (len(self.walkers)-len(exits) > self.target
                                           and distance > self.radius+1):
                exits.append(w.name)
                continue
            w.heading += self.random.uniform(-.65, .65)*math.sqrt(dt)
            x = w.x+math.cos(w.heading)*w.speed*dt
            y = w.y+math.sin(w.heading)*w.speed*dt
            if (self.ground.valid(x, y, (w.x, w.y)) and math.dist((x, y), robot) > .8
                    and all(o is w or math.dist((x, y), (o.x, o.y)) > .65
                            for o in self.walkers.values())):
                w.x, w.y, w.z = x, y, self.ground.height(x, y)
            else:
                w.heading += self.random.choice([-1, 1])*math.pi/2
        births = []
        # Bounded effort even if the sensor boundary lies outside the map.
        for _ in range(min(1, max(0, self.target-len(self.walkers)))):
            angle = self.random.uniform(-math.pi, math.pi)
            r = self.radius+3
            x, y = robot[0]+r*math.cos(angle), robot[1]+r*math.sin(angle)
            if (not self.ground.valid(x, y) or any(math.dist((x, y), (w.x, w.y)) < .8
                    for w in [*self.walkers.values(), *births])):
                continue
            self.serial += 1
            births.append(Walker('pedestrian_'+str(self.serial), x, y, self.ground.height(x, y),
                                 angle+math.pi+self.random.uniform(-.6, .6),
                                 self.random.uniform(.6, 1.4)))
        return births, exits


def model_sdf(w):
    """Simple human silhouette with matching collision and LiDAR visual geometry."""
    sdf = ET.Element('sdf', version='1.9')
    model = ET.SubElement(sdf, 'model', name=w.name)
    ET.SubElement(model, 'static').text = 'true'
    ET.SubElement(model, 'pose').text = f'{w.x} {w.y} {w.z} 0 0 {w.heading}'
    link = ET.SubElement(model, 'link', name='body')
    for name, y, z, radius, length in [('torso', 0, 1.1, .23, .65),
                                      ('left_leg', .13, .4, .085, .8),
                                      ('right_leg', -.13, .4, .085, .8),
                                      ('head', 0, 1.58, .14, None)]:
        for kind in ('visual', 'collision'):
            part = ET.SubElement(link, kind, name=name)
            ET.SubElement(part, 'pose').text = f'0 {y} {z} 0 0 0'
            geom = ET.SubElement(ET.SubElement(part, 'geometry'),
                                 'sphere' if length is None else 'cylinder')
            ET.SubElement(geom, 'radius').text = str(radius)
            if length is not None:
                ET.SubElement(geom, 'length').text = str(length)
            if kind == 'visual':
                material = ET.SubElement(part, 'material')
                ET.SubElement(material, 'diffuse').text = '.2 .5 .85 1'
                ET.SubElement(material, 'ambient').text = '.2 .5 .85 1'
    return ET.tostring(sdf, encoding='unicode')
