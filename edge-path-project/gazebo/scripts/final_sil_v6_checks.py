"""Gazebo-independent validation primitives.

Clearance is conservative PLANAR footprint separation, not a 3-D terrain
collision oracle. Full vehicle bounding box includes turned wheels and sensors.
Positive infinity is a valid no-return ray; NaN, -inf and out-of-range finite
values are invalid. At most 5% invalid rays are allowed.
"""
import math

HALF_LENGTH = 1.35
HALF_WIDTH = 1.05
HEIGHT = 1.50


def scan_valid(msg):
    ranges = msg.ranges
    if (len(ranges) < 360 or not math.isfinite(msg.angle_step)
            or msg.angle_step <= 0 or not math.isfinite(msg.angle_min)
            or not 0 < msg.range_min < msg.range_max
            or (len(ranges) - 1) * msg.angle_step < 2 * math.pi - .1):
        return False
    good = sum(v == math.inf or (math.isfinite(v) and msg.range_min <= v <= msg.range_max)
               for v in ranges)
    return good / len(ranges) >= .95


def point_clearance(distance, angle):
    """Shortest point-to-rectangle distance; zero for a point inside."""
    x, y = distance * math.cos(angle), distance * math.sin(angle)
    return math.hypot(max(abs(x) - HALF_LENGTH, 0), max(abs(y) - HALF_WIDTH, 0))


def hull(points):
    points = sorted(set(points))
    def cross(o, a, b):
        return (a[0]-o[0])*(b[1]-o[1])-(a[1]-o[1])*(b[0]-o[0])
    lower, upper = [], []
    for p in points:
        while len(lower) >= 2 and cross(lower[-2], lower[-1], p) <= 0:
            lower.pop()
        lower.append(p)
    for p in reversed(points):
        while len(upper) >= 2 and cross(upper[-2], upper[-1], p) <= 0:
            upper.pop()
        upper.append(p)
    return lower[:-1]+upper[:-1]


def footprint(pose, hx, hy, zmin=0, zmax=0):
    x, y, yaw, roll, pitch = pose
    cy, sy = math.cos(yaw), math.sin(yaw)
    cp, sp = math.cos(pitch), math.sin(pitch)
    cr, sr = math.cos(roll), math.sin(roll)
    return hull([(x+cy*cp*a+(cy*sp*sr-sy*cr)*b+(cy*sp*cr+sy*sr)*c,
                  y+sy*cp*a+(sy*sp*sr+cy*cr)*b+(sy*sp*cr-cy*sr)*c)
                 for a in (-hx,hx) for b in (-hy,hy) for c in (zmin,zmax)])


def polygon_distance(a, b):
    """Exact 2-D distance of two convex polygons, with SAT overlap test."""
    separated = False
    for poly in (a,b):
        for i,p in enumerate(poly):
            q = poly[(i+1)%len(poly)]
            nx,ny = -(q[1]-p[1]), q[0]-p[0]
            aa=[x*nx+y*ny for x,y in a]; bb=[x*nx+y*ny for x,y in b]
            if max(aa)<min(bb) or max(bb)<min(aa):
                separated=True
    if not separated:
        return 0.0
    def point_segment(p,u,v):
        dx,dy=v[0]-u[0],v[1]-u[1]
        den=dx*dx+dy*dy
        t=max(0,min(1,((p[0]-u[0])*dx+(p[1]-u[1])*dy)/den)) if den else 0
        return math.hypot(p[0]-u[0]-t*dx,p[1]-u[1]-t*dy)
    return min(point_segment(p, poly[i], poly[(i+1)%len(poly)])
               for points,poly in ((a,b),(b,a)) for p in points for i in range(len(poly)))


def scene_clearance_detail(poses):
    names=('aeb_vehicle_360','aeb_vehicle_360_b','final_static_obstacle','final_moving_obstacle')
    found={}
    for key,value in poses.items():
        for name in names:
            if key==name or key.endswith('::'+name):
                found[name]=value
    if len(found)!=4 or any(not all(math.isfinite(v) for v in p) for p in found.values()):
        return None
    a,b=[footprint(found[n],HALF_LENGTH,HALF_WIDTH,0,HEIGHT) for n in names[:2]]
    obstacles=[footprint(found[n],h,h,-h,h) for n,h in zip(names[2:],(.8,.7))]
    candidates=[(polygon_distance(a,b), names[0]+' / '+names[1])]
    candidates += [(polygon_distance(v,o), names[i]+' / '+names[j+2])
                   for i,v in enumerate((a,b)) for j,o in enumerate(obstacles)]
    return min(candidates, key=lambda item:item[0])


def scene_clearance(poses):
    detail=scene_clearance_detail(poses)
    return None if detail is None else detail[0]


def margins_ok(a_lidar, b_lidar, ground):
    return (math.isfinite(ground) and ground>=1.0
            and a_lidar>=1.0 and b_lidar>=1.0)


def starting_scene_issue(poses):
    """Refuse to treat a partially completed world as a fresh scenario."""
    expected={
        'aeb_vehicle_360':(20.,40.,math.atan2(-2.,-4.)),
        'aeb_vehicle_360_b':(20.,24.,math.pi),
        'final_static_obstacle':(8.,34.,0.),
        'final_moving_obstacle':(8.,20.,math.pi/2),
    }
    for name,(x,y,yaw) in expected.items():
        p=next((v for k,v in poses.items() if k==name or k.endswith('::'+name)),None)
        if p is None:
            return f'missing initial pose: {name}'
        if not all(math.isfinite(v) for v in p):
            return f'invalid initial pose: {name}'
        tolerance=.6 if name.startswith('aeb_vehicle') else .2
        if math.hypot(p[0]-x,p[1]-y)>tolerance:
            return f'{name} is away from its start; close and reopen the world'
        if name.startswith('aeb_vehicle'):
            error=math.atan2(math.sin(p[2]-yaw),math.cos(p[2]-yaw))
            if abs(error)>.25:
                return f'{name} heading is not at its start; close and reopen the world'
    return None


def loading_approach_clearance(a_pose, approach, goal):
    """Conservative reservation for B's final approach, in world XY.

    The radius bounds the full 3-D vehicle box projected at any orientation.
    An additional 0.30 m allowance covers lateral path-tracking error.
    This is coordinator geometry, not a measurement of B's actual clearance.
    """
    radius=math.sqrt(HALF_LENGTH**2+HALF_WIDTH**2+HEIGHT**2)+.30
    x0=min(approach[0],goal[0])-radius
    x1=max(approach[0],goal[0])+radius
    y0=min(approach[1],goal[1])-radius
    y1=max(approach[1],goal[1])+radius
    reservation=[(x0,y0),(x1,y0),(x1,y1),(x0,y1)]
    a=footprint(a_pose,HALF_LENGTH,HALF_WIDTH,0,HEIGHT)
    return polygon_distance(a,reservation)
