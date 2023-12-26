import math

def distance(point1, point2):
    # uses the 'haversine' formula
    # https://www.movable-type.co.uk/scripts/latlong.html
    R = 6371e3; # metres
    f1 = point1.lat * math.pi/180; # f, l in radians
    f2 = point2.lat * math.pi/180;
    df = (point2.lat - point1.lat) * math.pi/180;
    dl = (point2.lon - point1.lon) * math.pi/180;

    a = math.sin(df/2) * math.sin(df/2) + math.cos(f1) * math.cos(f2) * math.sin(dl/2) * math.sin(dl/2);
    c = 2 * math.atan2(math.sqrt(a), math.sqrt(1 - a));

    return R * c; # in metres

def direction(point1, point2):
    #from initial bearing formula
    # https://www.movable-type.co.uk/scripts/latlong.html
    l1 = point1.lon * math.pi/180
    l2 = point2.lon * math.pi/180

    f1 = point1.lat * math.pi/180
    f2 = point2.lat * math.pi/180

    y = math.sin(l2-l1) * math.cos(f2);
    x = math.cos(f1)*math.sin(f2) - math.sin(f1)*math.cos(f2)*math.cos(l2 - l1);
    return x, y

def rotate(angle_rad, x, y, clockwise=False):
    sn = math.sin(angle_rad)
    cs = math.cos(angle_rad)

    if clockwise:
        rx = cs * x + sn * y
        ry = -sn * x + cs * y
    else:
        rx = cs * x - sn * y
        ry = sn * x + cs * y

    return rx, ry

def scale_axis(max, x, y):
    # scale normilized vector so bigger coord became max value big
    if x == 0.0 and y == 0.0:
        return 0.0, 0.0

    abs_x = abs(x)
    abs_y = abs(y)
    if abs_x >= abs_y:
        mult = max / abs_x
    else:
        mult = max / abs_y
    return x * mult, y * mult