import sys

ibis_directory = "/home/pi/ARLISS_IBIS/IB/Library"
sys.path.append(ibis_directory)

from camera import Camera

def run():
    camera = Camera()
    camera.take_pic()
    res = camera.detect_center()
    print('percent={}, center={}'.format(res['percent'], res['center']))

    distance = 1
    x_m, y_m = camera.get_target_position(distance)
    print(f"x={x_m},y={y_m}")

run()
