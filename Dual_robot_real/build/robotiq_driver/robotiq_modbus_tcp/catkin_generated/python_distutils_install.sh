#!/bin/sh

if [ -n "$DESTDIR" ] ; then
    case $DESTDIR in
        /*) # ok
            ;;
        *)
            /bin/echo "DESTDIR argument must be absolute... "
            /bin/echo "otherwise python's distutils will bork things."
            exit 1
    esac
fi

echo_and_run() { echo "+ $@" ; "$@" ; }

echo_and_run cd "/media/jt/Extreme SSD/Github/Dual_robot_real/src/robotiq_driver/robotiq_modbus_tcp"

# ensure that Python install destination exists
echo_and_run mkdir -p "$DESTDIR/media/jt/Extreme SSD/Github/Dual_robot_real/install/lib/python3/dist-packages"

# Note that PYTHONPATH is pulled from the environment to support installing
# into one location when some dependencies were installed in another
# location, #123.
echo_and_run /usr/bin/env \
    PYTHONPATH="/media/jt/Extreme SSD/Github/Dual_robot_real/install/lib/python3/dist-packages:/media/jt/Extreme SSD/Github/Dual_robot_real/build/lib/python3/dist-packages:$PYTHONPATH" \
    CATKIN_BINARY_DIR="/media/jt/Extreme SSD/Github/Dual_robot_real/build" \
    "/usr/bin/python3" \
    "/media/jt/Extreme SSD/Github/Dual_robot_real/src/robotiq_driver/robotiq_modbus_tcp/setup.py" \
     \
    build --build-base "/media/jt/Extreme SSD/Github/Dual_robot_real/build/robotiq_driver/robotiq_modbus_tcp" \
    install \
    --root="${DESTDIR-/}" \
    --install-layout=deb --prefix="/media/jt/Extreme SSD/Github/Dual_robot_real/install" --install-scripts="/media/jt/Extreme SSD/Github/Dual_robot_real/install/bin"
