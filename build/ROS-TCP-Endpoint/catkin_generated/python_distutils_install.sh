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

echo_and_run cd "/home/administrator/ws_daniel/src/ROS-TCP-Endpoint"

# ensure that Python install destination exists
echo_and_run mkdir -p "$DESTDIR/home/administrator/ws_daniel/install/lib/python3/dist-packages"

# Note that PYTHONPATH is pulled from the environment to support installing
# into one location when some dependencies were installed in another
# location, #123.
echo_and_run /usr/bin/env \
    PYTHONPATH="/home/administrator/ws_daniel/install/lib/python3/dist-packages:/home/administrator/ws_daniel/build/lib/python3/dist-packages:$PYTHONPATH" \
    CATKIN_BINARY_DIR="/home/administrator/ws_daniel/build" \
    "/usr/bin/python3" \
    "/home/administrator/ws_daniel/src/ROS-TCP-Endpoint/setup.py" \
    egg_info --egg-base /home/administrator/ws_daniel/build/ROS-TCP-Endpoint \
    build --build-base "/home/administrator/ws_daniel/build/ROS-TCP-Endpoint" \
    install \
    --root="${DESTDIR-/}" \
    --install-layout=deb --prefix="/home/administrator/ws_daniel/install" --install-scripts="/home/administrator/ws_daniel/install/bin"
