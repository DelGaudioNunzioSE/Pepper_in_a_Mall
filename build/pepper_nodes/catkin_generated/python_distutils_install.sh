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

echo_and_run cd "/home/sirc/Scrivania/project_ws/src/cogrob_pepper_nodes/src/pepper_nodes"

# ensure that Python install destination exists
echo_and_run mkdir -p "$DESTDIR/home/sirc/Scrivania/project_ws/install/lib/python3/dist-packages"

# Note that PYTHONPATH is pulled from the environment to support installing
# into one location when some dependencies were installed in another
# location, #123.
echo_and_run /usr/bin/env \
    PYTHONPATH="/home/sirc/Scrivania/project_ws/install/lib/python3/dist-packages:/home/sirc/Scrivania/project_ws/build/pepper_nodes/lib/python3/dist-packages:$PYTHONPATH" \
    CATKIN_BINARY_DIR="/home/sirc/Scrivania/project_ws/build/pepper_nodes" \
    "/usr/bin/python3" \
    "/home/sirc/Scrivania/project_ws/src/cogrob_pepper_nodes/src/pepper_nodes/setup.py" \
     \
    build --build-base "/home/sirc/Scrivania/project_ws/build/pepper_nodes" \
    install \
    --root="${DESTDIR-/}" \
    --install-layout=deb --prefix="/home/sirc/Scrivania/project_ws/install" --install-scripts="/home/sirc/Scrivania/project_ws/install/bin"
