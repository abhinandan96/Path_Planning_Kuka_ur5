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
    DESTDIR_ARG="--root=$DESTDIR"
fi

echo_and_run() { echo "+ $@" ; "$@" ; }

echo_and_run cd "/home/abhinandan/Misc/Intro to Robotics/ros_wkspace_asgn4/src/assignment4/urdf_parser_py"

# ensure that Python install destination exists
echo_and_run mkdir -p "$DESTDIR/home/abhinandan/Misc/Intro to Robotics/ros_wkspace_asgn4/install/lib/python2.7/dist-packages"

# Note that PYTHONPATH is pulled from the environment to support installing
# into one location when some dependencies were installed in another
# location, #123.
echo_and_run /usr/bin/env \
    PYTHONPATH="/home/abhinandan/Misc/Intro to Robotics/ros_wkspace_asgn4/install/lib/python2.7/dist-packages:/home/abhinandan/Misc/Intro to Robotics/ros_wkspace_asgn4/build/lib/python2.7/dist-packages:$PYTHONPATH" \
    CATKIN_BINARY_DIR="/home/abhinandan/Misc/Intro to Robotics/ros_wkspace_asgn4/build" \
    "/usr/bin/python" \
    "/home/abhinandan/Misc/Intro to Robotics/ros_wkspace_asgn4/src/assignment4/urdf_parser_py/setup.py" \
    build --build-base "/home/abhinandan/Misc/Intro to Robotics/ros_wkspace_asgn4/build/assignment4/urdf_parser_py" \
    install \
    $DESTDIR_ARG \
    --install-layout=deb --prefix="/home/abhinandan/Misc/Intro to Robotics/ros_wkspace_asgn4/install" --install-scripts="/home/abhinandan/Misc/Intro to Robotics/ros_wkspace_asgn4/install/bin"
