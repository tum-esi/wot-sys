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

echo_and_run cd "/home/bowen/uarm_workspace/src/swiftpro"

# ensure that Python install destination exists
echo_and_run mkdir -p "$DESTDIR/home/bowen/uarm_workspace/install/lib/python3/dist-packages"

# Note that PYTHONPATH is pulled from the environment to support installing
# into one location when some dependencies were installed in another
# location, #123.
echo_and_run /usr/bin/env \
    PYTHONPATH="/home/bowen/uarm_workspace/install/lib/python3/dist-packages:/home/bowen/uarm_workspace/build/lib/python3/dist-packages:$PYTHONPATH" \
    CATKIN_BINARY_DIR="/home/bowen/uarm_workspace/build" \
    "/usr/bin/python" \
    "/home/bowen/uarm_workspace/src/swiftpro/setup.py" \
    build --build-base "/home/bowen/uarm_workspace/build/swiftpro" \
    install \
    $DESTDIR_ARG \
    --install-layout=deb --prefix="/home/bowen/uarm_workspace/install" --install-scripts="/home/bowen/uarm_workspace/install/bin"
