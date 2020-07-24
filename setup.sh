# Source this file into your shell

# Find the ros dist we want to use.
for _d in melodic noetic
do
    [ -f /opt/ros/$_d/local_setup.sh ] && break
done

# Source the ROS setup file
. /opt/ros/$_d/local_setup.sh

# Make an alias 'py' for the right python version. This is important as
# the #! lines will be wrong when the version we use changes.
alias py=python$ROS_PYTHON_VERSION

# Check if we need to mock up the Pi stuff.
if ! py -c 'import gpiozero' 2>/dev/null
then
    PYTHONPATH="$PYTHONPATH:$(pwd)/mock"
fi
