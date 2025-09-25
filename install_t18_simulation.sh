#!/bin/bash

# install scripts for the t18 simulation environment


# ================= PX4_MSGS SETUP =================
ROS2_WS=~/fsc_octorotor_simulation_ros2/ros2_node
PX4_MSGS_DIR=$ROS2_WS/src/px4_msgs

# create src folder if missing
mkdir -p "$ROS2_WS/src"

if [ -d "$PX4_MSGS_DIR" ]; then
    echo "px4_msgs already exists, resetting to correct version..."
    cd "$PX4_MSGS_DIR" || exit
    git fetch origin
    git checkout release/1.15
    git reset --hard a1045ec
    echo "Reset px4_msgs to the correct version successfully!"
else
    echo "Cloning px4_msgs repository..."
    git clone https://github.com/PX4/px4_msgs.git "$PX4_MSGS_DIR"
    cd "$PX4_MSGS_DIR" || exit
    git checkout release/1.15
    git reset --hard a1045ec
    echo "Cloned px4_msgs successfully!"
fi



# ================= PX4 AIRFRAME SETUP =================
SRC_AIRFRAME_DIR=~/fsc_octorotor_simulation_ros2/px4_airframe_configuration/4012_gz_t18 # source directory
DEST_AIRFRAME_DIR=~/PX4-Autopilot/ROMFS/px4fmu_common/init.d-posix/airframes/ # destination directory
DEST_AIRFRAME_FILE="$DEST_AIRFRAME_DIR/4012_gz_t18"

# overwrite existing airframe and copy if it doesn't exist
if [ -f "$DEST_AIRFRAME_FILE" ]; then
    echo "4012_gz_t18 airframe already exists, updating..."
    if rm -f "$DEST_AIRFRAME_FILE" && cp "$SRC_AIRFRAME_DIR" "$DEST_AIRFRAME_DIR"; then
        echo "Updated 4012_gz_t18 airframe successfully!"
    else
        echo "Failed to update 4012_gz_t18 airframe."
    fi
else
    if cp "$SRC_AIRFRAME_DIR" "$DEST_AIRFRAME_DIR"; then
        echo "Installed 4012_gz_t18 airframe successfully!"
    else
        echo "Failed to install 4012_gz_t18 airframe."
    fi
fi

# add airframe to CMakeLists.txt if not already present
AIRFRAME_CMAKE="$DEST_AIRFRAME_DIR/CMakeLists.txt"
if grep -qF '4012_gz_t18' "$AIRFRAME_CMAKE"; then
    echo "4012_gz_t18 already exists in CMakeLists.txt, skipping addition."
else
    if sed -i '/^)/i\    4012_gz_t18' "$AIRFRAME_CMAKE"; then
        echo "Added 4012_gz_t18 to CMakeLists.txt successfully!"
    else
        echo "Failed to add 4012_gz_t18 to CMakeLists.txt."
    fi
fi


# ================= GAZEBO MODEL SETUP =================
SRC_MODELS_DIR=~/fsc_octorotor_simulation_ros2/gazebo/gazebo_model # source directory
DEST_MODELS_DIR=~/PX4-Autopilot/Tools/simulation/gz/models # destination directory

# create destination directory if it doesn't exist
mkdir -p "$DEST_MODELS_DIR"

# copy t18, t18_base, t18_mono_cam, and Pine Tree models
for model in t18 t18_base t18_mono_cam "Pine Tree"; do
    src="$SRC_MODELS_DIR/$model"
    dest="$DEST_MODELS_DIR/$model"

    # overwrite existing model and copy if it doesn't exist
    if [ -d "$dest" ]; then
        echo "Gazebo model '$model' already exists, updating..."
        if rm -rf "$dest" && cp -r "$src" "$dest"; then
            echo "Updated Gazebo model: $model successfully!"
        else
            echo "Failed to update Gazebo model: $model."
        fi
    else
        if cp -r "$src" "$dest"; then
            echo "Installed Gazebo model: $model successfully!"
        else
            echo "Failed to install Gazebo model: $model."
        fi
    fi
done


# ================= GAZEBO WORLD SETUP =================
SRC_WORLDS_DIR=~/fsc_octorotor_simulation_ros2/gazebo/gazebo_world # source directory
DEST_WORLDS_DIR=~/PX4-Autopilot/Tools/simulation/gz/worlds # destination directory

# create destination directory if it doesn't exist
mkdir -p "$DEST_WORLDS_DIR"

src="$SRC_WORLDS_DIR/customforest.sdf"
dest="$DEST_WORLDS_DIR/customforest.sdf"

# overwrite existing world and copy if it doesn't exist
if [ -f "$dest" ]; then
    echo "Gazebo world 'customforest.sdf' already exists, updating..."
    if rm -f "$dest" && cp "$src" "$dest"; then
        echo "Updated Gazebo world successfully!"
    else
        echo "Failed to update Gazebo world."
    fi
else
    if cp "$src" "$dest"; then
        echo "Installed Gazebo world successfully!"
    else
        echo "Failed to install Gazebo world."
    fi
fi

# add customforest to CMakeLists.txt if not already present
CMAKELISTS=~/PX4-Autopilot/src/modules/simulation/gz_bridge/CMakeLists.txt
if grep -qF 'customforest' "$CMAKELISTS"; then
    echo "customforest already exists in CMakeLists.txt, skipping addition."
else
    if sed -i '/set(gz_worlds/,/)/ s/)/    customforest\n)/' "$CMAKELISTS"; then
        echo "Added customforest world to CMakeLists.txt successfully!"
    else
        echo "Failed to add customforest world to CMakeLists.txt."
    fi
fi

echo "Installation script completed!"