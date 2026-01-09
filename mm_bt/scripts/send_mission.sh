#!/bin/bash

# ============================================
# SEND MISSION FROM YAML CONFIGURATION
# Auto-retry mechanism + YAML parameter loading
# Supports gamma_bt missions (includes FindBuoy and Move)
# ============================================

SELECTED_MISSION=${1:-goal_A}
MAX_RETRIES=20
RETRY_DELAY=2

echo "================================================================"
echo " 🚀 GAMMA_BT MISSION SENDER (AUTO-RETRY + YAML)"
echo "================================================================"
echo " Selected Mission: $SELECTED_MISSION"
echo "================================================================"

# ============================================
# WAIT FOR ROS PARAMETER SERVER
# ============================================
echo " ⏳ Waiting for ROS Parameter Server..."
sleep 5

# ============================================
# LOAD PARAMETERS FROM YAML
# ============================================
echo " 📋 Loading mission parameters from missions.yaml..."

MISSION_ID=$(rosparam get /$SELECTED_MISSION/mission_id 2>/dev/null)
MISSION_TAG=$(rosparam get /$SELECTED_MISSION/mission_tag 2>/dev/null)
LAT=$(rosparam get /$SELECTED_MISSION/latitude 2>/dev/null)
LON=$(rosparam get /$SELECTED_MISSION/longitude 2>/dev/null)
DEPTH=$(rosparam get /$SELECTED_MISSION/depth 2>/dev/null)
YAW=$(rosparam get /$SELECTED_MISSION/yaw 2>/dev/null)
RADIUS=$(rosparam get /$SELECTED_MISSION/radius 2>/dev/null)
USE_YAW=$(rosparam get /$SELECTED_MISSION/use_yaw 2>/dev/null)
USE_SPIRAL=$(rosparam get /$SELECTED_MISSION/use_spiral 2>/dev/null)
COLOR=$(rosparam get /$SELECTED_MISSION/color 2>/dev/null)

# Set default values if not specified
COLOR=${COLOR:-""}

# Check if parameters were loaded successfully
if [ -z "$MISSION_ID" ]; then
    echo " ❌ ERROR: Could not load mission '$SELECTED_MISSION' from parameter server!"
    echo " Available missions: goal_A, goal_B, survey_A, survey_B, find_buoy_yellow, move_north"
    exit 1
fi

echo " ✅ Parameters loaded successfully!"

# ============================================
# WAIT FOR GAMMA_BT TO SUBSCRIBE
# ============================================
echo " ⏳ Waiting for gamma_bt to subscribe to /gamma_bt/setInfo..."

for i in $(seq 1 $MAX_RETRIES); do
    SUBSCRIBERS=$(rostopic info /gamma_bt/setInfo 2>/dev/null | grep -c "Subscribers:")
    
    if [ "$SUBSCRIBERS" -gt 0 ]; then
        echo " ✅ gamma_bt is subscribed! (attempt $i/$MAX_RETRIES)"
        break
    fi
    
    if [ $i -eq $MAX_RETRIES ]; then
        echo " ❌ ERROR: gamma_bt did not subscribe after $MAX_RETRIES attempts!"
        exit 1
    fi
    
    echo " ⏳ Attempt $i/$MAX_RETRIES - waiting ${RETRY_DELAY}s..."
    sleep $RETRY_DELAY
done

sleep 1  # Safety delay

# ============================================
# SEND MISSION
# ============================================
echo ""
echo "📡 Sending mission '$SELECTED_MISSION'..."
echo "----------------------------------------------------------------"
echo " Mission ID: $MISSION_ID"
echo " Mission Tag: $MISSION_TAG"
echo " Position: $LAT°, $LON°, ${DEPTH}m"
echo " Mission params: Yaw=$YAW rad, Radius=${RADIUS}m"
echo " Flags: use_yaw=$USE_YAW, use_spiral=$USE_SPIRAL"

# Display color only for FindBuoy missions (tag 5 or 6)
if [ "$MISSION_TAG" -eq 5 ] || [ "$MISSION_TAG" -eq 6 ]; then
    echo " Color: $COLOR"
fi

echo "----------------------------------------------------------------"

# Build the message based on mission type
if [ "$MISSION_TAG" -eq 5 ] || [ "$MISSION_TAG" -eq 6 ]; then
    # Mission with color field (FindBuoy or Move)
    rostopic pub -1 /gamma_bt/setInfo rami_msgs/SetInfoMissionManager \
    "mission_id: $MISSION_ID
mission_tag: $MISSION_TAG
latitude: $LAT
longitude: $LON
depth: $DEPTH
yaw: $YAW
radius: $RADIUS
use_yaw: $USE_YAW
use_spiral: $USE_SPIRAL
color: '$COLOR'"
else
    # Standard mission without color
    rostopic pub -1 /gamma_bt/setInfo rami_msgs/SetInfoMissionManager \
    "mission_id: $MISSION_ID
mission_tag: $MISSION_TAG
latitude: $LAT
longitude: $LON
depth: $DEPTH
yaw: $YAW
radius: $RADIUS
use_yaw: $USE_YAW
use_spiral: $USE_SPIRAL
color: ''"
fi

if [ $? -eq 0 ]; then
    echo ""
    echo "✅ Mission '$SELECTED_MISSION' sent successfully!"
    echo "================================================================"
else
    echo ""
    echo "❌ ERROR: Failed to send mission '$SELECTED_MISSION'!"
    exit 1
fi



