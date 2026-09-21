# SafeCar 환경 설정 — 다른 스크립트가 source 해서 쓴다.
# 직접 쓸 때:  source ~/safecar_env.sh
source /opt/ros/jazzy/setup.bash
source /home/pi/camera_ws/install/local_setup.bash    # camera_ros
source /home/pi/jazzy_ws/install/local_setup.bash     # cv_bridge
source /home/pi/2026_CDP/install/local_setup.bash     # safecar
export ROS_DOMAIN_ID=52
