#!/usr/bin/env bash
# 주행 준비: 차체 + 카메라 + 차선인식 + 안전게이트 + 영상 스트리머를 띄운다.
# 주행 자체는 시작하지 않는다 (safecar_drive.sh 로 시작).
source /home/pi/safecar_env.sh
cd /home/pi/runlog 2>/dev/null || { mkdir -p /home/pi/runlog; cd /home/pi/runlog; }

start() {  # start <로그이름> <명령...>
    local log="$1"; shift
    setsid nohup "$@" > "$log" 2>&1 < /dev/null &
}

start md.log  ros2 launch stella_md stella_md_launch.py
sleep 6
# 라이다: MRM 갓길 판정이 /scan의 후방·우측 섹터를 본다. 없으면 자차로정차로 폴백된다.
start lidar.log ros2 launch ydlidar ydlidar_launch.py
sleep 3
start cam.log ros2 run camera_ros camera_node --ros-args -p width:=640 -p height:=480 -p orientation:=180
sleep 8
start vd.log  ros2 run safecar_perception vision_detector_node
sleep 3
start dm.log  ros2 run safecar_control decision_maker_node
start img.log python3 /home/pi/image_server.py --topic /perception/lane_image
start imgraw.log python3 /home/pi/image_server.py --topic /camera/image_raw --port 8081
sleep 4

echo "=== 실행 상태 ==="
timeout 8 ros2 topic hz /perception/lane_offset 2>&1 | tail -1
echo "차선 오프셋: $(timeout 6 ros2 topic echo /perception/lane_offset --once --field data 2>/dev/null | head -1)"
echo "라이다: $(timeout 6 ros2 topic hz /scan 2>&1 | tail -1)"
echo
echo "영상:  http://raspberrypi.local:8080/   (차선 검출 결과)"
echo "       http://raspberrypi.local:8081/   (원본)"
echo "주행 시작:  ~/safecar_drive.sh"
echo "정지:       ~/safecar_stop.sh"
