#!/usr/bin/env python3
"""Repository-grounded architecture model. Archify specs and inspector facts share IDs."""
import hashlib, json, pathlib, re, subprocess, sys
ROOT=pathlib.Path(__file__).resolve().parents[5]
OUT=pathlib.Path(__file__).resolve().parents[1]
LOC='src/localization/'
LAYERS={}
def src(path,anchor=None):
 p=ROOT/path
 assert p.is_file(),path
 lines=p.read_text().splitlines()
 line=next((i+1 for i,s in enumerate(lines) if anchor in s),None) if anchor else 1
 assert line is not None,(path,anchor)
 return {'path':path,'line':line}
def n(id,label,sub,summary,paths=(),target=None,kind='backend',state='구현',io=()):
 return dict(id=id,label=label,sublabel=sub,summary=summary,sources=[src(*p) if isinstance(p,tuple) else src(p) for p in paths],target=target,type=kind,state=state,io=list(io))
def e(a,b,label,planned=False):return dict(id=a+'-to-'+b,**{'from':a,'to':b},label=label,variant='dashed' if planned else 'default')
def layer(id,title,parent,nodes,edges=(),note='',layout='snake',positions=None):
 LAYERS[id]=dict(id=id,title=title,parent=parent,nodes=nodes,edges=list(edges),note=note,layout=layout,positions=positions)
def local(path):return LOC+path
layer('system','전체 시스템',None,[
 n('sensors','센서 드라이버','IMU · GNSS · LiDAR · Camera','센서별 bringup과 Localization 내장 드라이버 경로를 함께 탐색한다. 실제 활성화는 launch 인자와 외부 실행 상태에 따라 달라진다.',['src/sensor_drivers/README.md',local('launch/sensors.launch')],'sensors',kind='external'),
 n('localization','Localization','위치 추정 · 초기화 · 상태 · 복구','mando_localization 패키지. 두 EKF, GPS 승인, RDDF 초기화, Supervisor와 Output Gate로 구성된다. 최종 공개 Odometry와 valid를 제공한다.',[local('launch/bringup.launch'),local('config/localization_interfaces.yaml')],'localization',io=['/molit/localization/odometry','/molit/localization/valid']),
 n('planning','경로 계획·선택','플래너 뼈대 · Selector 구현','두 플래너는 package.xml/CMakeLists.txt만 있다. 현재 selector는 /path/lidar를 /path/final로 전달하며 주차 경로 선택은 구현되어 있지 않다.',['src/selector/scripts/selector_node','src/lidar_path_planning/CMakeLists.txt','src/parking_path_planning/CMakeLists.txt'],'planning',state='부분 구현'),
 n('control','차량 제어','control · 실행 코드 없음','README는 /path/final과 /TL_label을 받아 DriveCmd를 계산하도록 설계한다. 현재 control에는 실행 노드가 없어 이 연결은 구현 완료 상태가 아니다.',['src/control/CMakeLists.txt',('README.md','### Nodes')],'control',state='설계 / 뼈대'),
 n('interfaces','인터페이스','메시지 정의 · 차량 연결','perception_interfaces, sensor_interfaces, erp42_msgs와 Arduino rosserial bringup의 계약을 확인한다.',['src/interfaces/README.md'],'interfaces',kind='messagebus'),
 n('perception','인지','객체 검출 · 신호등 인식','object_detection과 traffic_light는 패키지 뼈대다. 입출력은 README의 설계 계약이며 실행 중 ROS 연결을 의미하지 않는다.',['src/object_detection/CMakeLists.txt','src/traffic_light/CMakeLists.txt',('README.md','### Nodes')],'perception',state='설계 / 뼈대'),
 n('operations','실행·관측','launch · replay · Viewer','센서 실행 옵션, RDDF/지도 준비, replay 차이, 진단과 시각화를 확인한다.',[local('launch/bringup.launch'),local('launch/replay.launch')],'operations',kind='frontend')
],[e('sensors','localization','센서 입력'),e('localization','planning','위치 연결 미구현',True),e('planning','control','Path · 설계',True),e('control','interfaces','DriveCmd · 설계',True),e('sensors','perception','Scan/Image · 설계',True),e('perception','planning','객체 정보 · 설계',True)],
'실선은 코드로 확인한 흐름, 점선은 README 설계 또는 미구현 소비 경로다. /current_pos는 현재 Localization 공개 출력과 다르며 호환 브리지는 확인되지 않는다.',positions={'sensors':[60,300],'localization':[455,300],'planning':[850,300],'control':[850,560],'interfaces':[455,560],'perception':[455,60],'operations':[60,560]})
layer('localization','Localization 내부','system',[
 n('motion','IMU·엔코더 처리','보정 · 어댑터 · Local EKF','IMU normalized/calibrated와 엔코더 Twist를 만든다. InterfaceAdapter가 EKF 내부 토픽으로 전달한다. Local Odom은 GPS 측정 시각 정합의 이력으로도 사용된다.',[local('launch/local_fusion.launch')],'motion'),
 n('global','Global EKF','map 좌표의 추정 결과','움직임 입력과 승인 GPS pose를 융합한다. Local Odom을 관측으로 중복 융합하지 않는다. Global 출력은 Output Gate의 승인 전 추정값이다.',[local('config/ekf_global.yaml'),local('launch/global_fusion.launch')],'filters'),
 n('output','Output Gate','승인된 최종 Odometry','fresh valid와 초기화 준비, frame/time/pose/twist 품질 검사를 통과한 메시지만 공개한다.',[local('src/output_gate/localization_output_gate.cpp')],'output',kind='security'),
 n('gps','GPS 위치 승인','측정 시각 정합 · gate pose','NavSatFix를 측정 시각 Local Odom과 정합하고 datum·레버암·innovation·연속 정상 조건을 검사한다. Coordinator가 공개 GPS pose를 소유한다.',[local('src/odometry_gps_fusion/odometry_gps_fusion.cpp')],'gps'),
 n('safety','Supervisor 프로세스','Manager · Coordinator · 중재','StatusManager, RelocalizationCoordinator, Supervisor는 한 ROS 프로세스 안의 세 객체다. 최종 state/valid/status 공개 권한은 Supervisor에 있다.',[local('src/supervisor/localization_supervisor_node.cpp')],'safety',kind='security'),
 n('rddf','RDDF 초기화','GPS 자동 · 수동 선택 · 확인','RDDF 경로에서 시작 위치와 yaw를 고르고 IMU heading 및 두 EKF를 초기화한 뒤 출력 확인을 기다린다.',[local('scripts/rddf_initializer_node.py')],'rddf',kind='database'),
 n('timing','센서 시각','host · driver · 측정 timestamp','PC clock과 GPS driver 시각 정책, 센서 시간 품질을 관측하고 clock_ready를 발행한다.',[local('scripts/sensor_timing_monitor.py')],'timing'),
 n('operations','TF·실행·관측','프레임 소유권 · Viewer · replay','TF 소유자, 센서 실행 옵션과 Viewer/diagnostics 경로를 확인한다.',[local('launch/safety_and_tf.launch'),local('launch/visualization.launch')],'operations',kind='frontend')
],[e('motion','global','IMU · Twist'),e('global','output','Global Odom'),e('gps','global','GPS pose'),e('safety','output','valid')],
'위치는 위쪽 경로로 추정·공개된다. 아래쪽 초기화·시각·TF는 여러 노드에 적용되는 공통 조건이며 상세 화면에서 각 소비 경로를 확인한다.',positions={'motion':[60,60],'global':[455,60],'output':[850,60],'lidar':[60,325],'gps':[455,325],'safety':[850,325],'rddf':[60,590],'timing':[455,590],'operations':[850,590]})
layer('sensors','센서 드라이버','system',[
 n('imu','Xsens IMU','imu_bringup / 내장 driver','별도 imu_bringup은 raw IMU에 측정 covariance를 적용한다. Localization 내장 경로는 imu_driver.yaml을 사용한다. 둘의 설정과 토픽이 동일하다고 가정하지 않는다.',['src/sensor_drivers/imu/imu_bringup/launch/xsens_mti.launch','src/sensor_drivers/imu/xsens_ros_mti_driver/param/xsens_mti_node.yaml',local('config/imu_driver.yaml')],target='imu'),
 n('gps','u-blox GNSS','Fix · NavPVT · timing','외부 gps_bringup의 RTK/NTRIP 경로와 Localization 내부 driver를 구분한다. 내부 driver는 check_time_sync.py를 launch-prefix로 사용한다.',[local('launch/sensors.launch'),local('config/gps_driver.yaml'),'src/sensor_drivers/gps/gps_bringup/launch/gps.launch'],target='timing'),
 n('lidar','RPLIDAR S2','LaserScan · frame remap','외부 rplidar_s2.launch 기본 /scan과 laser 프레임은 Localization 계약 /molit/sensors/lidar/scan 및 laser_link와 연결 설정을 확인해야 한다.',['src/sensor_drivers/lidar/launch/rplidar_s2.launch',local('config/lidar_driver.yaml')],target='lidar'),
 n('camera','USB Camera','usb_cam · Image','카메라 bringup과 장치 설정은 존재한다. 영상의 신호등 소비 노드는 현재 뼈대다.',['src/sensor_drivers/cam/cam_bringup/launch/cam.launch','src/sensor_drivers/cam/README.md'],target='perception'),
 n('encoder','ERP42 feedback','rosserial · SerialFeedBack','Arduino firmware가 발행하는 /erp42_serial/feedback을 소비한다. Header가 없어 adapter는 PC 수신 시각을 부여한다.',[local('launch/sensors.launch'),local('config/encoder_driver.yaml'),'src/interfaces/vehicle_interface/erp42_msgs/msg/SerialFeedBack.msg'],target='encoder')
],note='센서별 독립 실행 경로다. 연결선이 없는 카드는 서로 직렬 처리 단계라는 뜻이 아니다.')
layer('perception','인지: 설계와 구현 범위','system',[
 n('image','카메라 영상','sensor_msgs/Image','외부 camera bringup의 원본 영상 입력.',['src/sensor_drivers/cam/cam_bringup/launch/cam.launch'],kind='external'),
 n('traffic','신호등 인식','traffic_light · 뼈대','README의 신호등 인식 노드 설계. 현재 실행 파일과 추론 모델 연결이 없다.',['src/traffic_light/CMakeLists.txt','src/traffic_light/package.xml'],state='설계 / 뼈대'),
 n('label','신호등 결과','/TL_label · TLLabel','메시지 정의는 구현되어 있지만 이 저장소에 발행 노드는 없다.',['src/interfaces/perception_interfaces/msg/TLLabel.msg'],kind='messagebus',state='메시지 정의'),
 n('scan','LiDAR 거리 스캔','sensor_msgs/LaserScan','외부 LiDAR driver가 제공할 수 있는 입력.',['src/sensor_drivers/lidar/launch/rplidar_s2.launch'],kind='external'),
 n('objects','객체 검출','object_detection · 뼈대','README의 2D LiDAR 객체 검출 설계. 현재 검출 실행 코드가 없다.',['src/object_detection/CMakeLists.txt','src/object_detection/package.xml'],state='설계 / 뼈대'),
 n('info','장애물 정보','/object_info · ObjectInfo','Header와 장애물 수·중심·크기 등 메시지 필드를 소스에서 확인한다. 발행자는 미구현이다.',['src/interfaces/perception_interfaces/msg/ObjectInfo.msg'],kind='messagebus',state='메시지 정의')
],[e('image','traffic','Image · 설계',True),e('traffic','label','인식 결과 · 설계',True),e('scan','objects','Scan · 설계',True),e('objects','info','객체 목록 · 설계',True)],layout='rows',note='점선은 README 설계다. 설치 가능한 패키지 뼈대와 실제 실행 노드를 구분한다.')
layer('planning','경로 계획·Selector','system',[
 n('inputs','설계 입력','객체 정보 · 현재 위치','README는 /object_info와 /current_pos를 계획 입력으로 둔다. 현 Localization 출력으로의 변환/연결 구현이 없다.',[('README.md','### Nodes'),local('config/localization_interfaces.yaml')],kind='external',state='설계'),
 n('lidar_planner','LiDAR 경로 계획','lidar_path_planning · 뼈대','장애물 회피 경로 /path/lidar 생성은 설계만 존재한다.',['src/lidar_path_planning/CMakeLists.txt','src/lidar_path_planning/package.xml'],state='설계 / 뼈대'),
 n('selector','Selector','/path/lidar → /path/final','현재 Python 노드는 Path를 그대로 전달한다. 위치·주차·상태 토픽 구독이나 주행 모드 분기는 없다.',['src/selector/scripts/selector_node'],io=['/path/lidar · nav_msgs/Path','/path/final · nav_msgs/Path']),
 n('final','최종 경로','nav_msgs/Path','Selector의 실제 발행 토픽이다. 이 토픽을 소비하는 control 실행 노드는 저장소에서 확인되지 않는다.',['src/selector/scripts/selector_node'],kind='messagebus'),
 n('parking','주차 경로 계획','parking_path_planning · 뼈대','README에 /path/park 출력과 Selector 연결이 설계되어 있으나 실제 생성·선택 코드는 없다.',['src/parking_path_planning/CMakeLists.txt','src/parking_path_planning/package.xml'],state='설계 / 뼈대')
],[e('inputs','lidar_planner','위치·객체 · 설계',True),e('lidar_planner','selector','/path/lidar · 설계',True),e('selector','final','/path/final'),e('parking','selector','주차 선택 미구현',True)],note='Selector의 relay는 실제 구현이며, 두 플래너와 주차 선택은 미구현이다.')
layer('control','차량 제어: 설계 계약','system',[
 n('path','경로·신호등 입력','/path/final · /TL_label','README가 지정한 제어 입력. Path relay만 구현되어 있고 TLLabel 발행 노드는 없다.',[('README.md','### Nodes'),'src/selector/scripts/selector_node'],kind='external',state='설계'),
 n('controller','제어 계산','control · 실행 노드 없음','경로 추종과 속도·조향·제동을 계산하도록 설계되었다. CMake에는 catkin_package만 있으며 알고리즘·노드·안전 연동 구현은 없다.',['src/control/CMakeLists.txt','src/control/package.xml'],state='설계 / 뼈대'),
 n('drive','차량 명령','/pure_pursuit/raw_drive','erp42_msgs/DriveCmd는 메시지 정의이며, 해당 명령의 발행 실행 코드는 현재 control에 없다.',['src/interfaces/vehicle_interface/erp42_msgs/msg/DriveCmd.msg',('README.md','DRIVE_CMD[')],kind='messagebus',state='설계 계약'),
 n('vehicle','차량 인터페이스','ERP42 메시지 · Arduino','Arduino rosserial bringup과 메시지 정의가 있다. DriveCmd를 실제 액추에이터 프로토콜로 변환하는 control 구현이 있다고 추정하지 않는다.',['src/interfaces/vehicle_interface/README.md','src/interfaces/vehicle_interface/vehicle_interface_bringup/launch/arduino.launch'],target='interfaces',kind='external')
],[e('path','controller','제어 입력 · 설계',True),e('controller','drive','DriveCmd · 설계',True),e('drive','vehicle','차량 전달 · 설계',True)],note='Localization valid=false는 위치 출력 승인 해제다. 차량 제동 동작을 수행한다는 뜻이 아니며 실제 제어 연동은 별도 구현 대상이다.')
layer('interfaces','메시지·ROS 계약','system',[
 n('localization_contract','Localization 계약','공개 · 내부 · 서비스 · 프레임','YAML의 topics/internal_topics/services/frames/message_types가 현재 노드의 계약이다. 아래 원문과 파라미터 표에서 전체 목록을 확인한다.',[local('config/localization_interfaces.yaml')],target='adapter',kind='messagebus'),
 n('perception_messages','인지 메시지','ObjectInfo · TLLabel','메시지 구조 자체는 존재한다. 생산자·소비자의 실행 여부와 별도다.',['src/interfaces/perception_interfaces/msg/ObjectInfo.msg','src/interfaces/perception_interfaces/msg/TLLabel.msg'],kind='messagebus'),
 n('gps_status','GNSS 상태','sensor_interfaces/GpsStatus','GNSS solution, 정확도, RTK 관련 상태 필드의 정의.',['src/interfaces/sensor_interfaces/msg/GpsStatus.msg'],kind='messagebus'),
 n('erp_messages','ERP42 메시지','피드백 · 명령 · 모드','SerialFeedBack, CANFeedBack, DriveCmd, CmdControl, ModeCmd 원문을 제공한다.',['src/interfaces/vehicle_interface/erp42_msgs/msg/'+f+'.msg' for f in ['SerialFeedBack','CANFeedBack','DriveCmd','CmdControl','ModeCmd']],kind='messagebus'),
 n('local_services','초기화·복구 계약','SetInitialHeading · Reanchor','heading 서비스와 GPS gate 재정합 요청/ACK 메시지. transaction 식별자로 오래된 응답을 구분한다.',[local('srv/SetInitialHeading.srv'),local('msg/GpsGateReanchor.msg')],kind='messagebus')
],note='이 화면은 계약 분류도다. 서로 다른 메시지 정의 사이의 데이터 전달 관계를 만들지 않는다.')
layer('motion','IMU·엔코더와 Local EKF','localization',[
 n('encoder','엔코더 변환','feedback → Twist','유효한 feedback을 검사하여 전진 속도와 vy=0 모델 제약을 발행한다.',[local('src/imu_encoder_fusion/encoder_to_twist_adapter.cpp'),local('config/encoder_calibration.yaml')],'encoder'),
 n('adapter','InterfaceAdapter','공개 ↔ 내부 토픽','공개 Twist/calibrated IMU와 승인 절대 pose를 EKF 입력으로 relay하고 내부 EKF 출력과 센서 입력을 공개 토픽으로 전달한다.',[local('src/common/localization_interface_adapter.cpp')],'adapter'),
 n('local','Local EKF','odom → base_link','Twist vx/vy와 IMU yaw/yaw-rate를 사용한다. 30Hz는 설정 목표이며 실측 처리율이 아니다.',[local('config/ekf_local.yaml'),local('launch/local_fusion.launch')],'filters'),
 n('raw_imu','IMU 정규화','raw → normalized','원본 IMU의 frame/time/quaternion/covariance 등을 검증한다. 의미와 조건은 imu_normalizer.cpp를 확인한다.',[local('src/imu_encoder_fusion/imu_normalizer.cpp'),local('config/imu_driver.yaml')],'imu'),
 n('calibrated','CalibratedIMU','RDDF yaw · 정지 hold · GNSS','초기화된 yaw 기준과 GNSS 직진 보정, encoder 정지 yaw hold를 적용한다. GNSS 보정 완료 여부와 시작 초기화 준비는 별개다.',[local('scripts/calibrated_imu_core.py'),local('scripts/calibrated_imu_node.py')],'imu')
],[e('encoder','adapter','Twist'),e('adapter','local','EKF 입력'),e('raw_imu','calibrated','normalized IMU'),e('calibrated','adapter','calibrated IMU')],layout='rows')
layer('adapter','InterfaceAdapter의 토픽 경계','motion',[
 n('public_motion','공개 움직임','Twist · calibrated IMU','encoder_twist와 imu_calibrated만 EKF 내부 움직임 입력으로 전달한다.',[local('src/common/localization_interface_adapter.cpp'),local('config/localization_interfaces.yaml')],kind='messagebus'),
 n('relay','검증·Relay','localization_interface_adapter','노드가 수행하는 변환·검증·복제 경로는 소스의 각 callback에서 확인한다.',[local('src/common/localization_interface_adapter.cpp'),local('src/common/localization_interface_adapter.hpp')]),
 n('private_ekf','EKF 내부 토픽','/mando_localization/internal/ekf','twist, imu, gps_pose 입력과 local/global_odometry 출력을 분리한다.',[local('config/ekf_local.yaml'),local('config/ekf_global.yaml'),local('src/common/localization_interface_adapter.cpp')],kind='messagebus'),
 n('absolute','승인 절대 pose','GPS map_pose','품질 후보를 직접 EKF에 넣지 않는다. Coordinator에서 공개한 승인 pose가 입력이다.',[local('config/localization_interfaces.yaml'),local('src/common/localization_interface_adapter.cpp')],kind='messagebus'),
 n('driver','내부 driver','공개 센서 · EKF 경계','내장 driver raw 입력 및 EKF 결과 relay 등 전체 구성은 소스의 subscribe/advertise 목록에서 확인한다.',[local('src/common/localization_interface_adapter.cpp')],kind='messagebus')
],[e('public_motion','relay','공개 입력'),e('relay','private_ekf','내부 입력'),e('absolute','relay','승인 pose')],layout='rows')
layer('filters','두 EKF와 관측 소유권','localization',[
 n('motion','공통 움직임 입력','vx · vy=0 · yaw · yaw-rate','두 EKF는 동일한 Twist와 calibrated IMU 입력을 사용한다.',[local('config/ekf_local.yaml'),local('config/ekf_global.yaml')],kind='messagebus'),
 n('local','Local EKF','world_frame: odom','odom 기준 추측 항법. odom → base_link TF 소유자. 출력은 GPS 시각 정합/예측 비교에도 사용된다.',[local('config/ekf_local.yaml')]),
 n('history','Local Odom 이력','GPS 원래 측정 시각의 pose','Global EKF에 Local Odom을 재융합하지 않는다. GPS 정합 노드가 이력을 사용한다.',[local('src/odometry_gps_fusion/local_odometry_history.hpp'),local('src/odometry_gps_fusion/odometry_gps_fusion.cpp')],target='gps'),
 n('global','Global EKF','world_frame: map','움직임 + GPS XY. map → odom TF 소유자. 과거 측정 재처리 이력 2초, corrected publication 금지.',[local('config/ekf_global.yaml')]),
 n('gps','승인 GPS','pose0 · XY','Global pose0_config는 GPS 위치 XY만 관측한다.',[local('config/ekf_global.yaml')],kind='messagebus'),
],[e('motion','local','움직임'),e('local','history','Local Odom'),e('motion','global','움직임'),e('gps','global','GPS XY')],note='두 필터의 설정 배열은 각 노드의 파라미터 표에서 원문을 확인한다.',positions={'motion':[60,60],'local':[455,60],'history':[850,60],'global':[60,380],'gps':[455,380],'lidar':[60,650]})
layer('lidar','LiDAR 스캔 수집·표시','sensors',[
 n('driver','RPLIDAR S2','원시 LaserScan','센서 드라이버가 원시 스캔을 발행한다.',[local('config/lidar_driver.yaml'),local('launch/map_data_collection.launch')],kind='external'),
 n('front','전방 스캔 표시','거리·각도 필터','전방 표시용 스캔을 별도 토픽으로 만든다.',[local('scripts/lidar_front_scan_visualizer.py'),local('config/lidar_front_visualization.yaml')]),
 n('viewer','공통 Viewer','센서 스캔 관측','차량 위치와 전방 라이다 스캔을 표시한다.',[local('scripts/localization_viewer.py')],target='viewer',kind='frontend'),
 n('record','원시 토픽 기록','rosbag','원시 스캔과 위치 추정 센서 데이터를 기록한다.',[local('launch/map_data_collection.launch'),local('scripts/localization_record_command.sh')],kind='database')
],[e('driver','front','LaserScan'),e('front','viewer','전방 스캔'),e('driver','record','원시 스캔')],positions={'driver':[60,70],'front':[455,70],'viewer':[850,70],'record':[60,380]})
layer('safety','Supervisor 프로세스 내부','localization',[
 n('manager','StatusManager','센서·관측·추정 건강도 평가','여러 센서의 freshness와 Global 일관성을 평가하고 내부 evaluated state/valid/status를 발행한다.',[local('src/status_manager/localization_status_manager.cpp'),local('src/status_manager/localization_state_evaluator.cpp')],'state',kind='security'),
 n('arbiter','Supervisor 중재','공개 상태의 단일 소유자','평가 heartbeat와 recovery heartbeat가 없거나 stale이면 FAULT/false. fresh recovery active가 평가 결과보다 우선한다.',[local('src/supervisor/localization_supervisor.cpp')],kind='security'),
 n('gate','Output Gate','별도 ROS 노드','공개 valid와 메시지 품질 및 초기화 준비를 검사한 최종 Odometry만 내보낸다.',[local('src/output_gate/localization_output_gate.cpp')],'output',kind='security'),
 n('recovery','Recovery Coordinator','장기 복구 · reset · reanchor ACK','GPS 승인 relay와 장기 복구 상태를 관리한다. gate reanchor와 reset 확인을 조정한다.',[local('src/relocalization/relocalization_coordinator.cpp'),local('src/relocalization/relocalization_policy.cpp')],'recovery',kind='security'),
 n('startup','초기화 준비','RDDF ready heartbeat','bringup의 initialization/required=true 경로에서 시작 준비 상태가 출력 승인에 관여한다.',[local('launch/safety_and_tf.launch'),local('src/output_gate/localization_output_gate.cpp')],'rddf')
],[e('manager','arbiter','evaluated 상태'),e('arbiter','gate','공개 valid'),e('recovery','arbiter','recovery active')],layout='rows',note='Manager·Coordinator·Supervisor는 supervisor_node.cpp에서 같은 프로세스에 생성된다. Output Gate는 별도 프로세스다.')
# Reuse factual chapter descriptions, matching every source anchor against current working files.
chapters=json.loads((ROOT/local('docs/code-detail/chapters.json')).read_text())
for old,title,items,edges in chapters:
 if old=='sensors':
  subsets=[('encoder','엔코더 검사·Twist 변환','motion',items[:3],edges[:2]),('imu','CalibratedIMU의 입력·보정','motion',items[3:],[(a-3,b-3,l) for a,b,l in edges if a>=3])]
 elif old=='fusion': continue
 else: subsets=[(old,re.sub(r'^\d+\s*','',title),'safety' if old in ['recovery','state','output'] else 'localization',items,edges)]
 for id,title,parent,subset,links in subsets:
  nodes=[n(id+str(i),it[0],it[1],it[2],[(local(it[3]),it[4])],kind='security' if id!='encoder' else 'backend') for i,it in enumerate(subset)]
  config={'encoder':['encoder_calibration.yaml'],'imu':['imu_heading_calibration.yaml','initial_heading.yaml'],'gps':['gps_reference.yaml','time_sync.yaml'],'recovery':['relocalization_policy.yaml'],'state':['status_policy.yaml'],'output':['status_policy.yaml']}[id]
  for node in nodes: node['sources'] += [src(local('config/'+f)) for f in config]
  layer(id,title,parent,nodes,[e(id+str(a),id+str(b),l if l!='다음 처리' else '검사·처리') for a,b,l in (links if links is not None else [(i,i+1,'평가·처리') for i in range(len(subset)-1)])],note='이 화면의 선은 함수 내부 처리 또는 판정 순서다. ROS 프로세스 간 토픽 연결과 구분한다. 자세한 실패 조건·소스·설정을 노드에서 확인한다.')
# Restore the startup condition omitted by the older chapter snapshot.
LAYERS['output']['note']+=' RDDF 초기화가 required인 경우 ready heartbeat도 선행 조건이다.'
LAYERS['output']['nodes'][0]['summary']+=' RDDF 초기화가 required인 경우 초기화 ready와 heartbeat를 먼저 요구한다.'
layer('rddf','RDDF 시작 위치 초기화','localization',[
 n('routes','RDDF 경로 지도','CSV 경로 · origin','경로 파일을 적재하고 WGS84 origin을 기준으로 GPS를 map 평면에 투영한다. 지도 기준이 측량 완료됐다는 의미는 아니다.',[local('scripts/rddf_initialization_core.py'),local('config/rddf_initialization.yaml')],kind='database'),
 n('candidate','후보 선택','GPS 자동 · 수동 클릭','GPS의 연속 후보 또는 수동 Point를 RDDF 선분에 투영한다. 멀거나 분기점에서 모호한 후보는 거부한다.',[(local('scripts/rddf_initialization_core.py'),'    def match('),(local('scripts/rddf_initializer_node.py'),'    def gps_callback('),(local('scripts/rddf_initializer_node.py'),'    def manual_callback(')],target='rddf_match'),
 n('transaction','초기화 호출','heading → Local → Global','서비스 존재와 정지 조건을 확인한 후 heading, Local set_pose, Global set_pose 순서로 호출한다. 예외 시 FAULT이며 앞선 서비스 변경을 되돌리는 rollback 코드는 없다.',[(local('scripts/rddf_initializer_node.py'),'    def initialize('),local('srv/SetInitialHeading.srv')]),
 n('confirm','두 EKF 결과 확인','위치·yaw · 증가 stamp','초기화 후 Local/Global Odometry를 관측하여 설정된 연속 표본의 위치·yaw 허용 범위 충족을 확인한다.',[(local('scripts/rddf_initializer_node.py'),'    def odom_callback('),local('config/rddf_initialization.yaml')]),
 n('ready','준비 상태 발행','Supervisor · Output Gate','초기화 ready heartbeat를 소비하는 경로에 시작 승인 상태를 전달한다. heartbeat stale 및 미준비는 사용 승인과 분리해 관리한다.',[(local('scripts/rddf_initializer_node.py'),'    def publish('),local('src/output_gate/localization_output_gate.cpp')],kind='messagebus')
],[e('routes','candidate','경로 선분'),e('candidate','transaction','확정 후보'),e('transaction','confirm','출력 확인'),e('confirm','ready','연속 조건 충족')],note='자동 후보는 clock 및 정지 조건을 요구한다. 수동 선택도 정지/서비스/출력 확인을 우회하지 않는다. 기본 경계값은 설정 원문에서 확인한다.')
layer('rddf_match','RDDF 선분 매칭과 모호성','rddf',[
 n('project','좌표 투영','WGS84 → map XY','origin으로 GPS 좌표를 투영한다.',[(local('scripts/rddf_initialization_core.py'),'    def project_gps(')]),
 n('segments','선분 최근접점','projection · distance','각 경로 선분으로 직교 투영하고 제한된 t로 최근접점을 구한다.',[(local('scripts/rddf_initialization_core.py'),'    def match(')]),
 n('ambiguity','거리·분기 모호성','거리 제한 · incompatible branch','가까운 후보들 중 서로 양립하지 않는 분기는 모호한 후보로 거부한다. 동일 경로라도 자가 교차 후보가 별개로 남을 수 있다.',[local('scripts/rddf_initialization_core.py'),local('config/rddf_initialization.yaml')],kind='security'),
 n('pose','경로상의 시작 pose','XY · yaw · route · index','승인한 선분에서 시작 위치와 진행 방향을 만든다. 실제 yaw 선택은 원문 분기와 설정을 확인한다.',[local('scripts/rddf_initialization_core.py')],kind='messagebus')
],[e('project','segments','map 좌표'),e('segments','ambiguity','거리 후보'),e('ambiguity','pose','승인 선분')])
layer('timing','센서 시각과 clock_ready','localization',[
 n('host','Host clock 확인','chrony · 동기 상태','check_time_sync.py와 timing_core가 PC의 동기 상태와 offset 정책을 확인한다. 센서 timestamp freshness와 별개의 관측이다.',[local('scripts/check_time_sync.py'),local('scripts/timing_core.py'),local('config/time_sync.yaml')]),
 n('monitor','SensorTimingMonitor','센서·driver·host 진단','센서 stamp 통계와 driver 정책·건강도, host 상태를 진단한다. clock_ready Bool은 host.ready와 host 검사 freshness로 계산하며 sensor/driver 진단 결과를 모두 AND한 값이 아니다. replay는 별도 상태로 표시한다.',[local('scripts/sensor_timing_monitor.py'),local('config/time_sync.yaml')]),
 n('ready','clock_ready','내부 Bool heartbeat','GPS gate와 CalibratedIMU, RDDF 자동 초기화의 clock 조건에 사용된다.',[local('config/localization_interfaces.yaml'),local('scripts/sensor_timing_monitor.py')],kind='messagebus'),
 n('driver','GNSS 시각 정책','NavPVT · receiver epoch','GPS driver timing diagnostics를 구독하여 설정한 시각 정책과 상태를 점검한다.',[local('config/gps_driver.yaml'),(local('scripts/sensor_timing_monitor.py'),'    def _driver_status(')]),
 n('consumers','시각 조건 소비자','GPS gate · 보정 · 초기화','GPS는 원래 측정 stamp에서 Local 이력을 정합한다. 현재 수신 시각이나 현재 yaw로 대체하지 않는다.',[local('src/odometry_gps_fusion/local_odometry_history.hpp'),local('scripts/calibrated_imu_node.py'),local('scripts/rddf_initializer_node.py')],target='gps')
],[e('host','monitor','host 상태'),e('monitor','ready','준비 판정'),e('driver','monitor','timing 진단'),e('ready','consumers','clock_ready')],positions={'host':[60,70],'monitor':[455,70],'ready':[850,70],'driver':[455,380],'consumers':[850,380]},note='토픽의 발행 빈도와 timestamp 정확도는 다르다. 설정값과 코드가 제공하는 관측 범위를 구분한다.')
layer('operations','TF·실행·관측','localization',[
 n('tf','TF 프레임','동적 소유권 · 정적 장착값','Local/Global EKF와 정적 TF publisher의 프레임 소유권, 실측 상태를 확인한다.',[local('config/tf_configuration.yaml'),local('src/tf/transform_configuration.cpp')],target='tf'),
 n('bringup','Bringup','기능 인자 · 필수 초기화','기본 RDDF 초기화 ON, GPS fusion ON. 내장 encoder driver ON, IMU/GPS driver OFF. 외부 센서 시작과 remap을 확인해야 한다.',[local('launch/bringup.launch'),local('launch/sensors.launch')],target='launches'),
 n('replay','Replay·지도 수집','실차 bringup과 다른 옵션','replay.launch와 map_data_collection.launch의 override를 원문으로 제공한다. bag 재생 결과를 실제 센서 clock 검증과 혼동하지 않는다.',[local('launch/replay.launch'),local('launch/map_data_collection.launch')]),
 n('viewer','Viewer·시각화','경로 · 진단 · 초기 위치 선택','localization_viewer.py와 visualization 노드, front scan visualizer가 위치·센서·초기화 정보를 보여 준다.',[local('scripts/localization_viewer.py'),local('scripts/lidar_front_scan_visualizer.py'),local('src/visualization/localization_visualization.cpp')],target='viewer',kind='frontend'),
 n('config','운영 설정','YAML · 측정 여부 · reload','config의 measured/calibration_state 등 준비 상태는 실행 성공이나 센서 정확도와 다르다. 변경은 노드 재시작 계약을 따른다.',[local('docs/configuration.md'),local('config/localization_interfaces.yaml')],kind='database')
],note='각 카드에서 원문 launch, 구성 파라미터와 함수 목록을 열 수 있다.')
layer('tf','TF 프레임과 발행 소유자','operations',[
 n('map','map','전역 ENU 기준','Global EKF의 world_frame. GPS datum/RDDF 기준과 일치시켜야 한다.',[local('config/gps_reference.yaml'),local('config/rddf_initialization.yaml')],kind='external'),
 n('odom','odom','Global EKF: map → odom','Global EKF가 map→odom을 발행한다.',[local('config/ekf_global.yaml')]),
 n('base','base_link','Local EKF: odom → base_link','Local EKF가 odom→base_link를 발행한다. 후륜축 기준과 센서 lever arm 설정을 함께 확인한다.',[local('config/ekf_local.yaml'),local('config/tf_configuration.yaml')]),
 n('sensor_frames','장착 센서 프레임','imu_link · gps_link · laser_link','static_transform_publisher가 활성 정적 변환을 검증해 발행한다. 현재 imu_link는 verified, laser_link는 measured이며 gps_link 정적 TF는 unmeasured·enabled=false다. 이것만으로 GPS fusion 기능이 OFF라는 뜻은 아니다.',[local('config/tf_configuration.yaml'),local('src/tf/static_transform_publisher.cpp'),local('src/tf/transform_configuration.cpp')],kind='external')
],[e('map','odom','Global EKF'),e('odom','base','Local EKF'),e('base','sensor_frames','정적 장착 TF')])
layer('launches','Launch 구성 트리','operations',[
 n('bringup','bringup.launch','최상위 구성','기능별 launch를 include하고 RDDF initializer를 실행한다. launch/localization.launch는 현재 bringup으로 연결된 진입점이다.',[local('launch/bringup.launch'),local('launch.sh')]),
 n('local','local_fusion.launch','전처리·Local EKF','InterfaceAdapter, Normalizer, CalibratedIMU, EncoderAdapter, Local EKF.',[local('launch/local_fusion.launch')],target='motion'),
 n('absolute','절대 위치 관측','gps_fusion','timing monitor/GPS gate를 구성한다.',[local('launch/gps_fusion.launch')]),
 n('global','global_fusion.launch','Global EKF','입출력 remap, 필터 YAML과 초기 heading 옵션을 적용한다.',[local('launch/global_fusion.launch')],target='filters'),
 n('safety','safety_and_tf.launch','Supervisor·Gate·정적 TF','프로세스 합성, 안전 출력 승인과 TF 소유권 검증을 구성한다.',[local('launch/safety_and_tf.launch')],target='safety'),
 n('display','visualization.launch','시각화·Viewer','시각화 구성과 실행 타입은 현재 launch 원문에 근거한다.',[local('launch/visualization.launch'),local('launch/viewer.launch')],target='viewer')
],note='기능별 include의 구성도다. 실행 순서를 직렬화한 그림이 아니며 실제 group 조건은 각 launch에서 확인한다.')
layer('viewer','Viewer·진단·표시','operations',[
 n('odometry','위치·센서·상태','공개 토픽 · 진단 입력','시각화는 추정 결과와 안전 승인 상태를 함께 읽는다. 토픽별 계약은 interfaces YAML이 기준이다.',[local('config/localization_interfaces.yaml')],kind='messagebus'),
 n('visualization','시각화 노드','Path · Marker','차량 상태 표시용 path/marker를 만든다. 시각화 메시지가 위치 출력 승인 자체를 바꾸지는 않는다.',[local('src/visualization/localization_visualization.cpp'),local('config/visualization.yaml')]),
 n('viewer','Localization Viewer','센서 패널 · 경로·초기 위치','Qt 기반 화면의 콜백과 초기화 선택, 센서 정보, 진단 표시를 소스 함수 목록으로 탐색한다.',[local('scripts/localization_viewer.py'),local('config/localization_viewer.yaml')],kind='frontend'),
 n('front','전방 Scan 표시','거리·유효 스캔 시각화','LiDAR front scan visualizer의 필터와 표시 설정. 지도 정합 승인과는 다른 표시 역할이다.',[local('scripts/lidar_front_scan_visualizer.py'),local('config/lidar_front_visualization.yaml')],kind='frontend')
],[e('odometry','visualization','Odometry·상태')],note='화면과 진단은 관측 도구다. 제어 명령을 실행하는 Controller와 구분한다.')

# Deterministic geometry: at most three readable columns; branching maps use explicit positions.
for id,L in LAYERS.items():
 nodes=L['nodes']; positions=L['positions'] or {}
 viewport_fixes=json.loads((OUT/'viewport-repairs.json').read_text()) if (OUT/'viewport-repairs.json').exists() else {}
 positions.update(viewport_fixes.get(id,{}))
 for i,node in enumerate(nodes):
  row,col=divmod(i,3)
  if L['layout']=='snake' and row%2:col=2-col
  node['pos']=positions.get(node['id'],[60+395*col,70+270*row])
  node['size']=[250,90]
  if len(nodes)<=3 and not positions:node['pos'][1]=270
 height=max(n['pos'][1]+n['size'][1] for n in nodes)+130
 overrides_path=OUT/'geometry-repairs.json'
 if overrides_path.exists():
  overrides=json.loads(overrides_path.read_text()).get(id+'.architecture',{})
  for edge in L['edges']:
   edge.update(overrides.get(edge['id'],{}))
   if id in viewport_fixes and 'labelAt' in edge:
    a=next(n for n in nodes if n['id']==edge['from']);b=next(n for n in nodes if n['id']==edge['to'])
    if a['pos'][0]==b['pos'][0]:edge['labelAt']=[a['pos'][0]+125,(a['pos'][1]+90+b['pos'][1])/2]
 spec={'schema_version':1,'diagram_type':'architecture','meta':{'title':L['title'],'quality_profile':'showcase','viewBox':[1160,max(650,height)]},'components':[{k:n[k] for k in ['id','type','label','sublabel','pos','size']} for n in nodes],'connections':L['edges']}
 (OUT/(id+'.architecture.json')).write_text(json.dumps(spec,ensure_ascii=False,indent=2)+'\n')
# Source snapshots are local working-tree evidence, not a claim about HEAD or live ROS.
SOURCES={}
for L in LAYERS.values():
 for node in L['nodes']:
  for s in node['sources']:
   path=s['path']
   if path in SOURCES:continue
   raw=(ROOT/path).read_text()
   # Never copy authentication values from driver launch/config examples into the guide.
   lines=raw.splitlines();redacted=[]
   for i,line in enumerate(lines):
    if re.search(r'(?i)(password|passwd|username|api[_-]?key|access[_-]?token|secret)',line):
     lines[i]='[인증 관련 설정 줄은 문서 스냅샷에서 생략]';redacted.append(i+1)
   functions=[]
   for i,line in enumerate(lines):
    if re.match(r'\s*(?:class |def )',line) or re.match(r'^(?:[\w:<>,&*]+\s+)+[\w]+::[\w]+\(',line):functions.append({'line':i+1,'label':line.strip()})
   params=[]
   if path.endswith('.yaml'):
    import yaml
    try:
     def flatten(v,prefix=''):
      if isinstance(v,dict):
       for k,x in v.items():flatten(x,prefix+'.'+str(k) if prefix else str(k))
      else:params.append([prefix,v])
     flatten(yaml.safe_load(raw))
     params=[r for r in params if not re.search(r'(?i)password|username|token|secret',r[0])]
    except Exception:pass
   SOURCES[path]={'path':path,'sha256':hashlib.sha256(raw.encode()).hexdigest(),'text':'\n'.join(lines),'line_count':len(lines),'redacted_lines':redacted,'functions':functions,'params':params}
# Verify graph references and full reachability through navigation (cross-links are intentional).
for L in LAYERS.values():
 ids={n['id'] for n in L['nodes']}
 assert len(ids)==len(L['nodes'])
 for edge in L['edges']:assert edge['from'] in ids and edge['to'] in ids,(L['id'],edge)
 for node in L['nodes']:assert not node['target'] or node['target'] in LAYERS,node
model={'title':'HL-FMA2026 아키텍처 탐색기','language':'ko','archify_original_ui':'English fallback (Korean authored content)','layers':LAYERS,'sources':SOURCES,'provenance':{'basis':'현재 working tree의 정적 코드·설정 확인','git_head':subprocess.check_output(['git','rev-parse','HEAD'],cwd=ROOT,text=True).strip(),'runtime_verified':False}}
(OUT/'model.json').write_text(json.dumps(model,ensure_ascii=False,indent=2)+'\n')
(OUT/'source-manifest.json').write_text(json.dumps({p:{k:v[k] for k in ['sha256','line_count','redacted_lines']} for p,v in SOURCES.items()},ensure_ascii=False,indent=2)+'\n')
print(json.dumps({'layers':len(LAYERS),'nodes':sum(len(L['nodes']) for L in LAYERS.values()),'sources':len(SOURCES)}))
