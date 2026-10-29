# 회전교차로 코드 핵심 실행 명령

## 1. 기존 주행 종료

```bash
cd ~/AutoVehicle_new_final_ws/morai_ws/docker/final_ws
bash run_highway.sh stop || true
```

## 2. `test/roundabout` 브랜치 받기

```bash
cd ~
git clone --branch test/roundabout --single-branch \
  https://github.com/shinejihun1227/AutoVehicle.git \
  AutoVehicle_roundabout
```

## 3. 실행 설정 만들기

```bash
cd ~/AutoVehicle_roundabout/morai_ws/docker/final_ws
cp -n highway.env.example highway.env
cp -n highway-test.env.example highway-test.env
nano highway.env
```

`highway.env`에서 실제 IP와 새 컨테이너 이름을 설정한다.

```bash
UBUNTU_IP=192.168.10.21
MORAI_IP=192.168.10.14
CONTAINER_NAME=morai-roundabout-net10
IMAGE_NAME=morai-final:highway
```

`IMAGE_NAME`은 `docker images`에 표시되는 기존 ROS 이미지 이름과 맞춘다.

## 4. 모델과 Docker 이미지 확인

```bash
python3 fetch_highway_model.py
docker images

IMAGE_NAME="$(sed -n 's/^IMAGE_NAME=//p' highway.env)"
docker image inspect "$IMAGE_NAME" >/dev/null \
  && echo "IMAGE OK: $IMAGE_NAME" \
  || echo "IMAGE NOT FOUND: $IMAGE_NAME"
```

`IMAGE OK`이면 빌드하지 않고 다음 단계로 이동한다. `IMAGE NOT FOUND`일 때만
다음 명령으로 이미지를 빌드한다.

```bash
BUILDKIT_PROGRESS=plain bash run_highway.sh build \
  2>&1 | tee ~/roundabout-build.log
```

빌드가 실패하면 아래 결과를 확인하고, 원인을 해결하기 전에는 뒤의
`start`, `stop`, `install_curvature_signal.sh`를 실행하지 않는다.

```bash
grep -nE 'ERROR|FAILED|Traceback|CMake Error|No module|not found' \
  ~/roundabout-build.log | tail -80
```

빌드 후 이미지가 생성됐는지 반드시 확인한다.

```bash
docker image inspect "$IMAGE_NAME" >/dev/null && echo "IMAGE OK"
```

## 5. 컨테이너 생성과 코드 설치

```bash
bash run_highway.sh start
bash run_highway.sh stop
bash install_curvature_signal.sh --config
bash run_highway.sh start
```

## 6. 설정과 센서 확인

```bash
bash run_test.sh show 2
bash run_test.sh monitor 2
```

센서 수신과 `RUNTIME_MATCH`를 확인한 후 `Ctrl+C`로 `monitor`를 종료한다.

## 7. 실제 주행

```bash
bash run_test.sh drive 2
```

별도 터미널에서 진단하려면 다음을 실행한다.

```bash
cd ~/AutoVehicle_roundabout/morai_ws/docker/final_ws
bash run_test.sh diagnose 2
```

주행은 `Ctrl+C`로 종료한다. 컨테이너까지 종료하려면 다음을 실행한다.

```bash
bash run_highway.sh stop
```

> `monitor 2`는 차량 제어 UDP를 보내지 않고, `drive 2`부터 실제 제어 명령을 보낸다.
> 기존 환경과 새 환경은 같은 UDP 포트를 사용하므로 동시에 실행하지 않는다.
