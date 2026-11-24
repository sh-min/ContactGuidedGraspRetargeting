# 논문용 그림

모든 그림은 HRDexDB의 같은 사람 오른손 레몬 파지 영상(`human/lemon/0`, 카메라 `25452062`, 30 fps)을 사용했다. 프레임 번호는 0부터 센다. PNG는 화면 열람과 원고 삽입에, PDF는 글자와 배치를 유지한 인쇄 조판에 사용할 수 있다.

| 용도 | 프레임 | 독자가 확인할 내용 | 파일 |
| --- | --- | --- | --- |
| 그림 1 · 손 형태 간 변환 | 100 · 3.33초 | 같은 사람 자세를 Robotiq 2F-85와 Shadow Hand에 적용한 예 | [PNG](fig01-morphologies.png) · [PDF](fig01-morphologies.pdf) |
| 그림 2 · 방법 간 비교 | 184 · 6.13초 | 같은 Panda에서 CGR과 두 비교 방법의 양쪽 패드 배치가 모두 뚜렷하게 다른 예 | [PNG](fig02-panda-f184.png) · [PDF](fig02-panda-f184.pdf) |
| 부록 그림 S1 · CGR 자체의 근접도 | 116 · 3.87초 | CGR의 좌우 패드 중 더 먼 쪽까지 레몬에 가장 가까운 예 | [PNG](supp-panda-f116.png) · [PDF](supp-panda-f116.pdf) |

## 그림 1. 손 형태에 따른 자세

왼쪽은 입력 영상이며 가운데와 오른쪽은 CGR 출력이다. 위 행은 전체 장면, 아래 행은 세 열에서 동일하게 자른 확대 영역이다. 두 로봇 영상은 같은 프레임에서 사람의 팔과 소매를 지운 배경, 같은 레몬 형상과 외관을 사용한다. 두 손의 관절 자세는 각각의 모델에서 계산했다. Shadow Hand의 뒤쪽 손가락처럼 원래 시점에서 가려지는 부분이 있어 이 그림으로 판독할 수 있는 범위는 투영된 자세와 가시적인 손끝 배치까지다.

## 그림 2와 부록 그림 S1. 공통 Panda 비교

위 행의 첫 번째 열은 사람 손이 담긴 입력 영상이며 나머지 세 열은 원본 카메라 시점에 로봇을 합성한 결과다. Masquerade와 Ego2Robot은 각 방법의 목표 자세를 공통 Panda 제어기에 연결했다. CGR은 레몬 형상과 물체 자세, 추정된 사람 손의 접촉 정보를 추가로 이용한다.

아래 행의 첫 번째 열은 같은 순간의 사람 손과 레몬을 3D로 재구성한 모습이다. 세 로봇 열에서는 각 방법의 저장된 관절 자세와 레몬 형상을 그대로 두고 팔 링크를 숨겼다. 사람 손목·엄지 끝·검지 끝이 이루는 평면을 수직으로 보는 가상 카메라를 프레임마다 한 번 정해 모든 열에 적용했다. 원본 시점에서 팔이 가리는 패드와 레몬의 상대적 배치를 이 행에서 읽을 수 있다.

각 안쪽 패드에서 면적에 따라 512개 점을 표집하고, 레몬 메시 표면까지 부호 없는 거리의 5백분위수를 계산했다. 단위는 mm이며 값이 작을수록 해당 패드의 가까운 표면 영역이 레몬 메시에 가까이 있다.

| 프레임 | 방법 | 왼쪽 패드 (mm) | 오른쪽 패드 (mm) |
| --- | --- | ---: | ---: |
| 184 | Masquerade | 9.56 | 7.98 |
| 184 | Ego2Robot | 8.81 | 9.73 |
| 184 | CGR | 0.57 | 0.53 |
| 116 | Masquerade | 5.92 | 7.71 |
| 116 | Ego2Robot | 9.49 | 9.57 |
| 116 | CGR | 0.15 | 0.25 |

선정 대상은 같은 영상에서 엄지와 검지의 접촉이 함께 추정된 짝수 프레임 62개다. 184번에서는 좌우 패드를 각각 두 방법과 비교한 네 가지 거리 차이 중 가장 작은 값도 7.45 mm로, 이 값이 후보 중 최대였다. 116번은 CGR만 보았을 때 두 패드 중 더 먼 쪽의 거리가 가장 작았다. 두 도판은 이렇게 선정한 기하학적 사례이며 실제 접촉력이나 물체를 들어 올려 유지하는 능력은 별도의 실험으로 평가해야 한다.

## 출처와 이용 조건

입력 영상, 카메라 보정, 손·물체 자세 및 물체 형상의 출처는 Jongbin Lim 등 HRDexDB 연구진이다. 원 데이터의 이용 조건은 CC BY-NC 4.0이다. 이 도판을 비상업적 연구·발표에 재사용할 때 저자, 출처, 이용 조건과 변경 내용(프레임 추출·확대, 배경 복원, 로봇 합성 또는 보조 시점 렌더링)을 함께 표시한다. [HRDexDB 논문](https://arxiv.org/abs/2604.14944) · [공식 데이터 카드](https://huggingface.co/datasets/HRDexDB/HRDexDB) · [CC BY-NC 4.0](https://creativecommons.org/licenses/by-nc/4.0/)

그림 1의 로봇 모델은 MuJoCo Menagerie 배포본의 Robotiq 2F-85와 Shadow Hand이다. Panda 비교에는 Ego2Robot 배포본에 포함된 Franka Panda 모델을 사용했다. 아래 행의 사람 손 재구성은 Romero, Tzionas, Black의 MANO에 기반한다. [Robotiq 2F-85](https://github.com/google-deepmind/mujoco_menagerie/tree/d7b481bac63d3d600ab404ee74f7474c5efe8a67/robotiq_2f85) · [Shadow Hand](https://github.com/google-deepmind/mujoco_menagerie/tree/d7b481bac63d3d600ab404ee74f7474c5efe8a67/shadow_hand) · [Panda 모델](https://github.com/www-Ye/ego2robot_blog/tree/244a4ee02f02b2e93e94a81f27963597e0815ce6/ego2robot/robot/xml/panda) · [MANO](https://mano.is.tue.mpg.de/)
