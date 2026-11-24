# 논문을 쓰면서 찾아볼 원문

아래의 인용 키는 [references.bib](references.bib)와 일치한다. 그림 번호는 각 링크에서 열리는 판본을 기준으로 한다.

## 그림을 왜 두 시점으로 제시하는가

| 인용 키 | 바로 찾아볼 곳 | 이 논문에 참고할 내용 |
|---|---|---|
| `wu2026diffmorphgrasp` | Wu 등, *DiffMorphGrasp*, CoRL 2026. [Fig. 11](https://arxiv.org/html/2602.00915v2#A5.F11) · [부록 B](https://arxiv.org/html/2602.00915v2#A2) · [저자 프로젝트](https://georgewuzy.github.io/DiffMorphGrasp-website/) | 자기 방법, GenDexGrasp, DRO-Grasp의 로봇 파지를 각각 두 각도에서 보여준다. 그립의 안정성은 물체에 여러 방향의 힘을 가하는 별도 시뮬레이션으로 측정한다. |
| `ye2022whats` | Ye 등, *What's in Your Hands?*, CVPR 2022. [Fig. 3, PDF 5쪽](https://openaccess.thecvf.com/content/CVPR2022/papers/Ye_Whats_in_Your_Hands_3D_Reconstruction_of_Generic_Objects_in_CVPR_2022_paper.pdf#page=5) | 사람 손과 물체의 복원 결과를 비교할 때 입력 영상의 시점과 가상 시점을 함께 쓴다. 원본 시점과 보조 렌더를 짝지어 배치하는 선례다. |
| `qin2022dexmv` | Qin 등, *DexMV*, ECCV 2022. [Fig. 4, 저자판 PDF 8쪽](https://arxiv.org/pdf/2108.05877#page=8) | 같은 사례의 사람 손–물체 자세, 손끝 매핑으로 얻은 로봇 자세, 제안 방법으로 얻은 자세를 세 행에 배치한다. 사람의 시연을 기준으로 리타게팅 방법 간 손가락 배치를 읽을 때 참고할 수 있다. |

우리 보조 그림의 카메라 방향은 사람의 엄지 끝·검지 끝·손목이 만드는 평면에서 정한다. 같은 프레임의 모든 방법에 같은 카메라와 배율을 쓰고, 저장된 로봇 관절 자세를 적용한다. 팔의 근위 링크를 숨긴 것은 패드와 레몬 사이의 틈을 읽기 위한 표시상의 선택이다. 이 세부 조건은 우리 구현에 해당하며 캡션에 직접 밝힌다.

## 접촉과 표면 거리를 어떻게 해석하는가

| 인용 키 | 바로 찾아볼 곳 | 이 논문에 참고할 내용 |
|---|---|---|
| `grady2021contactopt` | Grady 등, *ContactOpt*, CVPR 2021. [Fig. 6 및 Table 1, PDF 7쪽](https://openaccess.thecvf.com/content/CVPR2021/papers/Grady_ContactOpt_Optimizing_Contact_To_Improve_Grasps_CVPR_2021_paper.pdf#page=7) | Fig. 6은 손 표면점과 물체 사이의 거리 분포를 보여준다. Table 1은 열영상으로 관측한 접촉과 비교한 정밀도·재현율을 함께 보고한다. CGR 그림 아래의 수치는 각 패드와 레몬 메시 사이의 **부호 없는 기하 거리**다. |
| `brahmbhatt2020contactpose` | Brahmbhatt 등, *ContactPose*, ECCV 2020. [논문](https://arxiv.org/abs/2007.09545) · [데이터와 시각화](https://contactpose.cc.gatech.edu/) | 사람 손이 물체에 남긴 열을 이용해 접촉 위치를 관측하고 손의 자세와 짝지은 데이터다. 손의 자세와 별도로 접촉을 관측하는 평가가 무엇인지 설명할 때 참고할 수 있다. |

## 방법, 입력 데이터와 비교 구현

| 인용 키 | 원문 또는 저자 공개 구현 | 이 논문에서의 역할 |
|---|---|---|
| `choi2026dexstitch` | Choi 등, *DexStitch*, arXiv 2026. [§III-B](https://arxiv.org/html/2610.17789v1#S3.SS2) · [§III-C](https://arxiv.org/html/2610.17789v1#S3.SS3) | 손가락별 접촉으로 로봇 관절을 보정하고, 물체 가림을 고려해 로봇 손을 합성하는 선행 모듈이다. CGR의 기술적 출발점을 설명할 때 인용한다. |
| `lim2026hrdexdb` | Lim 등, *HRDexDB*, arXiv 2026. [논문](https://arxiv.org/abs/2604.14944) · [공식 프로젝트](https://snuvclab.github.io/HRDexDB/) · [데이터 카드](https://huggingface.co/datasets/HRDexDB/HRDexDB) | 레몬 시연과 카메라·물체·손 주석의 출처다. 데이터 카드의 이용조건은 [CC BY-NC 4.0](https://creativecommons.org/licenses/by-nc/4.0/)이다. 비교 그림은 `human/lemon/0`의 카메라 `25452062` 프레임을 잘라 로봇 및 복원 형상과 합성했다. |
| `romero2017mano` | Romero 등, *Embodied Hands*, ACM TOG 2017. [저자 논문 페이지](https://is.mpg.de/ps/en/publications/embodiedhands) · [MANO 이용 안내](https://mano.is.tue.mpg.de/license.html) | 원본 사람의 손을 3D로 표시하고 관절을 얻을 때 사용하는 MANO 손 모델의 출처다. |
| `lepert2026masquerade` | Lepert 등, *Masquerade*, ICRA 2026. [저자 프로젝트의 Approach](https://masquerade-robot.github.io/) · [비교에 사용한 공개 코드 버전](https://github.com/MarionLepert/phantom/tree/a8bb81c1bbe6ade129a1f6f0906482f510354a5e) | 사람 손의 궤적을 이용해 로봇의 목표 자세와 그리퍼 폭을 얻는 비교 방법이다. 이 도판에서는 손 관절 주석을 입력해 공개 처리기로 만든 명령을 공통 Panda 제어기에 전달했다. |
| `wang2026ego2robot` | Wang 등, *Ego2Robot*, arXiv 2026. [§3](https://arxiv.org/html/2608.02580v1#S3) · [비교에 사용한 공개 코드 버전](https://github.com/www-Ye/ego2robot_blog/tree/244a4ee02f02b2e93e94a81f27963597e0815ce6) | 같은 손 관절 주석과 카메라 외부 파라미터를 공개 동작 처리기에 입력하고, 산출한 명령을 공통 Panda에 적용했다. 장면 복원과 렌더링은 세 방법에 공유했다. |
