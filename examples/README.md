# sim.py 

## Description
Open AI Gym 기반으로 제작된 F1TENTH 시뮬레이터 실행 파일입니다. 

## Quick Start
```shell
python sim.py
```

## Explanation
- planner 라이브러리에서 주행 알고리즘이 작성되어 있는 클래스를 불러 옵니다. 
```python
# planner 불러오기
from planner import fgm_convolution as fc
from planner import wall_following as wf
from planner import pure_pursuit as pp
```


```python
# 시뮬레이터 실행
if __name__ == "__main__":
    with open('params.yaml') as file:
        conf_dict = yaml.load(file, Loader=yaml.FullLoader)
    conf = Namespace(**conf_dict)
    
    # planners 변수에 실행하기 원하는 planner Class를 추가하여 시뮬레이팅 할 수 있습니다.
    
    # 자동차 3대
    planners = [fc.FgPlanner(conf, 0.3302), pp.PurePursuitPlanner(conf, 0.3302), wf.WallPlanner(conf, 0.3302)] 
    # 자동차 2대 
    planners = [fc.FgPlanner(conf, 0.3302), pp.PurePursuitPlanner(conf, 0.3302)]
    # 자동차 1대
    planners = [fc.FgPlanner(conf, 0.3302)]
    
    env_process = EnvProcess(conf, planners)
    env_process.main()
```


## Environment Functions

- `_render_callback()`: 시뮬레이터 실행 시 맵에 웨이포인트를 표시하거나, 첫번째 자동차를 중심으로 렌더링하는 함수입니다.  
- `_pack_obs()`: 시뮬레이터 실행 시 자동차의 위치와 속도를 포함하여 각 자동차의 상태를 포함하는 함수입니다.  
- `reset()`: 시뮬레이터 초기화 함수입니다.  
- `step()`: 시뮬레이터 실행 함수입니다.  
- `render()`: 시뮬레이터 렌더링 함수입니다.  
- `main()`: 시뮬레이터 메인 프로세스 함수입니다. 이 함수를 사용하여 시뮬레이터를 실행할 수 있습니다.


## Observation
환경으로 부터 받는 observation은 다음과 같은 형태로 Return 됩니다.
```
obs  : {'ego_idx': 0,
'scans': [array([1.86620415, 1.89307968, 1.8363868 , ..., 1.81007522, 1.80036026, 1.81282915])],
'poses_x': [-1.9], 'poses_y': [-3.0], 'poses_theta': [4.1421595],
'linear_vels_x': [0.0], 'linear_vels_y': [0.0], 'ang_vels_z': [0.0],
'collisions': array([0.]),
'lap_times': array([0.01]),
'lap_counts': array([0.])}
```
* `ego_idx` : ego 차량의 인덱스를 나타냅니다. 환경에서 4대까지 동시에 경주할수 있으므로 그 중 한대의 인덱스를 리턴합니다.
* `scans` : LiDAR 센서 데이터입니다. 4.7 radian 을 약 1080개의 점으로 나누어 각각 포인트에 대한 거리 값이 들어있는 배열을 리턴합니다.
* `poses_x` : 각 차량들의 x 좌표가 있는 float 배열을 리턴합니다.
* `poses_y` : 각 차량들의 y 좌표가 있는 float 배열을 리턴합니다.
* `poses_theta` : 각 차량들의 theta 좌표가 있는 float 배열을 리턴합니다.
* `linear_vels_x` : 각 차량들의 종방향 속도를 나타내는 float 배열을 리턴합니다.
* `linear_vels_y` : 0으로 이루어진 있는 float 배열을 리턴합니다.
* `ang_vels_z` : 각 차량들의 yaw rate를 나타내는 float 배열을 리턴합니다.
* `collisions` : 차량들의 충돌 여부를 나타내는 boolean 배열을 리턴합니다. True는 충돌입니다.
* `lap_times` : 현재 랩타임을 출력합니다.
* `lap_counts` : 현재 차량이 몇바퀴 돌았는지를 나타내는 배열을 리턴합니다.

## Step_reward
환경이 진행되는 Step 당 몇 초로 계산할 것인지를 설정하는 부분입니다. 환경에서 몇초가 지났는지를 표시하기 위해사용됩니다.  
```step_reward  : 0.01```

## Done
환경이 완료 되었는 지를 나타내는 부분입니다.  
```done  : False```

## Info
```
info  : {'checkpoint_done': array([False])} -
(info  : {'checkpoint_done': array([ True])}
```

`checkpoint_done`: 각 Agent가 완주를 하였는지를 나타냄