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
from planner import fgm_gnu as fg
```

- sim.py의 메인 프로세스입니다.  파라미터를 로드하여 planner와 EnvProcess에 전달하여 EnvProcess의 main()를 실행 시키는 구조로 되어있습니다. 
```python
# Sim.py main process
if __name__ == "__main__":
    # 파라미터를 로드하는 부분
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

- 환경의 메인 프로세스 입니다.
```python
# Envrionment Main Process
def main(self):
    self.render()
    start = time.time()
    laptime = 0.0
    while self.done == False or isinstance(self.done, float):
        actions = []
        futures = []
        # 병렬 연산을 위해 ThreadPoolExecutor() 함수 사용 
        with ThreadPoolExecutor() as executor:
            for i, p in enumerate(self.planners):
                # Planner Class에 plan이나 driving 함수 사용
                if hasattr(p, 'plan'):
                    futures.append(executor.submit(p.plan, self._pack_obs(i)))
                elif hasattr(p, 'driving'):
                    futures.append(executor.submit(p.driving, self._pack_obs(i)))
        # 병렬 연산이 완료 되었을 경우에 action에 각각의 planner가 도출한 Steering Angle 값과 Speed 값을 삽입
        for future in futures:
            speed, steer = future.result()
            actions.append([steer, speed])
        # 시뮬레이터에서 Action 수행 
        self.step(actions)
        self.render()
        laptime += self.step_reward
```
### Observation (self.obs)
Observation은 `dict()` 자료형이며, Planner가 필요로 하는 데이터를 포함하고 있습니다.

 `scans`, `poses_x`, `poses_y`, `poses_theta`, `linear_vels_x`, `linear_vels_y`, `ang_vels_z`, `collisions`, `lap_times`, `lap_counts` 는 planner의 개수에 따라 리턴되는 배열의 길이가 달라집니다. 

```python
# Example (Solo)
{'ang_vels_z': [0.21410657322896876],
 'collisions': array([0.]),
 'ego_idx': 0,
 'lap_counts': array([0.]),
 'lap_times': array([0.01]),
 'linear_vels_x': [1.5976799999999998],
 'linear_vels_y': [0.0],
 'poses_theta': [4.157316060524506],
 'poses_x': [-1.065539773722662],
 'poses_y': [-0.10794704909029744],
 'scans': [array([0.6761423 , 0.67802234, 0.67004165, ..., 3.19306503, 3.16019914,
       3.133911  ])]}
```

```python
# Example (Duo)
{'ang_vels_z': [0.11602889767770622, -1.5098502899182016],
 'collisions': array([0., 0.]),
 'ego_idx': 0,
 'lap_counts': array([0., 0.]),
 'lap_times': array([0.01, 0.01]),
 'linear_vels_x': [5.0212800000000035, 2.7387574993163417],
 'linear_vels_y': [0.0, 0.0],
 'poses_theta': [4.217987940042492, 3.63788345704231],
 'poses_x': [-1.638661850122417, -0.20652682338989056],
 'poses_y': [-1.1328136458418403, -1.1006959963950702],
 'scans': [array([0.82393009, 0.8300521 , 0.828538  , ..., 3.11615862, 3.1745315 ,
       3.19225711]),
           array([6.15585722, 6.20618202, 5.99863893, ..., 0.98217016, 0.99055202,
       1.00827764])]}
```
* `ang_vels_z` : 각 차량들의 yaw rate를 나타내는 Float Array를 포함합니다.
* `collisions` : 차량들의 충돌 여부를 나타내는 Bool Array를 포함합니다. `True`시 충돌이며, 기본값은 `False` 입니다.
* `ego_idx` : Unused
* `lap_counts` : 현재 차량이 완주한 Lap을 나타내는 Float Array를 포함합니다. 기본값은 `0.0` 입니다.
* `lap_times` : 현재 Lap Time을 나타내는 Float Array를 포함합니다. 
* `linear_vels_x` : 각 차량들의 종방향 속도를 나타내는 Float Array를 포함합니다.
* `linear_vels_y` : Unused
* `poses_theta` : 각 차량들의 theta 좌표가 있는 Float Array를 포함합니다. 
* `poses_x` : 각 차량들의 x 좌표가 있는 Float Array를 포함합니다.
* `poses_y` : 각 차량들의 y 좌표가 있는 Float Array를 포함합니다.
* `scans` : LiDAR 센서 데이터입니다. 4.7 radian 을 약 1080개의 점으로 나누어 각각 포인트에 대한 거리 값이 들어있는 Float Array를 포함합니다.

### Step_reward (self.step_reward)
환경이 진행되는 Step 당 몇 초로 계산할 것인지를 설정하는 부분입니다. 환경에서 몇초가 지났는지를 표시하기 위해 사용됩니다.  
* `0.01 (Float)`

### Done (self.done)
환경이 완료 되었는 지를 나타내는 부분입니다.  
* `False (Bool)`

### Info (self.info)
```python
# Example (Solo)
{'checkpoint_done': array([False])}

# Example (Duo)
{'checkpoint_done': array([False, False])}
```

* `checkpoint_done`: 각 Agent가 완주를 하였는지를 나타내며 Bool이 포함된 Array를 포함합니다. 

## Environment Functions

- `_render_callback()`: 시뮬레이터 실행 시 맵에 웨이포인트를 표시하거나, 첫번째 자동차를 중심으로 렌더링하는 함수입니다.  
- `_pack_obs()`: 각 자동차의 상태를 포함하는 Observation 값을 Planner의 Index에 따라 리패키징하여 리턴하는 함수 입니다. 
- `reset()`: 시뮬레이터 초기화 함수입니다.  
- `step()`: 시뮬레이터 실행 함수입니다.  
- `render()`: 시뮬레이터 렌더링 함수입니다.  
- `main()`: 시뮬레이터 메인 프로세스 함수입니다. 이 함수를 사용하여 시뮬레이터를 실행할 수 있습니다.

