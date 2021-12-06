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