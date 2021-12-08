# params.yaml
본문에서는 `params.yaml`을 이용하여 차량과 환경을 설정하는 방법에 대해서 다룹니다.

## map paths
해당 부분은 환경에서 맵을 불러오기 위해 사용되는 것입니다.
* `map_path` 는 맵의 경로를 설정하기 위해서 사용합니다. 일반적으로 이미지 파일인 png형식의 이미지 파일을 사용합니다.  
* `map_ext` 는 맵파일의 확장자를 설명합니다.

## starting pose for map
맵에서 차량의 위치를 정의하기 위하여 사용합니다. 해당 환경에서는 동시에 4대의 차량이 주행 가능하므로 4대 까지 설정가능합니다.
```yaml
...
  p1: 설정할 포지션의 번호입니다 1~4번으로 구성되어 있습니다.
    sx: 해당 포지션의 x좌표입니다.
    sy: 해당 포지션의 y좌표입니다.
    stheta: 해당 포지션에서 출발할때 차량의 각도입니다 단위는 Radian 입니다.
...
```

## raceline path and indices
waypoint 를 사용할때, waypoint파일의 경로와 각각 좌표의 인덱스를 나타냅니다.
* `wpt_path` : waypoint 파일의 경로입니다.
* `wpt_delim` : waypoint 파일을 load 할때 사용할 csv의 구분자를 설정합니다.
* `wpt_xind` : waypoint column 들 중 x 좌표에 해당하는 index 를 표시합니다.
* `wpt_yind` : waypoint column 들 중 y 좌표에 해당하는 index 를 표시합니다.
* `wpt_thind` : waypoint column 들 중 헤딩을 나타내는 theta에 해당하는 index 를 표시합니다.
* `wpt_vind` : waypoint column 들 중 속도에 해당하는 index 를 표시합니다.

## render params
해당 환경을 rendering 할때 사용되는 옵션을 나타냅니다.
* `render_waypoints` : render 화면에 waypoint를 표시할것인지를 설정합니다.
* `render_center` : rendering 할때, ego 차량(p1)을 추적할 것인지를 설정합니다.
* `render_mode` : 시뮬레이션 속도를 설정합니다. 'human', 'human_fast' 두가지 속도가 있으며 'human_fast'는 'human'에 비해 약 2배의 속도로 진행됩니다.

## vehicle params
환경에 사용될 차량을 설정합니다.
* `mass` : 차량의 중량을 설정합니다.
* `lf` : 차량의 무게중심으로 부터 전방까지의 축간거리(wheelbase)를 설정합니다.


## planner set speed
* `max_speed`: planner에서 사용할 최대속도입니다.
* `min_speed`: planner에서 사용할 최저속도입니다.

##varied params bound
환경에서 사용되는 다양한 매개변수 입니다. 각 값은 Fix되어 있으며 수정하지 않는것을 **권장**합니다.
### physical params
* `mass_min`: 차량무게의 최솟값입니다.
* `mass_max`: 차량무게의 최댓값입니다.
* `lf_min`: 차량 전방까지의 wheelbase의 최솟값입니다.
* `lf_max`: 차량 전방까지의 wheelbase의 최댓값입니다.
### controller params
* `tlad_min`: ?
* `tlad_max`: ?
* `vgain_min`: ?
* `vgain_max`: ?