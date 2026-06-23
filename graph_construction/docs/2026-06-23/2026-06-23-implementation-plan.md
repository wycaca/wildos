# Graph Construction 瀹炵幇鏂规

鏃ユ湡: 2026-06-23

鐩爣: 鍦?`external/wildos` 婧愮爜鐩綍涓柊澧?`graph_construction` 妯″潡, 鍏堝疄鐜板彲鑱旇皟鐨勫熀纭€鐗堟湰, 鎵撻€?`T_geo -> NavigationGraph -> WildOS scoring -> graphnav_planner` 杩欐潯閾捐矾

## 褰撳墠杈圭晫

鏈樁娈靛彧瀹炵幇 Graph Construction 妯″潡, 涓嶆敼 `graphnav_msgs`, 涓嶉噸鍐?`graphnav_planner`, 涓嶆敼 WildOS 瑙嗚璇勫垎涓绘祦绋?
Graph Construction 璐熻矗鍙戝竷鍘熷鍑犱綍鍥?

```text
/spot1/nav_graph
```

WildOS 宸叉湁瑙嗚妯″潡璐熻矗鎶婂師濮嬪浘璇勫垎涓?

```text
/spot1/scored_nav_graph
```

宸叉湁 `graphnav_planner` 璐熻矗璺緞鎼滅储:

```text
scored_nav_graph -> Dijkstra -> graphnav_planner/path
```

## 鏂板妯″潡浣嶇疆

鏂板 ROS 2 Python 鍖?

```text
external/wildos/graph_construction
```

寤鸿鐩綍:

```text
graph_construction/
  package.xml
  setup.py
  resource/graph_construction
  configs/graph_construction.yaml
  launch/graph_construction.launch.py
  graph_construction/
    __init__.py
    node.py
    graph_builder.py
    grid_adapter.py
    frontier_detector.py
    edge_builder.py
    graph_memory.py
    deadend_recovery.py
    msg_utils.py
    viz.py
```

閫夋嫨 Python 鐨勫師鍥?

- 绗竴鐗堥噸鐐规槸绠楁硶闂幆鍜屽彲璋冭瘯鎬?- 渚夸簬蹇€熸帴涓嶅悓 grid topic
- 渚夸簬鍐欐竻妤氭敞閲婂拰鍒嗘ā鍧楅獙璇?- 鍚庣画鎬ц兘鐡堕鏄庣‘鍚? 鍐嶆妸 SDF 鎴?edge collision check 绉诲埌 C++

## 杈撳叆杈撳嚭

绗竴鐗堣緭鍏?

```text
/spot1/odom, nav_msgs/Odometry
/spot1/traversability_grid, nav_msgs/OccupancyGrid
```

鍚庣画杈撳叆:

```text
grid_map_msgs/GridMap
elevation_mapping_cupy output
```

绗竴鐗堣緭鍑?

```text
/spot1/nav_graph, graphnav_msgs/NavigationGraph
/spot1/graph_construction_viz, visualization_msgs/MarkerArray
/spot1/debug_traversability_grid, nav_msgs/OccupancyGrid
```

## 鏁版嵁妯″瀷

鍐呴儴缁存姢涓€涓交閲?graph memory:

```text
GraphState:
    nodes: dict[node_id, InternalNode]
    edges: set[(from_id, to_id)]
    current_node_id: node_id
    removed_frontiers: list[position]
```

鑺傜偣瀛楁:

```text
InternalNode:
    uuid
    position
    free_radius
    explored_radius
    frontier_points
    is_frontier
    last_seen_time
    failed_frontier_count
```

ROS 杈撳嚭鏄犲皠:

```text
InternalNode -> graphnav_msgs/Node
InternalEdge -> graphnav_msgs/Edge
GraphState -> graphnav_msgs/NavigationGraph
```

## 鍩虹绠楁硶

鏁翠綋娴佺▼:

```text
function UpdateGraph(grid, odom, previous_graph):
    classified_grid = ClassifyGrid(grid)
    sdf_obstacle = DistanceToObstacle(classified_grid)
    sdf_unknown = DistanceToUnknown(classified_grid)

    UpdateExistingNodes(previous_graph, sdf_obstacle, sdf_unknown)
    SampleFreeNodes(classified_grid, previous_graph)
    DetectAndAssignFrontiers(classified_grid, previous_graph)
    BuildCollisionFreeEdges(classified_grid, previous_graph)
    UpdateCurrentNode(odom, previous_graph)
    ApplyDeadendRecovery(previous_graph)

    return NavigationGraph
```

### 鍦板浘鍒嗙被

绗竴鐗堝皢 `OccupancyGrid` 鏄犲皠涓?

```text
unknown: value < 0
free: 0 <= value <= free_threshold
obstacle: value >= obstacle_threshold
```

榛樿鍙傛暟:

```text
free_threshold: 20
obstacle_threshold: 65
```

鍚庣画鎺?`GridMap` 鏃? 鍐嶄粠 elevation, traversability, variance, is_valid 绛?layer 鐢熸垚鍚屾牱鐨勪笁鍊?grid

### SDF 鍜屽崐寰?
瀵?obstacle 鍜?unknown 鍒嗗埆璁＄畻璺濈鍦?

```text
SDF_obs(p) = distance from p to nearest obstacle
SDF_unk(p) = distance from p to nearest unknown
```

鑺傜偣鍗婂緞:

```text
free_radius = min(SDF_obs(node), SDF_unk(node), max_free_radius)
explored_radius = max(previous_explored_radius, SDF_unk(node))
```

瑙ｉ噴:

- `free_radius` 琛ㄧず鑺傜偣闄勮繎鏈夊澶ц寖鍥存槸瀹夊叏宸茬煡鍖哄煙
- `explored_radius` 琛ㄧず鑺傜偣闄勮繎鏈夊澶ц寖鍥村凡缁忚瑙傚療杩?- `explored_radius` 鏄璺洖閫€鍜岄伩鍏嶉噸澶嶆帰绱㈢殑鍏抽敭

### 鑺傜偣閲囨牱

绗竴鐗堜娇鐢ㄨ鍒欓噰鏍? 涓嶆槸闅忔満閲囨牱:

```text
for each free cell with stride sample_stride:
    if distance to nearest existing node > min_node_separation:
        if SDF_obs(cell) > min_obstacle_clearance:
            create node
```

鐞嗙敱:

- 杈撳嚭绋冲畾
- 鏇村鏄撹皟璇?- UUID 鏇村鏄撲繚鎸?
鍚庣画鍙互鍔犲叆璁烘枃涓殑 `N_samples = 1000` 闅忔満閲囨牱妯″紡

### frontier 妫€娴?
frontier cell 瀹氫箟:

```text
frontier_cell = free cell with at least one unknown neighbor
```

浼唬鐮?

```text
for each free cell:
    if any 8-connected neighbor is unknown:
        add to frontier_cells
```

### frontier 鍒嗛厤

鎶婂瘑闆?frontier cells 鍒嗛厤缁欓檮杩?graph node:

```text
for each frontier_cell:
    if inside explored_radius of any node:
        continue

    owner = nearest node within frontier_assign_radius

    if owner exists and line from owner to frontier_cell is collision-free:
        owner.frontier_points.append(frontier_cell_position)
        owner.is_frontier = true
```

闇€瑕佷繚璇?

```text
frontier node 鍦?free 鍖哄煙
frontier_points 鎸囧悜 unknown 杈圭晫
mean(frontier_points) - node.position 鑳借〃绀?frontier heading
```

杩欐槸鎺ュ叆 WildOS 瑙嗚璇勫垎鐨勫叧閿?
### 寤鸿竟

绗竴鐗堜娇鐢ㄧ┖闂磋繎閭诲拰鐩寸嚎纰版挒妫€娴?

```text
for each node_i:
    neighbors = nodes within edge_radius

    for each node_j in neighbors:
        if BresenhamLine(node_i, node_j) has no obstacle:
            add edge
```

杈规潈閲嶇涓€鐗?

```text
traversability_cost = EuclideanDistance(node_i, node_j)
```

鍚庣画鍗囩骇:

```text
traversability_cost = distance + obstacle_clearance_penalty + unknown_penalty + slope_penalty
```

### current_node_idx

褰撳墠鑺傜偣閫夋嫨:

```text
current_node = nearest node to odom pose among reachable free nodes
```

濡傛灉鏈€杩戣妭鐐逛笉鍙揪, fallback:

```text
current_node = nearest node with collision-free line to robot pose
```

## 鎺ュ叆 WildOS 璇勫垎

Graph Construction 涓嶇洿鎺ュ啓 `frontier_scores`

瀹冨彂甯?

```text
NavigationGraph.trav_classes = ["default"]
Node.trav_properties[0].is_frontier
Node.trav_properties[0].frontier_points
Node.trav_properties[0].free_radius
Node.trav_properties[0].explored_radius
Edge.traversability[0].traversability_cost
```

WildOS 瑙嗚妯″潡浼氳鍙?frontier nodes, 鎶曞奖鍒板浘鍍? 骞惰拷鍔?

```text
node.properties:
    key = frontier_scores
    value = [score_bin_0, ..., score_bin_15]
```

鍥犳瀹炵幇閲嶇偣鏄?

```text
frontier geometry 姝ｇ‘
node uuid 绋冲畾
graph frame 鍜?odom frame 涓€鑷?frontier_points 闈炵┖
```

## 鎺ュ叆璺緞绠楁硶

绗竴鐗堝鐢ㄥ凡鏈?`graphnav_planner`

宸叉湁 planner 閫昏緫:

```text
scored_nav_graph -> internal weighted graph
frontier_scores -> virtual goal edge cost
Dijkstra -> high level path
```

Graph Construction 鍙渶瑕佷繚璇佸浘婊¤冻 planner 鐨勮緭鍏ュ绾?
## 姝昏矾鍥為€€

姝昏矾鍥為€€涓嶅崟鐙啓涓€涓帶鍒跺櫒, 鍏堥€氳繃 graph memory 鏀寔

鍩虹瑙勫垯:

```text
濡傛灉 frontier 闄勮繎 unknown 琚?explored_radius 瑕嗙洊, 绉婚櫎 frontier
濡傛灉 frontier 杩炵画 N 娆℃病鏈夊甫鏉ユ柊澧?unknown 杈圭晫, 闄嶄綆浼樺厛绾?濡傛灉褰撳墠 path 鐨勬湯绔?frontier 娑堝け, 閲嶆柊鍙戝竷 graph
濡傛灉娌℃湁瑙嗚璇勫垎, planner fallback 鍒板嚑浣?frontier distance
```

鍐呴儴璁板綍:

```text
removed_frontiers
low_value_frontiers
frontier_failed_count
```

绗竴鐗堜笉鏀?`graphnav_planner`, 鎵€浠ラ檷鏉冨彲浠ュ厛浣撶幇鍦?Graph Construction 杈撳嚭涓?

```text
绉婚櫎澶辨晥 frontier
鍑忓皯 frontier_points
鎴栧皢 frontier 鏍囪涓洪潪 frontier
```

鍚庣画濡傛灉闇€瑕佹洿寮烘帶鍒? 鍐嶈€冭檻缁?node.properties 鍔?

```text
key = graph_construction_penalty
value = [penalty]
```

浣嗙涓€鐗堜笉渚濊禆杩欎釜瀛楁

## 瑙嗚杈圭晫鑺傜偣鎺㈢储

瑙嗚杈圭晫鎺㈢储渚濊禆涓ゅ眰 frontier:

```text
geometric frontier: Graph Construction 鐢熸垚
visual frontier: ExploRFM 浠?RGB 棰勬祴
```

宸ヤ綔鏂瑰紡:

```text
Graph Construction 鐢熸垚 F_geo
WildOS 灏?F_geo 鎶曞奖鍒?camera image
ExploRFM 杈撳嚭 F_vis 鍜?T_vis
Scoring 璁＄畻姣忎釜 F_geo 鍦ㄤ笉鍚?heading bin 鐨勫垎鏁?Planner 閫夋嫨楂樺垎 frontier
```

鍥犳绗竴鐗?Graph Construction 瑕佷紭鍏堜繚璇佸嚑浣?frontier 绋冲畾, 涓嶉渶瑕佽嚜宸卞疄鐜拌瑙?frontier

## 鍙鍖?
鏂板 `graph_construction/viz.py`, 鍙戝竷 RViz markers:

```text
free nodes: green sphere
frontier nodes: blue sphere
frontier_points: purple cube
edges: red line
current node: yellow sphere
free_radius: red transparent disk
explored_radius: cyan transparent disk
removed frontier: gray marker
```

宸叉湁 WildOS 鍙鍖栫户缁娇鐢?

```text
/spot1/nav_graph_viz
/spot1/score_rings
/spot1/model_visualization
```

## 閰嶇疆鏂囦欢

`configs/graph_construction.yaml`:

```yaml
robot_namespace: spot1
global_frame: spot1/odom
odom_topic: /spot1/odom
grid_topic: /spot1/traversability_grid
nav_graph_topic: /spot1/nav_graph
viz_topic: /spot1/graph_construction_viz
debug_grid_topic: /spot1/debug_traversability_grid

trav_class: default
map_resolution: 0.1
local_map_radius: 10.0

free_threshold: 20
obstacle_threshold: 65
sample_stride: 8
min_node_separation: 1.0
max_free_radius: 4.0
min_obstacle_clearance: 0.5
edge_radius: 8.0
frontier_assign_radius: 5.0
frontier_min_points: 2

deadend_observation_count: 3
publish_rate_hz: 2.0
```

## 瀹炵幇闃舵

### 闃舵 1, 鍖呭拰娑堟伅闂幆

鐩爣:

```text
graph_construction 鍖呭彲 build
node 鍙惎鍔?鍙戝竷涓€涓渶灏?NavigationGraph
RViz 鍙湅鍒拌妭鐐瑰拰杈?```

### 闃舵 2, OccupancyGrid 鍒?graph

鐩爣:

```text
璁㈤槄 OccupancyGrid
鐢熸垚 free nodes
鐢熸垚 frontier nodes
鐢熸垚 edges
鍙戝竷 /spot1/nav_graph
```

### 闃舵 3, 鎺ュ叆 WildOS scoring

鐩爣:

```text
WildOS 璁㈤槄 /spot1/nav_graph
鍙戝竷 /spot1/scored_nav_graph
frontier_scores 鍐欏叆 node.properties
score_rings 姝ｅ父鏄剧ず
```

### 闃舵 4, 鎺ュ叆 planner

鐩爣:

```text
graphnav_planner 璁㈤槄 /spot1/scored_nav_graph
鑳藉杈撳嚭 graph path
current_node_idx 姝ｇ‘
frontier 鍙樺寲鍚庤兘閲嶆柊瑙勫垝
```

### 闃舵 5, 姝昏矾鍥為€€

鐩爣:

```text
澶辨晥 frontier 琚Щ闄?鍘嗗彶 free nodes 淇濈暀
璧拌繘姝昏矾鍚庤兘閫夋嫨鍏跺畠 frontier
```

### 闃舵 6, GridMap / elevation map 鎺ュ叆

鐩爣:

```text
浠?elevation_mapping_cupy 鎴?grid_map_msgs/GridMap 璇诲彇鐪熷疄鍑犱綍 layer
鏇挎崲 OccupancyGrid adapter
淇濈暀 GraphBuilder 涓婚€昏緫涓嶅彉
```

## 楠岃瘉鏂瑰紡

鍩虹鍛戒护:

```text
colcon build --packages-select graph_construction graphnav_msgs graphnav_planner visual_navigation
ros2 launch graph_construction graph_construction.launch.py ns:=spot1
```

妫€鏌?topic:

```text
ros2 topic echo /spot1/nav_graph --once
ros2 topic echo /spot1/scored_nav_graph --once
ros2 topic list | grep graph
```

RViz 妫€鏌?

```text
/spot1/graph_construction_viz
/spot1/nav_graph_viz
/spot1/score_rings
/spot1/graphnav_planner/path
```

鍏抽敭楠屾敹:

```text
NavigationGraph 闈炵┖
trav_classes 鍖呭惈 default
current_node_idx 鍦?nodes 鑼冨洿鍐?frontier nodes 鏈?frontier_points
edges 鐨?from_idx 鍜?to_idx 鍚堟硶
planner 鑳借緭鍑?path
```

## 椋庨櫓鍜屽緟纭

闇€瑕佺‘璁ょ湡瀹?`T_geo` 鏉ユ簮:

```text
OccupancyGrid topic
GridMap topic
elevation_mapping_cupy layer names
frame_id 鍜?odom frame
```

闇€瑕佺‘璁?robot footprint:

```text
鍗婂緞鎴?footprint polygon
inflation radius
鏈€灏忛殰纰嶈窛绂?```

闇€瑕佺‘璁よ繍琛岀幆澧?

```text
ROS 2 Humble
Python package dependencies
grid_map_msgs 鏄惁鍙敤
scipy 鏄惁鍙敤, 鐢ㄤ簬 distance transform
```

濡傛灉 `scipy` 涓嶅彲鐢? 绗竴鐗堝彲浠ョ敤 OpenCV distance transform 鎴栫函 numpy BFS 鍏滃簳

## 绗竴鐗堜氦浠樿寖鍥?
绗竴鐗堝疄鐜板畬鎴愬悗, 搴旇鍏峰:

```text
涓€涓彲鍚姩鐨?graph_construction ROS2 package
鍙粠 OccupancyGrid 鐢熸垚 NavigationGraph
鍙鍖?free nodes, frontier nodes, edges, radius
鍙帴鍏?WildOS scoring
鍙 graphnav_planner 鍩轰簬 scored graph 瑙勫垝
鍏峰鍩虹姝昏矾 frontier 绉婚櫎鑳藉姏
```

绗竴鐗堜笉鎵胯:

```text
瀹屽叏澶嶇幇 NASA/JPL 鏈紑婧愬疄鐜?瀹屽叏澶嶇幇 elevation traversability filter
澶嶆潅鍦板舰 slope / roughness cost
鐙珛 Web UI
```
