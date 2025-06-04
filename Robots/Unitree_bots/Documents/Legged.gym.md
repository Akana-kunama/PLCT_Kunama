# legged_gym

原库 ： legged gym

Repo：https://github.com/leggedrobotics/legged_gym

> This repository provides the environment used to train ANYmal (and other robots) to walk on rough terrain using NVIDIA's Isaac Gym. It includes all components needed for sim-to-real transfer: actuator  network, friction & mass randomization, noisy observations and  random pushes during training.

## Discription

Reference: https://blog.csdn.net/Guanhj_BIT_1995/article/details/145454503

Reference：https://github.com/Prcheems/Modify_the_terrain_in_Isaac_Gym

### scripts

#### `train.py`

接收终端启动程序时给定的参数``args`，返回环境`env`以及环境初始化参数`env_cfg`

重要参数：

- `--task` 任务名称
- `--resume` 恢复训练的 checkpoint
- `--experiment_name` 运行/加载的 experiment 名称
- `--checkpoint` 保存模型的checkpoint编号，设置为 -1加载最新一次文件
- `--num_envs` 并行训练的环境个数
- `--seed` 随机种子
- `--max_iterations` 训练的最大迭代次数

+ `--rl_device`: 强化学习计算设备，默认 `cuda0`

#### `play.py`

用于 **加载已经训练好的 PPO 策略**，并在 Isaac Gym 中对指定的机器人进行 **仿真推理**

说明：

+ **EXPORT_POLICY**
  - `True`：每次运行脚本时，都会把当前加载的训练模型导出成 Torch JIT 文件。
  - `False`：跳过导出步骤，只进行纯粹的推理。
+ **RECORD_FRAMES**
  - `True`：在推理过程中把每一帧画面都保存下来（通常用于后期生成视频）。
  - `False`：不保存帧，节省存储和性能开销。
+ **MOVE_CAMERA**
  - `True`：在仿真里通过某种方式让摄像机动画移动（便于调试观察多角度）。
  - `False`：保持相机静止不动，省去额外的相机控制逻辑。

### utils

#### `task_registry.py`

定义了一个全局的 `TaskRegistry` 类，用于在 Legged Gym（基于 Isaac Gym 的强化学习环境）中注册、管理和实例化各种“任务”（task）。

`TaskRegistry`

最重要的类，在模块底部以单例形式暴露给全局使用

```python
# 脚本最后：创建全局唯一的注册表对象
task_registry = TaskRegistry()
```

+ `register(self, name, task_class, env_cfg, train_cfg)`

  把一个“任务”注册到注册表里，让后续通过名称 `"name"` 就能找到对应的环境类与配置信息。

  **参数**：

  - `name: str`：任务名称（如 `"Go1"`, `"Go2"`, `"Minitaur"` 等）。
  - `task_class: VecEnv`：对应任务的环境类，这个类必须继承自 `rsl_rl.env.VecEnv`。
  - `env_cfg: LeggedRobotCfg`：该任务对应的环境配置对象（通常从 yaml 里解析得到），类型为 `LeggedRobotCfg`。
  - `train_cfg: LeggedRobotCfgPPO`：该任务对应的训练（PPO）超参数配置对象，类型为 `LeggedRobotCfgPPO`。

+ `make_env(self, name, args=None, env_cfg=None) -> Tuple[VecEnv, LeggedRobotCfg]`

  根据注册的任务名称（或直接传入一个 `env_cfg` 对象），创建并返回一个并行仿真环境实例（`VecEnv`），以及最终使用的环境配置对象（`LeggedRobotCfg`）。

  **参数**：

  - `name: str`
    - 注册时使用的任务名称，必须已经调用过 `register(name, ...)`。
  - `args: Optional[Args]`
    - 默认值 `None`。如果为 `None`，内部会自动调用 `get_args()` 从命令行读取 Isaac Gym 相关的参数（如 `physics_engine`、`sim_device`、`headless` 等）。
    - `get_args()` 通常至少会返回：
      - `args.physics_engine`（物理引擎类型，如 `"physx"`）
      - `args.sim_device`（运行仿真的设备，如 `"cuda:0"`）
      - `args.headless`（是否关闭图形渲染）
      - 以及其他可能影响环境实例化的参数。
  - `env_cfg: Optional[LeggedRobotCfg]`
    - 默认值 `None`。如果为 `None`，方法会先调用 `get_cfgs(name)` 从内部注册表加载一份 `env_cfg`。
    - 如果外部直接传入 `env_cfg`，则不会使用注册表中保存的那一份，而是用传入的。

  **返回**：

  - `env`：一个具体的 `VecEnv` 并行环境实例，可以直接调用 `env.reset()`, `env.step(action)` 等接口。
  - `env_cfg`：最终生效的 `LeggedRobotCfg` 对象（其中可能已经被 `update_cfg_from_args` 覆盖过）。

+ `make_alg_runner(self, env, name=None, args=None, train_cfg=None, log_root="default") -> Tuple[OnPolicyRunner, LeggedRobotCfgPPO]`

  创建并返回一个 **PPO 算法 Runner**（`OnPolicyRunner`），该 Runner 包含 Actor-Critic 网络、优化器、日志管理，以及可用于“训练”或“推理”的接口。同时返回最终使用的训练配置(`LeggedRobotCfgPPO`)对象 `train_cfg`。

  **参数**：

  - `env: VecEnv`

    - 已经创建好的并行环境实例（由 `make_env` 返回）。

  - `name: Optional[str]`

    - 任务名称（与 `register` 时相同）。如果 `train_cfg` 传入了，就会忽略 `name`；否则会根据 `name` 去注册表里找对应的默认训练配置。

  - `args: Optional[Args]`

    - 如果为 `None`，会自动调用 `get_args()`，从命令行里读取训练相关（PPO 算法）参数，比如学习率、批大小、梯度累积、resume、checkpoint 路径等等。

  - `train_cfg: Optional[LeggedRobotCfgPPO]`

    - 如果外部传了一个训练配置对象，就直接使用它；否则，必须传入 `name`，脚本会通过 `_, train_cfg = self.get_cfgs(name)` 从注册表中加载默认的训练配置。

  - `log_root: str`

    - 日志目录的根路径。如果为 `"default"`，内部会构造成：

      ```
      <LEGGED_GYM_ROOT_DIR>/logs/<experiment_name>/<当前时间>_<run_name>/
      ```

      其中 `<experiment_name>` 和 `<run_name>` 都来自 `train_cfg.runner.experiment_name` 和 `train_cfg.runner.run_name`。

    + 如果显式传 `None`，则不生成日志目录；否则默认用 `"default"`。

  **返回**：

  - `runner: OnPolicyRunner`
    - 这个对象封装了 PPO 训练／推理的所有逻辑，包括网络创建、优化器初始化、数据采集、TensorBoard 日志等。它还实现了：
      - `runner.train()` 用于执行训练。
      - `runner.load(path)` 用于从某个 checkpoint 恢复模型权重。
      - `runner.get_inference_policy(device)` 用于获取一个仅做前向推理的 Policy 网络。
  - `train_cfg: LeggedRobotCfgPPO`
    - 可能已经被 `update_cfg_from_args(None, train_cfg, args)` 覆盖过：
      - 例如如果你通过命令行 `--cfg_train=xxx.yaml` 指定了一个新的训练配置，原有的 `train_cfg` 会合并来自命令行的新配置。

#### `terrain.py`

定义了一个 `Terrain` 类，用于根据配置（`cfg: LeggedRobotCfg.terrain`）动态生成一张高度场（heightfield），并在必要时将其转成用于 Isaac Gym 的三角网格（trimesh）。这个高度场会被用来搭建强化学习环境中的地形，让机器人在不同难度或随机组合的地面上行走和训练。

 `Terrain`

+ `__init__(self, cfg: LeggedRobotCfg.terrain, num_robots) -> None`
  + `cfg: LeggedRobotCfg.terrain`
     这是一个由 `LeggedRobotCfg` 中 `terrain` 子配置（`dataclass`）构成的对象
  + `num_robots: int`
     代表环境里并行训练的机器人实例数量

+ 重要属性：

  `self.type`

  若其为 `"none"` 或 `"plane"`，则直接跳过后续地形生成逻辑（只有一个平面或无地形）。

  如果不是平面，就近一步设置地形属性并调用函数生成仿真地形

  + `randomized_terrain(self)`

    对每个子地形（共 `num_sub_terrains` 块）随机生成一种地形，难度与类型都按概率分布

  + `curiculum(self)`

    课程化顺序地分配难度：一般按行或列线性增大难度，方便做“从易到难”的训练。

  + `selected_terrain(self)`

    当用户在配置里指定了某种固定地形函数（存储在 `cfg.terrain_kwargs['type']`），就不随机了，由用户自己在 `terrain_kwargs` 中给出所有参数。

+ `make_terrain(choice, difficulty)`

  返回一个单独的 `SubTerrain` 对象（大小为 `width_per_env_pixels × length_per_env_pixels`），根据 `choice` 决定地形类型，并结合 `difficulty` 控制粗细参数。

  + **准备不同地形的参数**

    - `slope`＝最大 0.4（对应最难）到 0（最平），会传给 `pyramid_sloped_terrain`。

    - `step_height`＝最少 0.05m、最多约 0.23m（取决难度），用于台阶高度。

    - `discrete_obstacles_height`＝最少 0.05m、最多 0.25m，用于随机障碍物。

    - `stepping_stones_size`＝石块尺寸，随着难度变化（难度越大，石块越小），影响踏脚石的稀疏程度。

    - `gap_size`＝缺口大小，越高难度越大；

    - `pit_depth`＝陷阱深度，最大约1m。

  + **根据 `choice` 落在哪个比例区间，就调用不同 terrain_utils 函数**

    `terrain_utils` 函数在 `Issacgym` 包中

    + `pyramid_sloped_terrain` 生成一个正／反斜坡平面
    + `random_uniform_terrain`随机小起伏
    + `pyramid_stairs_terrain`生成台阶 
    + `discrete_obstacles_terrain` 离散障碍

+ `add_terrain_to_map(self, terrain, row, col)`

  将子地形贴到整体高度场， `row` 和 `col` 用来指定 **当前子地形（SubTerrain）在整体高度场中的“行索引”和“列索引”**

### envs

#### `base/base_config.py`

`BaseConfig` 类

当定义了一个配置类（如前面看到的 `LeggedRobotCfg` 或 `LeggedRobotCfgPPO`）时，自动 **递归地实例化** 其内部所有作为嵌套子类定义的“模块”或“子配置”。在使用 `LeggedRobotCfg()` 时，就不需要手动去实例化每一个嵌套的子类，所有一二级、三层……的配置信息会被自动创建为实例

#### `base/legged_robot_config.py`

用来 **集中、层级化地定义强化学习环境和 PPO 算法所需的所有参数**。意味着当执行  `cfg = LeggedRobotCfg()` / `cfg = LeggedRobotCfgPPO()`时， `BaseConfig.__init__` 会被运行，内部会调用 `init_member_classes(self)`，从而将 `LeggedRobotCfg` / `LeggedRobotCfgPPO` 类体里定义的所有嵌套“类”都依次实例化，并且不断递归下去，直至最底层没有更多子类出现为止。它通过继承 `BaseConfig`，将各种配置拆分成多个模块（子类），并在实例化时自动把这些子类一层层展开成实例。

+ `LeggedRobotCfg(BaseConfig)`

  负责环境（Environment）和机器人本身所需的所有“静态”或“默认”参数，分为以下几个子模块

  + **`env`**：并行环境的整体设置

    - `num_envs=4096` – 并行跑多少个机器人实例

    - `num_observations=48`、`num_actions=12` – 观测维度和动作维度

    - `episode_length_s=20` – 一个 episode 最多跑 20 秒

    - `test=False` – 推理时会把这个标记设置为 `True`，跳过某些随机化

  + **`terrain`**：地形生成与属性

    - `mesh_type='trimesh'` – 地形类型（可选 `none`、`plane`、`heightfield`、`trimesh`）

    - `horizontal_scale=0.1`, `vertical_scale=0.005` – 高度场的横向/纵向缩放（米/格）

    - `border_size=25` – 四周空白边框宽度（米），避免机器人一开始就掉出边缘

    - `curriculum=True` – 是否使用课程化方案，从最简单到最难逐行生成子地形

    - `num_rows=20`, `num_cols=20` – 把总地形拆成 20×20 共 400 块子地形

    - `terrain_proportions=[0.1, 0.1, 0.35, 0.25, 0.2]` – 各种子地形出现的概率（平滑斜坡、粗糙斜坡、上台阶、下台阶、离散障碍）

    - `measure_heights=True` + `measured_points_x/y` – 机器人脚底测量该片地形时需要采样的网格点位置

    - `slope_treshold=0.75` – 当用 `trimesh` 模式生成三角网格时，如果某格坡度 > 0.75，就把它变成“竖直面”以保证物理碰撞稳定

  + **`commands`**：对机器人下发“高层命令”时的配置

    - `curriculum=True`, `max_curriculum=1.` – 命令空间也做课程化，最开始只给很小的速度范围，后续逐步放开
    - `num_commands=4` – 默认有 4 个命令维度（`lin_vel_x`、`lin_vel_y`、`ang_vel_yaw`、`heading`）
    - `resampling_time=10.` – 每 10 秒重新采样一次新命令
    - 嵌套 **`ranges`** 指定每条命令的最小/最大取值范围

  + **`init_state`**：机器人每个 episode 开始时的初始化状态

    - `pos=[0,0,1]`, `rot=[0,0,0,1]` – 在世界坐标把机器人抬到 z=1 米，保持水平姿态
    - `lin_vel=[0,0,0]`, `ang_vel=[0,0,0]` – 线速度、角速度全零
    - `default_joint_angles` – 当动作输出 “0.0” 时，关节要保持哪个默认角度

  + **`control`**：关节控制方式与 PD 参数

    - `control_type='P'` – 位置控制；可选 `'V'`（速度） 或 `'T'`（扭矩）
    - `stiffness={'joint_a':10, 'joint_b':15}`, `damping={'joint_a':1, 'joint_b':1.5}` – 如果是 P/PD 控制，需要给出关节刚度和阻尼放缩系数
    - `action_scale=0.5` – 当策略输出 `action` 时，执行给定关节目标 = `default_angle + action_scale * action`
    - `decimation=4` – 策略输出一次后，物理仿真里要执行 4 个时间步才更新一次动作

  + **`asset`**：机器人 URDF/视觉/碰撞 及物理属性

    - `file=""` – URDF 或模型文件路径（可在外部动态覆盖）
    - `foot_name="None"` – 脚底刚体名称，用于获取机器人与地面的接触信息
    - `penalize_contacts_on=[]`, `terminate_after_contacts_on=[]` – 如果碰触到指定的 link，就给惩罚或直接终止该 episode
    - `collapse_fixed_joints=True` – 把所有固定关节在 PhysX 中合并为一个刚体，以减少物理计算量
    - `replace_cylinder_with_capsule=True` – 把 URDF 中的圆柱碰撞体换成胶囊碰撞体，提高仿真稳定性和效率
    - 其他诸如 `density`, `angular_damping`, `linear_damping`, `max_angular_velocity`, `max_linear_velocity` 等，都是物理属性的默认值

  + **`domain_rand`**：域随机化（Domain Randomization）相关设置

    - `randomize_friction=True`, `friction_range=[0.5,1.25]` – 摩擦系数在 [0.5, 1.25] 之间随机
    - `randomize_base_mass=False`, `added_mass_range=[-1.,1.]` – 机器人底盘质量扰动范围
    - `push_robots=True`, `push_interval_s=15`, `max_push_vel_xy=1.` – 每 15 秒给机器人随机一次外力冲击，最大能让机器人获得 1 m/s 的额外线速度

  + **`rewards`**：奖励函数里各项的放缩系数与阈值

    - 嵌套 **`scales`**：把所有常见奖励项（`tracking_lin_vel`, `tracking_ang_vel`, `lin_vel_z`, `ang_vel_xy`, `torques`, `collision` 等等）给出一个默认系数
    - `only_positive_rewards=True` – 如果总奖励为负，直接当 0 处理，避免训练早期一堆负分带来提前终止问题
    - `tracking_sigma=0.25` – 用来计算 `exp(-error^2/σ)` 的 σ，给“跟踪目标速度”这项奖励做高斯核函数里的分母
    - `soft_dof_pos_limit=1.`, `soft_dof_vel_limit=1.`, `soft_torque_limit=1.` – 当关节位置/速度/力矩超过“URDF 最大值 × 1 倍” 开始惩罚
    - `base_height_target=1.` – 希望机器人基座保持 1 m，高度偏离给惩罚
    - `max_contact_force=100.` – 如果单次接触力超过 100 牛，就算惩罚

  + **`normalization`**：观测与动作的归一化系数

    - 嵌套 **`obs_scales`** – 分别对线速度、角速度、关节位置、关节速度、地形高度测量值给出尺度因子
    - `clip_observations=100.`, `clip_actions=100.` – 如果归一化后数值超过 ±100 就做裁剪

  + **`noise`**：给观测加噪相关配置

    - `add_noise=True`, `noise_level=1.0` – 是否加噪、噪声总体强度
    - 嵌套 **`noise_scales`** – 对 `dof_pos`, `dof_vel`, `lin_vel`, `ang_vel`, `gravity`, `height_measurements` 这几类观测分别给出对应的标准差比例

  + **`viewer`**：渲染时摄像机视角设置

    - `ref_env=0` – 以第 0 个并行环境为参考
    - `pos=[10,0,6]`, `lookat=[11,5,3]` – 摄像机在世界坐标系下的位置和目标朝向

  + **`sim`**：物理仿真(PhysX)相关基础参数

    - `dt=0.005`, `substeps=1`, `gravity=[0,0,-9.81]`, `up_axis=1` – 仿真步长 5 ms、一个子步、重力方向为 z 轴向下
    - 嵌套 **`physx`** – PhysX 求解器细节：线程数、解算器类型、位置/速度迭代次数、碰撞偏差、弹性碰撞阈值、最大分离速度、GPU 接触对数上限、缓冲区倍增因子、碰撞采集模式

+ `LeggedRobotCfgPPO`

  负责 PPO 算法训练与推理的超参数

  + `seed=1` – 随机种子，用于初始化 NumPy、PyTorch、物理仿真等
  + `runner_class_name='OnPolicyRunner'` – 指定用哪个 Runner 类来跑 PPO（通常是 `OnPolicyRunner`）

  + **`policy`**：Actor-Critic 网络架构

    - `init_noise_std=1.0` – 策略输出时的初始高斯噪声标准差

    - `actor_hidden_dims=[512,256,128]`, `critic_hidden_dims=[512,256,128]` – Actor/Critic 网络各层隐藏单元数

    - `activation='elu'` – 激活函数（可换成 `relu`、`tanh` 等）

  + **`algorithm`**：PPO 算法关键超参

    - `value_loss_coef=1.0`, `use_clipped_value_loss=True` – Critic 损失权重与是否使用“截断格式”价值损失

    - `clip_param=0.2` – PPO 核心截断系数 ε≈0.2

    - `entropy_coef=0.01` – 熵正则化系数，鼓励探索

    - `num_learning_epochs=5`, `num_mini_batches=4` – 每轮采集数据后要做多少轮梯度更新，以及分成多少个 mini-batch

    - `learning_rate=1e-3`, `schedule='adaptive'` – 初始学习率 0.001，以及使用 KL 自适应衰减策略

    - `gamma=0.99`, `lam=0.95` – 折扣因子 γ 和 GAE λ

    - `desired_kl=0.01`, `max_grad_norm=1.0` – 当实际 KL 超过 0.01 时提前停止当前更新；梯度裁剪范数阈值 1.0

  + **`runner`**：训练循环控制与日志/断点恢复配置

    - `policy_class_name='ActorCritic'`, `algorithm_class_name='PPO'` – 实际载入的类名

    - `num_steps_per_env=24` – 每个环境采集一次数据时跑 24 步

    - `max_iterations=1500` – 最多做 1500 次策略更新

    - `save_interval=50`, `experiment_name='test'`, `run_name=''` – 日志保存间隔和实验/运行名

    - `resume=False`, `load_run=-1`, `checkpoint=-1`, `resume_path=None` – 用于从已有 checkpoint 恢复训练或直接加载做推理

#### `base/base_task.py`

`BaseTask`

一个抽象的 “RL 任务基类”，封装了与 Isaac Gym 仿真环境交互所需的通用功能

+ 构造函数 `__init__` 

  + `self.gym = gymapi.acquire_gym()` 

    获得 Isaac Gym 的接口句柄

  + 读取用户传入的物理引擎类型 (`physics_engine`)、仿真设备 (`sim_device`) 以及是否无头模式 (`headless`) 等信息，并据此确定后续“在 GPU 上跑仿真”还是“在 CPU 上跑”。

  + 根据 `cfg.env.num_envs`、`cfg.env.num_observations`、`cfg.env.num_privileged_obs`、`cfg.env.num_actions` 等字段，初始化好若干个用于存放观测、奖励、重置标志、时长计数、超时标志的 PyTorch 张量缓冲区（`obs_buf`, `rew_buf`, `reset_buf`, `episode_length_buf`, `time_out_buf`，以及可选的 `privileged_obs_buf`）。这些张量大小都是 `(num_envs, …)`，方便并行地批量收集多个环境/机器人实例的数值。

  + `self.create_sim()` 

    调用 Isaac Gym API 去“真正构建”仿真中的世界：创建并行环境、加载地形、加载机器人 URDF 资产、设置碰撞检测参数、初始化各种刚体等。

  + `self.gym.prepare_sim(self.sim)`

    在底层把这些设置传给物理引擎，准备好进入仿真循环。

  + 并行数据缓冲区

    + `self.obs_buf`: 存放 `num_envs × num_observations` 大小的浮点观测。

    + `self.rew_buf`: 每个环境当前步的标量奖励。

    + `self.reset_buf`: 布尔张量，标记“哪个环境需要重置”（1 表示需要）。

    + `self.episode_length_buf`: 记录每个环境当前 episode 已经跑了多少步。

    + `self.time_out_buf`: 记录哪些环境到达最大步数而“超时”（True 表示已经到时）。

    + 如果 `cfg.env.num_privileged_obs` 不为 `None`，则还会创建 `self.privileged_obs_buf` 用于“非对称训练”（Critic 能见到更多观测）。否则该变量置为 `None`。

    这些缓冲区是整个 RL 循环中收集样本、计算奖励和判断重置时刻的核心数据结构。它们都会被后续调用 `step()` 时填充，然后被 PPO Runner 取过来进行 Tensor 计算。

+ `reset_idx(self, env_ids)`

  是一个抽象方法（`raise NotImplementedError`），子类必须实现。它的语义是：**对给定索引集合 `env_ids`（形如一个长度为 k 的 LongTensor，里面包含若干环境 id）进行“局部重置”**

+ `reset(self)`

  先把所有环境都重置 (`reset_idx([0, 1, …, num_envs-1])`)，然后立即调用一次 `step`（给一个全零动作），并把 `step` 返回的观测传给调用者。这样上层在开始新 episode 时，只需调用 `reset()` 即可拿到“初始观测”。

+ `step(self, actions)`

  同样是一个抽象方法，子类需要实现：**给定一个形如 `(num_envs, num_actions)` 的动作张量，推进仿真一步**，并返回五元组

  ```
  (obs, privileged_obs, rewards, resets, extras)
  ```

  + `obs`: 大小 `(num_envs, num_observations)` 的 Tensor，包含当前所有环境在新一步的观测。

  + `privileged_obs`: 大小 `(num_envs, num_privileged_obs)` 或 `None`，只在“非对称训练”时使用。

  + `rewards`: `(num_envs,)`，标量奖励。

  + `resets`: `(num_envs,)` 的布尔 Tensor，标记哪些环境在这一步结束时需要重置。

  + `extras`: 一个字典，子类可以任选地把额外信息（如 骨架高度、速度统计、接触点数量 等）塞进去供调试或日志使用。
     具体实现时，一般会调用 Isaac Gym 的 `apply_action_tensor()`、`step_simulation()`、`fetch_rigid_body_states()`、`compute_observations()`、`compute_rewards()`、根据 `reset_buf` 调用 `reset_idx()` 等一系列操作，最后填充上述五个输出。

#### `base/legged_robot.py`

`LeggedRobot`

一个具体的“腿式机器人任务”类，继承自 `BaseTask` ，把“机器人环境、仿真、观测/动作缓冲、奖励计算、地形加载、机器人加载”等逻辑全都串联起来，最终可以用来训练或推理一个多并行环境下的腿式机器人。

+ 构造函数`__init__`

  + 保存传入的 `cfg: LeggedRobotCfg`、`sim_params`、`physics_engine`、`sim_device`、`headless`。

  + 调用内部 `_parse_cfg()`，把一些常用配置（如 `dt`、`obs_scales`、`reward_scales`、`command_ranges`、`max_episode_length` 等）从 `cfg` 里提取并做预处理。

  + 执行 `super().__init__(…)`，触发 `BaseTask`：

    - 分配好并行张量缓冲区（`obs_buf`、`rew_buf`、`reset_buf`、`episode_length_buf`、`time_out_buf`、可选的 `privileged_obs_buf`）

    - 调用 `create_sim()`（在子类里实现）来“真正构建仿真世界”

    - 如果 `headless=False` 就创建 Viewer 并绑定 ESC/V 键

  + 在渲染窗口可用时，调用 `set_camera()` 把摄像机摆到 `cfg.viewer.pos / lookat` 指定的位置。

  + 调用 `_init_buffers()`，用 Isaac Gym 提供的 GPU 张量接口来包装 `root_states`、`dof_state`、`contact_forces`，并分解出 `base_pos`、`base_quat`、`dof_pos`、`dof_vel`、`feet_indices`、`penalised_contact_indices`、`termination_contact_indices`，同时构建好所有“后面会用到的张量”──`torques`、`p_gains`、`d_gains`、`actions`、`last_actions`、`last_dof_vel`、`last_root_vel`、`commands`、`noise_scale_vec`、`feet_air_time`、`last_contacts` 等。

  + 调用 `_prepare_reward_function()`：把 `cfg.rewards.scales` 里所有非零的奖励项（key）都对应到类里以 `_reward_<name>()` 命名的方法，并把它们装进 `self.reward_functions`、`self.reward_names`，同时按照 `dt` 乘以各自的放缩系数，初始化 `episode_sums` 用于累加每个环境中各 reward 项的总和。

  + 最后把 `self.init_done = True` 标记为“初始化已完成”，后续就可以放心在 `step()` 里调用各种缓冲和回调。

+ `step(actions)`

  + 裁剪并保存动作

    ```python
    clip_actions = cfg.normalization.clip_actions
    self.actions = torch.clip(actions, -clip_actions, clip_actions).to(self.device)
    ```

    把传进来的 `(num_envs, num_actions)` 类型动作先裁剪，再拷贝到设备上。

  + 渲染并推进物理多步（decimation）

    + 调用 `self.render()`：如果启用了 Viewer，就执行 `step_graphics()/draw_viewer()`；如果 headless，就什么也不做。

    + `for _ in range(self.cfg.control.decimation):`：例如 `decimation=4` 就让机器人每读取一次策略输出后在仿真里施加四次力/位置控制。

      ```python
      self.torques = self._compute_torques(self.actions).view(self.torques.shape)
      self.gym.set_dof_actuation_force_tensor(self.sim, gymtorch.unwrap_tensor(self.torques))
      self.gym.simulate(self.sim)
      if self.cfg.env.test:  # 推理时让物理时间和真实时间同步
          elapsed_time = self.gym.get_elapsed_time(self.sim)
          sim_time = self.gym.get_sim_time(self.sim)
          if sim_time - elapsed_time > 0:
              time.sleep(sim_time - elapsed_time)
      if self.device == 'cpu':
          self.gym.fetch_results(self.sim, True)
      self.gym.refresh_dof_state_tensor(self.sim)
      
      ```

      + `_compute_torques(actions)` 会根据 `cfg.control.control_type`（‘P’、‘V’ 或 ‘T’）用 PD 控制/纯力矩直接输出计算出 `self.torques`。

      + `simulate()` + `fetch_results()` + `refresh_dof_state_tensor()` 把刚体/关节状态拉到 CPU 张量。

+ `self.post_physics_step()`

  - 刷新 `actor_root_state_tensor`（base 位置/四元数/线速度/角速度）和 `net_contact_force_tensor`。

  - `self.episode_length_buf += 1`、`self.common_step_counter += 1`；

  - 从 `root_states` 里更新：`base_pos`、`base_quat`、`rpy`、`base_lin_vel`、`base_ang_vel`、`projected_gravity`；

  - 调用 `_post_physics_step_callback()`：

    - 这里会自动按 `cfg.commands.resampling_time`（比如 10s）决定哪些环境要“换新命令” `_resample_commands(env_ids)`。
    - 如果 `cfg.commands.heading_command=True`，则根据朝向误差自动把“航向角命令”转成合适的 `ang_vel_yaw`（yaw 速度）给到 `self.commands[:,2]`。

  - `self.check_termination()`：执行“终止判断”

    - 根据 `self.contact_forces[..., self.termination_contact_indices, :]` 判断哪些机器人脚触到被惩罚/终止接触体（>1 N 就算接触）
    - 检查 `rpy`（Roll/Pitch）是否过大，若 `|roll|>0.8` 或 `|pitch|>1.0` 则标记为终止
    - 检查 `episode_length_buf > max_episode_length`（超时）

  - `self.compute_reward()`：依次调用所有 `self.reward_functions[i]()`（例如 `_reward_tracking_lin_vel()`、`_reward_collision()`、`_reward_feet_air_time()` 等），乘以 `self.reward_scales[name]` 累加到 `self.rew_buf`，若 `only_positive_rewards=True` 就把负值截到 0；最后加上 `_reward_termination() * scale['termination']`。同时把每项奖励累加到 `self.episode_sums[name]`。

  - 找出需要重置的环境 ID：

    ```
    env_ids = self.reset_buf.nonzero(as_tuple=False).flatten()
    self.reset_idx(env_ids)
    ```

    如果 `reset_buf[i]==1`，就走到 `reset_idx([i])` 去执行“重置逻辑”：

    - 调用 `_reset_dofs(env_ids)`：随机选 DOF 位置（`0.5~1.5× default_pos`）、设关节速度为 0、再用 `set_dof_state_tensor_indexed(...)` 把这些环境的 DOF 状态直接写入仿真。
    - 调用 `_reset_root_states(env_ids)`：
      - 若 `self.custom_origins=True`（即用了地形课表，参考地面高度），先把 `root_states[env_ids]=base_init_state`，再把 `[x,y] += uniform(-1,1)` 做细微扰动，`z += env_origins[env_id,2]` 让 base 放在对应子地形高度；
      - 否则就把机器人 spawn 在平面网格 `env_origins`；
      - 把 base 的线速度/角速度随机设 `[-0.5,0.5]`，再调用 `set_actor_root_state_tensor_indexed(...)`。
    - `_resample_commands(env_ids)`：给这些新 episode 环境随机抽命令。
    - 重置 `self.actions, last_actions, last_dof_vel, feet_air_time, episode_length_buf, reset_buf`；根据 `episode_sums` 计算并把 `extras["episode"]["rew_<name>"]` 塞进去，清零 `episode_sums[...]`。如果 `cfg.commands.curriculum=True`，再把本次命令课表进度写入 `extras["episode"]["max_command_x"]`。如果 `send_timeouts=True`，把 `time_out_buf` 也放进 `extras["time_outs"]`。

+ 返还数据

  ```
  clip_obs = cfg.normalization.clip_observations
  self.obs_buf = torch.clip(self.obs_buf, -clip_obs, clip_obs)
  if self.privileged_obs_buf is not None:
      self.privileged_obs_buf = torch.clip(self.privileged_obs_buf, -clip_obs, clip_obs)
  return self.obs_buf, self.privileged_obs_buf, self.rew_buf, self.reset_buf, self.extras
  ```

  PPO Runner 拿到的就是 `(obs, privileged_obs, rewards, resets, extras)`

+ `create_sim(self)`

  + **`_create_ground_plane()`**：创建一个静态平面地面，设置摩擦/回复系数。
  + **`_create_heightfield()`**：用 `gymapi.HeightFieldParams()` 把 `self.terrain.height_field_raw`（整数高度矩阵）传给 PhysX，高度场尺寸 = `(tot_rows, tot_cols)`，横/纵缩放 = `cfg.terrain.horizontal_scale`，垂直缩放 = `cfg.terrain.vertical_scale`，并把其包装成 `self.height_samples` 方便后续可视化或调试。
  + **`_create_trimesh()`**：把 `self.terrain.vertices` + `self.terrain.triangles` 扁平化传给 PhysX，仍然把生成的 `self.terrain.height_field_raw` 存成 `height_samples`。

  + `_create_envs()`：生成环境

+ `compute_observations()`

  把一系列状态拼成 `(num_envs, num_obs)` 大小的 `self.obs_buf`

+ `compute_reward()`

  + 每一步先把 `self.rew_buf[:] = 0.`，遍历 `self.reward_functions`（从 `_prepare_reward_function()` 里收集的 `_reward_XXX` 方法列表），调用并乘以对应的 `reward_scales[XXX]` 累加到 `self.rew_buf`，同时把各项奖励累加到 `self.episode_sums[XXX]`。

  + 如果 `cfg.rewards.only_positive_rewards=True`，对 `self.rew_buf` 做 `clip(min=0.)`。

  + 最后如果 “termination” 在 `reward_scales`，再单独加上 `_reward_termination() * reward_scales["termination"]`，并把这笔奖励也加到 `episode_sums["termination"]`。

+ `_reward_XXX()`
  + `_reward_lin_vel_z()`：惩罚 Z 方向的线速度平方。
  + `_reward_ang_vel_xy()`：惩罚绕 X/Y 轴的角速度平方。
  + `_reward_orientation()`：惩罚机器人不水平（`projected_gravity[:,:2]`）。
  + `_reward_base_height()`：惩罚基座高度偏离目标 `cfg.rewards.base_height_target`。
  + `_reward_torques()`：惩罚扭矩平方。
  + `_reward_dof_vel()`：惩罚关节速度平方。
  + `_reward_dof_acc()`：惩罚关节加速度平方。
  + `_reward_action_rate()`：惩罚本步动作与上步动作差值平方。
  + `_reward_collision()`：惩罚脚接触非地面体的碰撞力（超过 0.1N 即算碰撞）。
  + `_reward_dof_pos_limits()` / `_reward_dof_vel_limits()` / `_reward_torque_limits()`：惩罚关节位置/速度/力矩超过软限阈值。
  + `_reward_tracking_lin_vel()`：用 `exp(-‖cmd_xy - base_lin_vel_xy‖² / σ)` 来鼓励机器人跟踪命令的线速度。
  + `_reward_tracking_ang_vel()`：类似地，对跟踪命令的角速度 Yaw 做高斯核。
  + `_reward_feet_air_time()`：统计“脚离地时间”，第一接触时才给奖励，越长步幅越高。
  + `_reward_stumble()`：当脚横向受力远大于垂直力（撞到墙之类）就惩罚。
  + `_reward_stand_still()`：当命令速度接近 0 却还在动时惩罚。
  + `_reward_feet_contact_forces()`：惩罚接触力超过 `cfg.rewards.max_contact_force`。
  + `_reward_termination()`：当 `reset_buf=True` 且非超时（`~time_out_buf`）时给一个终止奖励/惩罚。

+ `_push_robots()`

  若 `cfg.domain_rand.push_robots=True`，每隔 `push_interval` 步就在所有环境中随机把 `root_states[:,7:9]`（base 的 XY 速度）设为一个 `[−max_vel, +max_vel]`，模拟外力冲击。