# 中央協調把持制御とMuJoCo生成の構成

シミュレーションの起動入口は`grasp_sim.launch`です。MuJoCo、各Beeのbringup、中央制御を別々のlaunchに分けています。中央の`bee_cooperative_grasp_node`がphase管理・物体制御・協調参照生成を行い、各Beeの`BeeController`内のgrasp modeへ目標速度・加速度と接触力目標を送ります。grasp modeが自機の速度とobserverを使って追従・外力補償し、既存の飛行制御ループ内でWORLD/COGの加速度・姿勢目標を適用します。標準起動では独立したtrackerノードや自分宛ての`uav/nav`再送はありません。

```text
grasp_sim.launch
├─ mujoco_world.launch
│  └─ prepare_mujoco_scene.py → exec MuJoCo（共有の1プロセス）
├─ multi_robot_bringup.launch
│  └─ include/mujoco_vehicle.launch.xml × 台数
│     └─ bringup.launch
│        ├─ aerial_robot_base_node
│        │  └─ BeeController（通常制御／grasp mode）
│        └─ controller_spawner → attitude_controller
└─ cooperative_grasp.launch
   └─ bee_cooperative_grasp_node
```

`multi_robot_bringup.launch`はシミュレーション用です。実機では従来どおり各ロボット上で`bringup.launch`を個別に起動します。`grasp_real.launch`は作っていません。

## ファイルの入口

| 役割 | 実装 |
| --- | --- |
| シーン生成・物体／台座／初期配置 | `script/mujoco_scene_generator.py` |
| シミュレーション全体の起動 | `launch/grasp_sim.launch` |
| 共有MuJoCoの起動 | `launch/mujoco_world.launch` |
| 各Beeの起動 | `launch/multi_robot_bringup.launch` → `launch/bringup.launch` |
| MuJoCo起動前のシーン生成・パラメータ待機 | `script/prepare_mujoco_scene.py` |
| 中央ノード・ROS入出力・実行ゲート | `src/cooperative_grasp/node.cpp` |
| phase管理・目標生成 | `src/cooperative_grasp/phase_manager.cpp` |
| 制御の更新順序・各機体の指令合成 | `src/cooperative_grasp/controller.cpp` |
| 支持力移行・Z軌道生成（旧経路のPIDも保持） | `src/cooperative_grasp/vertical_control.cpp` |
| 中央の速度・加速度参照生成 | `src/cooperative_grasp/tracking_reference.cpp` |
| 各Beeの追従・外力補償・モード切替・通信監視 | `src/control/grasp_mode.cpp`（`BeeController`が所有） |
| grasp指令を既存の飛行制御へ適用 | `src/control/bee_controller.cpp` → `src/bee_navigation.cpp` |
| 中央→各Bee／各Bee→中央のメッセージ | `msg/GraspReference.msg` / `msg/GraspTrackingStatus.msg` |
| 押し付け力のramp | `src/cooperative_grasp/normal_force_control.cpp` |
| 局所モーメント・物体姿勢・力配分 | `src/cooperative_grasp/attitude_control.cpp` |
| 外レンチ変換・集約・接触位置・速度推定 | `src/cooperative_grasp/state_estimator.cpp` |
| 制御状態・出力の監視 | `src/cooperative_grasp/safety_monitor.cpp` |
| 状態・設定・目標・指令の型 | `include/bee/cooperative_grasp/types.h` |
| 開始・運搬・中止の操作 | `script/grasp_keyboard.py` |

`bee_cooperative_grasp`ライブラリはROSに依存しません。`step(state, reference, dt)`は計算と内部状態更新のみを行い、sleep、publish、ファイル書き込みをしません。ROS依存はnode側です。パラメータ名を列挙する`*_parameters.inc`はnodeの読み込みに使用します。

## データと座標系

- `InputState`: 物体pose、各機体baselink/CoG pose、world線速度、baselink局所角速度、CoG座標のobserver外レンチ、flight state。
- `GraspState`: 物体速度、worldでの物体合成レンチ、各4球代表中心の位置・速度、面法線・接線、相対接触高さなど。
- `Reference`: phase、目標物体高度・yaw・XY速度、法線力、roll、接近距離、支持倍率、荷重移行開始・接地イベント。
- `ControlOutput`: 各機体の接触代表点のworld速度・運動加速度、法線力目標、支持移行用の力と補償混合率、yaw、baselink姿勢。接近・退避では従来のナビゲーション指令を中継します。

単位はm、s、kg、N、Nm、radです。ROSメッセージはnodeでこれらの型へ変換します。CoG外レンチは接触前にbaselineを取り、worldへ回転してCoGから4球代表中心へ作用点を移します。物体側へは作用反作用で符号を反転し、物体中心へモーメントを移します。4球ごとの力を分解する処理はありません。

このリポジトリのMuJoCo `ground_truth` はframe_idが空です。`interface_mujoco.yaml`の`allow_empty_base_frame: true`で、この入力だけを既知のworld座標として扱います。CoG odometry、物体pose、observerのframe検査は省略しません。

## phaseと制御メモリ

```
Idle → Staging → Orient → Calibrate → Approach → Preload
     → SupportTransfer → Lift → Hold → Transport → DestinationSettle
     → Descend → Unload → Release → Retreat → Unwind → Verify → Complete
```

- 開始には全台HOVERと有効な入力が必要です。arm/takeoffは中央ノードから送信しません。
- `SupportTransfer`はobserverで支持力移行を確認し、`Lift`から有限高さ軌道を進めます。
- `Hold`への進入は物体とreferenceの高さ・速度の安定で判断します。
- `Lift → Hold → Transport`は同じ制御器instanceを継続し、積分・フィルタ・referenceをresetしません。
- 台座接地を確認すると`Unload`へ移り、接地イベントにより制御器がZ積分と共通slip補正を解除します。
- `task/automatic_transport: false`では`Hold`で待ち、`~transport`で先へ進みます。
- `controller/contact_tangent_control_enabled`は、`Preload`から`Release`までの接線方向PD（面の中央への位置補正と相対速度の減衰）を切り替えます。現在の`controller.yaml`では比較試験のため`false`です。`true`で従来の制御に戻ります。接近・退避時の接線PD、物体yaw補正、共通XY移動、法線力・Z制御には影響しません。ノード起動時に読み込むため、変更後は再起動してください。
- 中止は`Aborted`へラッチし、同じnodeで再開しません。execute時はHOVERを継続している機体へXYZ速度0を送り、姿勢目標を巻き戻さず、LAND/STOP機体には介入しません。

## 押し付け力と接触高さの設定

各機の共通目標力は `normal_force_margin * object_mass * (gravity + design_vertical_acceleration) / (robot_count * friction_coefficient)` で計算します。水平な物体の荷重を均等に支持する設計です。物体質量・台数はscene manifestを使用し、設計摩擦係数はシミュレータの接触パラメータから独立した`task/friction_coefficient`で指定します。モーメント補正による各機の力配分は別途従来どおり加算します。

- 標準設定は摩擦係数0.5、余裕率1.2、想定上向き加速度0 m/s²です。500 g・3台なら3.924 N/台になります。想定加速度0は静的支持に基づく設定です。
- 計算値はノードの`~normal_force_target`に公開します。計算値が`controller/max_normal_force`を超える場合は起動エラーにします。
- `task/normal_force`の固定値指定は廃止しました。残っている場合は、無視せず設定移行を求めるエラーにします。
- Preload完了は全機のフィルタ済みobserver法線力が`task/preload_force_ratio * normal_force_target`以上で、`task/preload_duration`だけ連続していることです。標準は90%・3秒（上記条件では3.5316 N以上）。指令値の到達は判定に使用しません。各機のモーメント配分後目標への収束判定ではなく、共通目標に対する各機の最低支持能力の確認です。
- 標準`controller.yaml`では`z_velocity_kp`と`z_velocity_kd`を0にし、支持移行前の接触相対高さPDだけを無効にしています。この間はZ速度0を指令します。支持移行後の物体高度制御・荷重支持・傾き補正は維持しています。旧中央加速度経路では接触高さをそろえるP項を`relative_z_kp: 8.0`に設定しています。既定の分散追従では、この項を使わず、接触配置は診断・安全監視に使います。位置を戻す補正はありません。速度差のD項は`relative_z_kd: 0.0`、共通すべり補正は`slip_kp/kd: 0.0`で無効のままです。相対高さ・すべりの計測と安全判定は継続します。従来のゲインは高さ差が8.0/2.0、共通すべりが5.0/2.0です。設定は起動時に読み込まれます。

## シーンとの境界

`MujocoSceneGenerator`はROS masterなしで、同じフラットなシーン設定からXML・composer YAML・制御用manifestを生成します。既存の`mujoco_multi_module.launch`の引数と既定値は維持しています。

`mujoco_world.launch`の起動前処理が生成完了後、manifestを`/bee_grasp/scene`へ一括登録します。起動時は`available: false`にします。中央制御は`~scene_namespace`で指定したmanifestを読み、各Beeのbringupのprivate parameterを直接参照しません。

ROS launchのinclude順には実行順序の保証がないため、`prepare_mujoco_scene.py`をMuJoCoノードの`launch-prefix`に指定しています。この小さな前処理がシーンを生成し、全台のモデル・motor/servo・AerialRobotHWSim設定を待ち、既存の初期化猶予3秒を置いてから`exec`でMuJoCoへ切り替わります。Pythonから子launchは起動しません。各Beeの標準controller spawnerはcontroller managerの出現を待ちます。MuJoCoを含む全ノードの終了処理はroslaunchが担当します。

manifestには`schema_version`、利用可否、機体名、確定した物体質量・寸法・初期pose、各面の角度・中心からの距離、待機距離、目的地の位置が入ります。密度指定時も生成側で確定した質量を渡します。ゲインや描画設定は含めません。物体なし・物体面数と機体数が一致しないシーンも生成できますが、中央制御用manifestは`available: false`になります。

シーンの例は`config/grasp/scene_triangle.yaml`、`scene_box.yaml`、`scene_pentagon.yaml`です。`grasp_sim.launch robot_count:=3/4/5`で対応するシーンを選びます。タスク設定は3台で`task.yaml`、4・5台で`task_multi.yaml`を選びます。目標押し付け力は、確定した物体質量と台数、タスクの設計摩擦係数・余裕率・想定鉛直加速度から計算します。`scene_config`・`task_config`等で別ファイルも指定できます。シーンYAML内の`robot_count`はlaunchの台数と一致する必要があり、不一致ならMuJoCo起動前にエラーにします。これらの明示的な設定選択以外に、旧デモの形状別チューニングをC++側で自動適用することはありません。

生成だけ行う場合:

```bash
rosrun bee mujoco_scene_generator.py \
  "$(rospack find bee)/config/grasp/scene_triangle.yaml" \
  --output-dir /tmp/bee_grasp_scene
```

生成結果は`bee_scene_3.xml`、`bee_scene_3.yaml`、`bee_scene_3_control.yaml`です。ソース機体モデルの再生成が必要な場合は、既存のmodel generatorを呼びます。

## 起動

ワークスペースで`catkin build bee`し、そのdevel/setup.bashをsourceしてください。

3台・三角柱でシーン、各Bee、中央制御をまとめて起動:

```bash
roslaunch bee grasp_sim.launch
```

MuJoCoのGUIを表示し、中央制御から実際に指令を送るモード:

```bash
roslaunch bee grasp_sim.launch headless:=false execute:=true
```

`execute`の既定値は`false`です。この場合、計算した指令は`~preview/beeN/reference`（正式な参照）、`~preview/beeN/nav`と`~preview/beeN/attitude`（旧形式の表示）に出します。中央は`/beeN/grasp/reference`へ送信しないため、各Beeのgrasp modeも飛行指令を出しません。`execute:=true`でも離陸や把持開始は自動では行いません。各Beeを既存の操作方法で離陸させ、全台のHOVERと入力がそろった後、`rosrun bee grasp_keyboard.py`で`g`を押すか、次のserviceで開始します。

```bash
rosservice call /bee_cooperative_grasp/start "{}"
```

操作は`g`が開始、`t`がHoldから運搬、`x`/Ctrl-Cが中止です。旧キーボードデモのXYZ/yaw連続操作はこの操作入口には含めていません。中央ノードはタスク設定に従って自動シーケンスを進めます。操作クライアントの終了自体は自律制御の停止条件ではありません。

このC++入口は現段階では`use_sim_time=true`でのみ実行可能です。旧デモと同時に飛行指令を送らないでください。実機実行は引き続き既存の`real_grasp_demo.py`を使う構成です。

4台・直方体／5台・五角柱の場合（どちらか一方を実行）:

```bash
roslaunch bee grasp_sim.launch robot_count:=4 execute:=true
roslaunch bee grasp_sim.launch robot_count:=5 execute:=true
```

個別に起動したい場合は、次の3つをそれぞれ別端末で実行できます（この場合は`grasp_sim.launch`を重ねて起動しません）。

```bash
roslaunch bee multi_robot_bringup.launch robot_count:=3
roslaunch bee mujoco_world.launch robot_count:=3
roslaunch bee cooperative_grasp.launch execute:=true
```

旧`mujoco_grasp_scene.launch`、`mujoco_multi_module.launch`と、それらが使う`mujoco_multi_bringup.py`は既存の起動手順との互換用に残しています。新しい`grasp_sim.launch`からは呼びません。

`~phase`、`~reason`を公開します。`~metrics`の順序は、法線力、Z reference位置・速度・加速度、旧Z積分加速度、鉛直目標力、旧共通Z加速度、支持ramp比、平均接触滑りです。分散追従では旧加速度項はゼロで、実際の参照と出力は`/beeN/grasp/reference`と`/beeN/grasp/tracking_status`を参照してください。CSVが必要な場合はこのtopic等を外部記録し、制御周期からファイルI/Oを分離します。

## 分散追従の制御分担

既定の`controller/distributed_tracking: true`では、中央は`/beeN/uav/nav`へ直接publishしません。各Beeの`/beeN/grasp/reference`を使い、`/beeN/grasp/tracking_status`でready、校正完了、速度誤差、推定外力、最終加速度、faultを返します。中央は各Beeのgrasp modeの準備完了とstatusの鮮度も開始・継続条件に含めます。

- 中央の水平参照は物体の目標速度から生成し、鉛直参照は滑らかな物体高さ参照に対するPI速度補正です。物体モーメント配分は中央で速度参照へ反映します。接触配置誤差から位置を戻す速度補正は加えません。物体yawの目標は一定で、実測の角速度を目標へ加えて不要な回転を追いかけることはしません。`cooperative_admittance`は既存の差動加速度を速度補正に変換する時定数です。以前の加速度用ゲインとは単位が異なります。
- 各速度参照を制限・slewした後、その離散時間差分を加速度FFとして送ります。これは接触代表点の運動参照です。物体の重量分を加速度FFへ加えません。位置目標は接触中のローカル追従には使いません。
- grasp modeは自機のbase速度・角速度とcontact offsetから接触代表点速度を計算し、接線・Z方向で速度P追従と外力補償を行います。法線は目標法線力へのフィードバックと相対速度減衰を使い、外力を重ねて打ち消しません。接触部の角加速度・遠心加速度をCoG指令へ厳密変換するものではなく、機体姿勢をほぼ一定に保つ運搬用です。
- 支持移行の最初から実測外力だけを補償すると、荷重を引き受ける力が立ち上がりません。このため`support_force`で各機の支持を立ち上げ、支持判定後に`(1-b)*support_force - b*estimated_external_z`で実測外力へ連続移行します。両者を全量ずつ加算しません。`Unload`では支持倍率で減らし、`Release`では支持補償をゼロにします。
- observer baselineは中央と各Beeのgrasp modeがCalibrate中にそれぞれ取得します。力のframeはCoG、位置・速度・参照はworldです。実機に接続する際のbase odometryはworld線速度・body角速度という契約を満たす必要があります。
- 共通滑り補正は現在無効。初期接触配置へ戻す補正と全機の接触高さを平均にそろえる処理は行いません。接触中の横位置を中央へ戻す補正は無効ですが、ローカルの接線速度追従は有効です。
- grasp modeは参照／自機状態の欠落、時刻異常、過大指令、持続する速度誤差で停止・faultをラッチします。中央が停止しても参照timeoutで各機が停止指令を出します。LAND/STOPへ移った機体へ停止速度を上書きしません。

`launch/bringup.launch grasp_mode:=true`で各機のcontroller内機能を有効化します。既定はfalse、MuJoCoのmulti bringupではtrueです。中央と各機に同じ設定ファイルを渡し、ローカルの速度ゲイン・timeoutは同ファイルの`tracker`節を使います。各機では設定を`/beeN/grasp/`配下へ読み込み、通常の飛行制御ゲインと分離します。実機実行の中央ゲートは従来どおり維持しています。

実行中の切り替えは`std_srvs/SetBool`を使います（パラメータを直接書き換えるだけでは切り替わりません）。

```bash
rosservice call /bee1/grasp/set_mode "data: true"   # grasp modeを有効化
rosservice call /bee1/grasp/set_mode "data: false"  # 停止指令を渡して通常モードへ戻る
```

- `/beeN/grasp/enabled`にモードをlatched配信し、`/beeN/controller/grasp_mode`にも反映します。有効化だけでは動かず、中央からの新たな参照を待ちます。
- 通常モードではgrasp参照を無視し、通常のナビゲーションを維持します。参照追従中は外部の`uav/nav`と競合しないよう、BeeNavigatorの同経路を抑止します。
- 参照計算・通信監視はcontroller所有の100 Hz WallTimerで行い、mutexで受信状態を保護します。最新の計算結果は`BeeController::update()`が既存の40 Hz飛行制御周期で適用します。飛行制御ループの停止も入力鮮度として監視します。
- HOVER・非合体状態でのみgrasp指令を適用し、LAND/STOP等には上書きしません。無効化／停止／fault時はゼロ速度指令を次の飛行制御周期で渡し、通常の入力経路を解放します。faultは参照再開だけでは解除せず、明示的なoff/onで新しい校正から開始します。
- `src/cooperative_grasp/tracker_node.cpp`と`grasp_tracker.launch`は旧実装の比較用として保持しています。`launch_grasp_tracker:=true grasp_mode:=false`の場合だけ旧独立ノードを起動し、controller内インタフェースを無効化します。標準起動からは使いません。

比較用に`distributed_tracking: false`を指定すると旧中央加速度経路になります。各Beeのgrasp modeは参照を受け取らないため出力しません。旧ゲインと新しい速度参照ゲインは別の設定です。

## 物体座標系の接触配置の監視

既定の分散追従では、Preloadが成立してSupportTransferへ入った時点の各接触代表点を記録します。

```text
rho_i_ref = R_object_capture^T * (p_contact_i_capture - p_object_capture)
e_i       = R_object_current^T * (p_contact_i_current - p_object_current) - rho_i_ref
e_diff_i  = e_i - mean(e)
```

初期接触点の高さが異なっていても、記録した配置が維持されていれば誤差はゼロです。`/tmp/check_bee_contact_geometry.cpp`で高さ差30 cmの配置、剛体並進・回転、1点のずれ、配置変形による停止、resetと共通変位の分離を確認しています。物体と接触点をまとめて並進・回転しても誤差は変わりません。物体の姿勢はyawだけでなく全姿勢を使います。基準は各tickで上書きせず、次の把持開始まで固定します。実際の接触位置から計算する力の作用腕は引き続き毎tick更新します。

`ContactGeometry`の3次元誤差は診断と安全判定に使います。初期配置からのずれを速度指令へ変換する比例補正は削除しました。各Beeの接線・Z方向の速度追従、法線力制御、中央の物体高さ制御とモーメント配分は維持しています。接触位置が移動して相対速度がゼロになった場合、その位置ずれ自体を戻す指令は出しません。

復元項の削除後、ビルドと既存回帰（集計26件）が成功しました。`/tmp/check_bee_contact_monitor_only.cpp`で、単点の位置ずれから速度・加速度の復元指令が出ないこと、診断誤差と配置変形による停止が残ることを確認しました。削除後の運搬飛行は下記の5回で再確認しました。

安全判定は、分散追従では`norm(e_diff_i) <= safety/max_contact_deviation`（既定0.04 m）と`norm(mean(e)) <= safety/max_slip`（既定0.06 m）です。初期配置そのものの高さの差は異常としません。`~contact_geometry_errors`で、manifestの機体順に`[ex, ey, ez, ...]`を物体座標・m単位で出力します。旧経路だけは既存の`max_height_difference`判定を維持します。

この変更は配置誤差の定義を一般化したものです。現在のシーン生成・接近・ローカルの力制御法線は側面を水平法線で扱い、支持配分も鉛直摩擦を前提にしています。任意の面法線、上下からの把持、任意姿勢での全工程を成立させるには、それらの力学・接近・安全な姿勢範囲も別途対応が必要です。

以下は配置復元項を削除する前の検証履歴です。配置補正の変更後、3台・500 g・0.4 mの運搬試験はCompleteまで到達しました。持ち上げ50.01 mm、最終XY誤差4.50 mm。記録：`/tmp/bee-distributed-flight-3-zom17nmf/result.json`。既存C++回帰26件も成功しました。異なる初期接触高さ・回転への不変性は数値試験での確認であり、任意配置での飛行を検証したという意味ではありません。

## BeeController内grasp modeの検証（2026-11-12）

- ビルド成功、既存回帰の集計26件・失敗0。`/tmp/bee-grasp-mode-build.log`、`/tmp/bee-grasp-mode-unit.log`。
- controller内で使用する同じ`GraspMode`実装を一時的なROSホストへ組み込み、4-thread spinnerと40 Hzの指令取得で検証。通常モードの参照無視、有効化前／参照前の無指令、稼働中の無効化によるゼロ速度停止、再有効化時の再校正、速度追従・外力補償、参照timeoutと入力timeoutの停止・faultラッチを確認。
- 合成入力試験：参照timeout `/tmp/bee-grasp-mode-contract-jx_i37_t/`、入力timeout `/tmp/bee-grasp-mode-contract-6k4n6qtv/`。予測荷重時1.200、実測外力へ移行後約1.200、速度誤差解消後約1.050 m/s²で補償の重複なし。
- 最初の合成試験ではfault通知直後に停止指令を検査して失敗。通知と次の40 Hz指令適用の順序を考慮し、停止指令の到着まで待機するハーネスへ修正して再成功。制御側の停止処理は変更していません。
- MuJoCoで3台500 g、4台500 g、5台800 gを各1回試験し、全3回Complete。約50 mm持ち上げ、400 mm運搬、載置・解放・退避まで確認。最終XY誤差は順に8.90 / 3.12 / 2.12 mm。
- 各回、起動ノード一覧に独立`grasp_tracker`がないことと各Beeの`aerial_robot_base_node`の存在を検査。モードのoff/on serviceとパラメータ反映も全機で確認。全工程でfaultなし、assert/Tracebackなし、全launch終了確認。ソース・設定・バイナリのハッシュは試験間で不変。
- 集計：`/tmp/bee-integrated-validation-0kgwvo0u/summary.json`、`results.csv`。各回の`nodes.json`にノード一覧を保存。実機では未検証。新しいテストソースはリポジトリへ追加していません。

## 分散追従の検証（2026-11-11）

- ビルド成功、既存C++回帰26件成功。新しいテストソースはリポジトリへ追加していません。
- `/tmp/check_bee_references.cpp`で校正フラグ、支持中の運動加速度に荷重を含めないこと、速度・加速度の差分整合、上限、observer混合率、停止したsim timeを確認。
- `/tmp/check_bee_tracker.py`の独立ROS masterと合成入力で、未指令時の無出力、速度誤差への応答、予測荷重と実測外力の非重複、指令timeout・自機状態timeout時の停止とfaultラッチを確認。
- 初回の閉ループ試験では実測角速度を参照へ足したことで不要なyaw回転を追従し、運搬中にyaw安全上限で中止。姿勢一定タスクの参照からこの項を除去。
- 修正後の3台・500 g三角柱は約50 mm持ち上げ、0.4 m運搬、載置、解放、退避までComplete。最終XY誤差3.24 mm。記録：`/tmp/bee-distributed-flight-3-ito6jnvd/`。
- 参照上限検査と停止処理を含む最終ビルドでも同条件でComplete。最終XY誤差1.16 mm、持ち上げ49.95 mm。記録：`/tmp/bee-distributed-flight-3-e1f29kb6/result.json`。各機の診断を複数callbackから書いた初期`tracking.csv`には破損行があるため、定量結果は単一スレッドの`telemetry.csv`と`result.json`を使用。以後のハーネスには書き込みlockを追加。
- 既存のROSインタフェース試験でも、旧経路のpreview無指令、実行、中止・再開拒否、時刻重複の確認が成功。
- 下記の追加試験で4・5台の分散追従もMuJoCoで完走を確認しました。実機は未検証です。

### 接触位置の復元なし・モーメント制御ありの反復検証（2026-11-11）

ユーザー指定の構成で、各回独立したROS masterとMuJoCoを起動しました。中央のモーメント配分は現在の接触位置から作用腕を計算し、初期配置へ戻す速度補正は加えません。接触配置の診断・安全監視は残しています。

| 条件 | 完了回数 | 最終XY誤差 |
| --- | --- | --- |
| 3台・500 g | 3/3 | 8.94 / 6.20 / 8.33 mm |
| 4台・500 g | 1/1 | 0.87 mm |
| 5台・800 g | 1/1 | 2.72 mm |

- 全5回で約50 mm持ち上げ、400 mm運搬、載置・解放・退避までComplete。実測最大持ち上げ49.92–50.05 mm。全工程のtracker fault、launchのassert/Tracebackなし。全launchの終了を確認。
- 運搬等の記録区間（Lift/Hold/Transport/DestinationSettle/Descend）の差動配置誤差最大1.76–2.83 mm、共通配置誤差最大16.08–22.23 mm。共通すべりを戻す制御は無効のままです。
- 3台では下降開始から終了までXY位置が5.72–8.58 mm移動しました。完走結果と位置精度は分けて評価します。前回との回数・条件だけでは制御性能差の統計的判断はできません。
- 毎回起動時のROSパラメータを保存し、distributed_tracking=true、contact_tangent_control_enabled=false、z_velocity_kp/kd=0、slip_kp/kd=0、配置復元ゲインの不存在を確認。ソース・設定・バイナリのハッシュは全試験で不変。
- `/tmp/check_bee_contact_monitor_only`も再実行し成功。位置変位だけでは復元速度・加速度を出さず、ずれの検出・安全停止を維持することを数値確認。
- 集計：`/tmp/bee-no-position-validation-7qwc1x5b/summary.json`、`results.csv`。各回の`runtime_controller.json`と`force_references.csv`に設定と目標力を記録。制御コード・設定の追加変更、リポジトリへのテストソース追加なし。
- 今回は標準の側面把持シーンでの反復確認です。任意接触配置や意図的に大きなすべりを加えた飛行は未検証です。前回のPreload timeoutは今回は再現しませんでしたが、原因修正を行ったわけではありません。

### 接触配置補正後の反復検証（2026-11-11、復元項削除前）

制御ソース・設定・バイナリを固定し、各回専用のROS masterとMuJoCoを起動して、50 mm持ち上げ、400 mm運搬、載置・解放・退避を5回試験しました。

| 条件 | 結果 | 完走時の最終XY誤差 |
| --- | --- | --- |
| 3台・500 g | 2/3 Complete、1回Preload timeout | 3.86 / 0.98 mm |
| 4台・500 g | 1/1 Complete | 3.15 mm |
| 5台・800 g | 1/1 Complete | 0.71 mm |

完走した4回の持ち上げは49.99–50.02 mm、記録上の最大差動接触配置誤差は1.56–2.52 mm、最大共通接触配置誤差は16.25–22.03 mmでした。全5回でtracker fault・assert・Tracebackは記録されず、launch終了を確認。ハッシュ比較でも試験間のソース・設定・バイナリの変更はありません。

失敗した3台の1回は、配置補正を有効にするSupportTransferより前のPreloadで150秒の上限に到達しました。法線力の共通目標は3.924 N、移行条件は全機がその90%（3.5316 N）以上です。一方、モーメント配分は各機の目標を最大±0.6 N補正でき、低い側の目標3.324 Nと共通の移行閾値が矛盾する可能性があります。終了前の各機の推定外力XYノルムは約4.37 / 4.07 / 3.32 Nでした。個別の目標法線力を今回のCSVに保存していないため、この不整合は原因候補であり確定ではありません。制御値や判定閾値は変更していません。

- 集計：`/tmp/bee-grasp-validation-hu3ukw4b/summary.json`、`results.csv`。失敗ログ：`/tmp/bee-distributed-flight-3-w6r4zqt0/`。
- ビルド成功、既存C++回帰の集計は26件・失敗0。接触配置の数値試験に加え、3/4/5台各1,000条件（seed 20261111）で剛体変換不変性、単点変位の復元、reset/captureを確認。異なる初期接触高さの飛行試験は未実施です。
- 合成入力による参照timeout試験は成功（`/tmp/bee-tracker-contract-qwdl13i5/`）。自機入力timeout試験は並行実行時に意図した入力停止より前の鮮度異常で失敗（`/tmp/bee-tracker-contract-w6l8jyx1/`）、全飛行試験後の単独再試験は停止・faultラッチを含め成功（`/tmp/bee-tracker-contract-b_ogb_t9/`）。先の鮮度異常の原因は未確定です。
- 試験ハーネスは`/tmp`に置き、リポジトリにテストソースを追加していません。

## 移行段階と検証範囲

これは新構成の初期実装です。既存Pythonの`legacy`を参照し、主要な力制御・Z制御・滑り補正・モーメント配分を実装していますが、全内部状態・全設定の完全互換を保証する置き換えではありません。

- 新しいphase管理では`Preload`と`SupportTransfer`を明示し、姿勢到達と実測法線力も遷移条件に使います。旧Pythonの全タイミングと同一ではありません。
- MPC、旧キーボードの連続XYZ/yaw操作、旧CSV全列、実機実行、形状別の旧デモのチューニング自動適用は移行対象に残っています。
- 元のPythonデモと既存実機入口は残し、比較・従来の実行に使えます。
- 旧中央加速度経路の新C++で3台・500 g三角柱と4台・500 g直方体のMuJoCo閉ループ試験が`Complete`まで到達しました。把持、約50 mm持ち上げ、X方向0.40 m運搬、載置、解放、退避を確認しています。3台の最終XY誤差は約1.1 mmでした。各条件1回の完走確認であり、旧制御との統計的な性能比較・実機検証は行っていません。
- 5台・800 g五角柱は2回試験し、いずれも`Approach`中に入力鮮度の検査で`Aborted`になりました（1回目はbee1のflight state、2回目は物体pose）。`input_timeout=1.0 s`は変更していません。5台の生成・数値計算は検証済みですが、自律飛行の完走は未確認です。入力配信とシミュレータ負荷の計測が残っています。

今回の検証:

- 2026-11-06の再確認で見つかった起動時のEigen assertは、IMU/mocap間のカルマンフィルタ状態切替競合と特定し、`kalman_filter`側のmutex保護を修正。競合の回帰テストを含む4件が通過し、利用側まで再ビルドした。修正後は3台headless 3回、4・5台headless各1回、3台GUI/execute=true 1回の全6起動試験が成功。これは起動・終了の確認であり、離陸・把持の再試験ではない。
- 起動構造の分離後、`grasp_sim.launch`の3・4・5台すべてで、専用localhost ROS masterによるheadless起動を確認。各台のattitude controllerがrunning、物理時刻が進行、中央制御がIdle、previewで中央制御の飛行publisherがないこと、終了後に起動したノードが残らないことを確認。この起動試験では離陸・把持指令は送っていません。
- 起動前処理のofflineテスト5件: YAMLと台数の不一致、不足した機体パラメータ、誤ったHW plugin、準備完了、準備中の終了。
- C++ライブラリとROSノードを独立した`/tmp`のbuild/develおよび通常のワークスペースへビルド。通常ビルドは`catkin build bee --no-deps --no-status -j2`で成功。
- C++テスト13件: 座標・作用点変換、baseline、時刻重複、Zメモリ継続、差動指令の平均、phase全工程、異常停止、Pythonとの数値比較。
- `grasp_press_reference.csv`は既存Pythonの押し付け計算36条件、`grasp_trajectory_reference.csv`はZ reference生成300 sampleから取得した固定参照値。比較元はcheckpoint `29c1a958`の制御式です。
- シーン生成: コミット済み旧実装との配置・物体辞書・確定質量の6条件比較、新しいofflineテスト6件、3形状のXMLをMuJoCo 2.3.7で読み込み。
- 既存の実機入口offlineテスト14件。
- 隔離したlocalhost ROS masterと合成入力で、プレビュー無指令、実行モードの指令送信、入力途絶による中止、再開拒否、シミュレーション時刻重複時にheartbeatを誤判定しないことを確認。

```bash
catkin test bee
python3 "$(rospack find bee)/test/test_mujoco_scene_generator.py"
python3 "$(rospack find bee)/test/test_prepare_mujoco_scene.py"
python3 "$(rospack find bee)/test/test_real_grasp_demo.py"
python3 "$(rospack find bee)/test/check_cooperative_grasp_ros.py" \
  --node /absolute/path/to/devel/lib/bee/bee_cooperative_grasp_node
```

ROS通信テストは自分専用のROS masterを起動し、終了時にテストプロセスを停止します。既存のmasterや実機には接続しません。
