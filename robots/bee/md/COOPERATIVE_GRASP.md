# 中央協調把持制御とMuJoCo生成の構成

現行の起動入口・C++担当・削除済み旧入口は [SOURCE_MAP.md](SOURCE_MAP.md) を参照してください。
旧Python把持入口（実機用を含む）は2026-11-16の整理で廃止しました。

力の応答時間を測る手順・実機bagの記録と解析・モデルの適用範囲は
[接触力応答の同定](FORCE_IDENTIFICATION.md)を参照してください。
同定ツールは制御設定を自動変更せず、実機へ指令を送る機能はありません。
別のMuJoCo専用runnerだけがシミュレーションで飛行・励起します。

押付け力と支持力の時間差・支持容量の検証は
[支持移行の検証](SUPPORT_TRANSFER_VALIDATION.md)を参照してください。
2026-11-15から円錐・円錐台を当面の評価対象から外し、標準の反復runnerは円錐台を除く7条件です。

## Joy での開始と緊急停止

単機の手動操縦では、操縦 PC で `rosrun joy joy_node __ns:=/bee1 _dev:=/dev/input/js0 _autorepeat_rate:=20.0` を起動する。
HOVER 中は左スティック上下で世界 X、左右で世界 Y を指令する。既定の位置制御では軸入力の絶対値が 0.2 未満だと速度目標はゼロになる。
同じ傾きを保持している間も再送する設定にして、ナビゲータの入力タイムアウト（既定 0.5 秒）を超えないようにする。

中央と各機の起動後、操縦PCで `roslaunch bee joy_stick.launch` を起動する。MuJoCo で手動開始する場合は `roslaunch bee grasp_sim.launch execute:=true robot_count:=3` を使う。

| 操作 | 動作 |
|---|---|
| 十字キー左＋○ | 従来の離陸 |
| L1＋R1 | 全機が HOVER かつ中央の開始条件を満たすと把持開始。再整列完了後は再接近を開始。拒否後は両ボタンを離して再入力 |
| R1＋□ | Approach／Preload 中に再整列。押し付け指令をゼロにし、現在の物体姿勢に対して各担当面の手前20 cmで全機待機 |
| L2＋R2（左右トリガーを同時に引く） | 協調緊急停止。中央は Aborted。分散制御の各 Bee は力・軌道追従を解除し、接触中で状態が新鮮なら現在高度で水平外向き 10 cm を位置目標にする。物体は落下し得る |
| L3＋R3（左右スティック押し込み） | 協調停止後、物体から離れて水平に戻せる状況で押す。各 Bee は位置保持を続け、その時の機首方位でロール・ピッチを徐々に 0° にする。把持は停止したまま |
| 十字キー右＋□ | 従来の着陸 |

中央指令の追従中は左右スティックの移動・yaw と手動の姿勢・モード切替を無効にする。着陸・緊急停止は操作できる。追従解除後は通常の手動操作に戻る。

再整列の進捗と接触待ちの警告は `/bee_cooperative_grasp/reason` と ROS ログに出る。荷重移行前の接触待ちタイムアウト、保持計画が成立しない場合はその場で再評価を続ける。通信・状態推定の途絶などによる自動停止は有効。

外部から同じ停止を指示するには `rostopic pub -1 /bee_cooperative_grasp/emergency_stop std_msgs/Empty '{}'` を使う。同一 ROS master 上でこのトピックを購読している全機が対象。停止は受信側でラッチされ、開始ボタンを押し直しても再開しない。次のデモは着陸後に中央と各機の参照追従を再初期化して開始する。

## SMACH Viewer と RViz

`roslaunch bee grasp_demo.launch auto_start:=false` は MuJoCo と両ビューアを起動する。Joy は別途起動し、離陸後 L1＋R1 で開始する。自動実演は `auto_start:=true`、SMACH ウィンドウの表示は `launch_smach` で選ぶ。

SMACH Viewer の `/BeeGrasp` は Prepare（準備）、Grasp（接触・把持）、Carry（運搬）、Place（着台・退避）の4工程を表示する。詳細を見るには Auto Focus を解除し、Path で `/BeeGrasp/Grasp` 等を選ぶ。中央のログは遷移時と動作中の約5秒ごとに `[BeeGrasp] Grasp/Preload` の形式で現在地を出す。実機では中央の起動後に `rosrun smach_viewer smach_viewer.py` で同じグラフを開ける。状態遷移は中央が管理し、ビューアは読み取り専用。

RViz の **Bee contact forces** は各 Bee の足の代表点から、物体に加える力を world 座標で描く。水色は中央の BeeCommand の力指令、橙は中央がゼロ点補正・座標変換した observer の反力を反転した推定値。両方とも **1 N = 5 cm**。矢印が重なる場合は表示内の `command` / `observer` を個別に切り替えられる。指令ゼロや終了時には該当する矢印が消える。実機の Bee bringup 用 RViz 設定でも、CAD と接触力の両方を既定で表示する。

## 実物三角柱の CAD を共用する

実物三角柱（1.248 kg）は `config/grasp/object_triangle_real.yaml` を指定する。重心と軸は CAD の座標系1。mocap 剛体の原点・軸もこれに揃えるか、実測したマーカー変換を設定する。表示は元の STL、接触は側板3枚と上下のキャップ6個の部品別凸包を使うため、キャップのくぼみは近似になる。標準3機の grasp_demo.launch はこのモデルを使う。滑り摩擦係数1.0で必要押し付け力は約6.09 N/機、押し付け上限は10 N/機、誤差余裕0.50 Nを引いたQP指令上限は9.50 N/機。この三角柱のMuJoCoデモでは中央と3機の世界座標推力L1上限を36 N、うち補正用を3 Nとする。実機の既定は32 N。

三角柱デモでは `controller/reference_tracking/force_at_velocity_point` を有効にし、力の作用点を足の代表点に合わせる。Bee は重心から作用点へのベクトルと力指令の外積を推力配分に加える。通常の単機制御ではこの設定は無効。支持中の接線方向の加速度範囲は、割り当てた支持力を機体質量で割った値を中心とする。支持移行中に既に浮いたときは、全荷重指令と観測支持力の成立が続けば Lift に移り、支持移行開始時の高さから目標高さを決める。

別のモデルを追加する場合は `config/grasp/object_triangle_example.yaml` を複製して実測値に置き換える。3機の三角柱では次の物体パラメータを実機と MuJoCo の両方に使う。

| YAML | 単位・座標系 |
|---|---|
| `object_mesh` | OBJ またはバイナリ STL。絶対パスまたは `package://`。RViz の PC からも読めるファイルを指定 |
| `mesh_scale`, `mesh_xyz`, `mesh_rpy` | CAD頂点を m に換算する3軸倍率（mmなら各0.001）、その後に適用する重心座標系から見たメッシュ原点の位置[m]・固定軸RPY[rad] |
| `object_mass`, `object_inertia` | kg、重心まわり・物体座標軸での慣性行列9成分（行優先、kg·m²）。CADの原点まわりの値は重心まわりに変換して入力 |
| `object_height`, `object_bottom_offset` | 全高、重心から底面までの距離[m]（省略時は全高の半分） |
| `face_angles`, `face_radii`, `contact_height` | 各担当面の外向き方位角[rad]、重心から面までの水平距離[m]、重心基準の接触高さ[m]。CADの三角形からは自動推定しない |
| `marker_to_model_xyz`, `marker_to_model_rpy` | mocapマーカー座標で表した重心の位置[m]と物体座標軸の向き[rad] |
| `collision_meshes` | 任意。中空部・凹部への接触を再現したい場合は、同じCAD座標系で凸部品に分けたメッシュのリスト。省略時は表示メッシュの凸包で接触計算 |

MuJoCo は `roslaunch bee grasp_demo.launch object_model:=/absolute/path/to/triangle.yaml auto_start:=false`。
配置と運搬先は `scene_config` の台設定に従う。実機は各機の bringup 後に `roslaunch bee cooperative_grasp.launch object_model:=/absolute/path/to/triangle.yaml`。物体用 OptiTrack の既定の剛体 ID は 10。既存の配信元を使う場合は `object_mocap_id:=false` を渡す。いずれも `/mocap/object/pose`（world でのマーカー姿勢、PoseStamped）を中央の入力とする。

`rosrun tf tf_echo world grasp_object` でモデル重心の TF を確認する。RViz の Fixed Frame は `world`、Marker は `/bee_cooperative_grasp/object_mesh`。表示は物体姿勢が届けば Idle 中も更新する。MuJoCo の従来の `/bee1/mujoco/grasp_object_pose` は重心の真値として残る。

## 運搬距離の設定

標準のMuJoCo試験は全形状で水平1.50 mです。`config/mujoco/scene_<形状>.yaml` の `object_x, object_y` が初期位置、`destination_pedestal_x, destination_pedestal_y` が運搬先の台の中心です。いずれもworld座標のmで、例えば (0, 1) から (1.5, 1) へ運びます。目的台のX・Yを省略した場合の既定値はそれぞれ `object_x + 1.50` と `object_y` です。
実機は `object_model` で指定する物体YAMLの `destination_position: [x, y, z]` に、載置後の物体重心のworld座標を指定します。初期位置はmocapで測るため、実機での水平距離は実際の置き場所に依存します。

## 離陸から協調把持終了まで（標準の分散制御）

各Beeが離陸してHOVERに入った後、中央の `~start` でStagingに進みます。`execute` は中央が実際の指令を配信するかを選ぶ起動引数です。
中央は各Beeの並進・世界基準のbaselink RPY・接触力を決め、BeeはRPYをCoG基準に変換してfinal targetに渡します。

| 状態 | 中央の目標と次へ進む条件 | 各Beeの動作 |
|---|---|---|
| Takeoff・Hover / Idle | 全機のHOVERと入力の準備を待つ | 単機の飛行制御で離陸し、高さ・姿勢を保持 |
| Staging | 各接触面の手前に待機点を設定。距離、高さ、鉛直速度を確認 | 水平姿勢で待機点へ移動 |
| Orient | 足先を予定接触点から法線方向に20 cm離した点へ誘導し、全機の足位置・相対速度・接触姿勢の整定を待つ | 移動しながら足を物体側へ向ける |
| Calibrate | 無接触の姿勢でobserverの校正完了を待つ | 外力推定のゼロ点を取得 |
| Approach | 予定接触点の法線上で足先の目標距離を縮める。全機の近接と観測法線反力が0.1秒続けばPreload | 先に接触した機体は弱い押付けを保持 |
| Preload | 全重量を支える配分を計画し、各機の押付け目標を漸増。観測法線力と摩擦容量を全機で確認 | 中央から受けた法線方向の力目標を単機の力PIで追従 |
| SupportTransfer | 観測した鉛直合力に合わせてQPの支持目標を増加。支持割合・観測荷重・鉛直速度が整うのを待つ | 配分力のFFと各機の力PIで追従 |
| Lift / Hold | 物体の高さ目標を上げ、高さと速度の整定後に保持。自動設定または運搬要求で次へ | 同伴加速度と各機の配分力を追従 |
| Transport / DestinationSettle | 目的地へ水平移動し、到達後に整定時間を取る | 物体と一緒に移動しながら支持 |
| Descend | 目標高さを下げ、物体の高さ・水平位置・鉛直速度で載置を判定 | 把持を維持して下降 |
| Unload | 台に荷重を戻すため支持割合を時間でゼロへ下げる | 減少する力目標に追従 |
| Release | 押付け上限を下げ、QPで残力を最小化。指令・観測の力が小さくなるまで待つ | 接触力を抜く |
| Retreat / Unwind | 離隔を確認し、roll目標を水平に戻す | 物体から離れて姿勢を戻す |
| Verify / Complete | 物体の位置と速度が許容内に一定時間収まれば協調指令を終了 | 単機の飛行制御に戻る。着陸は別操作 |

## 各Beeの共通制御（2026-11-16）

各Beeの力追従・フィルタ・校正はBeeControllerの共通参照処理へ統合しました。
下流は物体・法線・接触状態・把持フェーズを判断せず、運動/力の目標と汎用の制限を受けます。
独立trackerは廃止しました。操作・設定・メッセージ・移行方法は
[BeeControllerの共通指令追従](CONTROLLER_REFERENCE.md)を参照してください。

校正は`/beeN/controller/calibrate_observer`へ`std_msgs/Empty`を送ります。
力追従中でも受理し、実行タイミングは上位・操作者が決めます。
物体の荷重がかかっている場合、その荷重もゼロ点に含む操作です。
標準のobserver LPFは0.2秒、追加のPI出力LPFは0（無効）です。
設定場所は`config/controller/force_control.yaml`に変わりました。

## 制御で使う接触代表点のTF（2026-11-18）

モデルのURDFに`fc → grasp_contact_point`の固定フレームを定義しています。
既存の`robot_state_publisher`が`beeN/fc → beeN/grasp_contact_point`を`/tf_static`に配信し、
中央ノードなしの単機`bringup.launch`でも表示できます。中央からの重複配信はありません。
姿勢は`fc`と同じで、物体の法線方向を表すフレームではありません。

既定の`contact_offset: [0.0175, 0, -0.161]`はBeeの飛行基準`fc`からの変位です。
URDFの`base_link`からは`[0, 0, -0.107] m`に相当し、4個のゴム球の無圧縮時の中心を
まとめた代表点です。球表面の先端、ばねの圧縮を反映した点、実接触点/CoPではありません。
既存の`beeN/contact_point`は旧結合処理の前方26 cmの点なので、今回のTFと区別します。

位置の定義は`config/contact_point.yaml`で一元管理し、URDFと`cooperative_grasp.launch`が
共用します。独自`controller_config`でもこのgeometry値を最後に読み込みます。
形状を変える際は同ファイルを変更してモデルと中央を再起動してください。
中央の位置計算は従来どおり`p_base + R_base * contact_offset`です。
固定TFの存在は接触成立や自己位置の鮮度を示しません。world座標で使う際は別途、
機体姿勢・位置のTFが必要です。

```bash
rosrun tf tf_echo bee1/fc bee1/grasp_contact_point
```

RVizではFixed Frameを`beeN/fc`（位置推定のTFがある場合は`world`も可）、TF表示の対象を
`beeN/grasp_contact_point`にします。固定リンクに質量・衝突形状はなく、制御点の座標・QP・下流制御は変えません。

## MuJoCoの接触設定

協調把持シーン生成の既定値は`contact_impratio: 500.0`、`friction_cone: elliptic`、
`constraint_solver: Newton`です。シーンYAMLの明示値で上書きできます。
`impratio`は摩擦方向と法線方向の接触拘束の硬さの比であり、摩擦係数ではありません。
2026-11-14の三角柱比較では、制御と摩擦係数を固定して50から500へ変更すると、
接触代表点のずれの速度が0.355から0.0349mm/sへ減少しました。微小滑りは残り、
実物との接触特性の一致や、全条件での無滑りを保証する設定ではありません。
球と物体のgeomが同じpriorityの場合、MuJoCoは大きい側の摩擦係数を使います。
標準の足球・物体の滑り摩擦係数とQPの設計係数はすべて1.0です。

以下の加速度・Preload整合改修より前に行った、採用後の既存10条件・各1回のMuJoCo試験では、三角柱のみ運搬・載置・解放まで完遂しました。
三角柱の平均接触代表点ずれは最大4.83mm、球側面は2.60mmでしたが下降中に参照拒否で停止しました。
他条件にはPreloadの個別支持容量不足、指令加速度と受信上限の不整合、QP精度不足、
押付け中/把持前の物体転落が残っています。全形状の無滑り・安定運搬を確認した結果ではありません。
各条件の失敗も含む記録は、作業記録フォルダの`logs/2026-11-14-contact-model-adoption/`にあります。

## 物体PID・レンチ配分（wrench_control）

### 制御の境界

2026-11-15に保存した下流比較基準は作業ログ側の`logs/2026-11-15-bee-control-baseline/`にあります。
2026-11-16のユーザー指示でインターフェースを汎用化し、接触固有の判断を中央へ移しました。
力PI/フィルタ/減衰/推力枠の標準値は維持し、共通指令の受信処理はBeeController内で動作します。
`force_integral_enabled=false`は積分をゼロに戻す仕様を維持します。

接触開始・荷重移行・離床の共通QP化、台反力の扱い、観測接線力から法線下限を作る改修は
別の設計課題として未実装です。今回Releaseの目的関数や解放手順は変更していません。

`config/grasp/controller.yaml` の `wrench_control: true` は、支持移行以降に次の経路を使用します。

1. 中央の物体位置/速度PIDと姿勢/角速度PDから、物体の必要合力・重心まわりモーメントを作ります。
   位置目標はタスクの運搬軌道、姿勢目標は支持開始時の物体姿勢です。
   Unload前は、鉛直位置・速度PDが下向きの補正を要求すると、支持割合×全Bee質量×その補正加速度を目標鉛直力に加えます。
2. 代表接触点の力を凸QPで配分します。力/モーメント残差、押し付け量、前回からの変化をコストとし、
   法線力上下限、保守的な摩擦四角錐、観測法線力から見た接線力上限、力の変化率、
   自機重力/慣性分を含む推力のL1上限を制約にします。OSQP（依存パッケージ`osqp-eigen`）で解き、
   収束しない/制約違反の解は送信せずfaultとします。物体レンチは軟制約で、実現不能量を監視します。
   解放時は内力が残らないよう、法線力の上限そのものをrelease_rateでゼロへ下げます。
   台へ荷重を戻した後のReleaseでは、目的関数を接触力の二乗和の最小化へ切り替えます。
   力を打ち消し合う保持状態ではなく残力ゼロを目指し、法線上下限・摩擦・推力・変化率制約は維持します。
3. 中央で`-normal_force * normal + support_vector`を3次元の`BeeCommand.force`へ組み立て、
   Bee重心の同伴加速度を`navigation.target_acc_*`で送ります。角加速度と向心加速度、
   機体側の接触オフセットを含みます。配分後の実現レンチと加速度を整合させます。
4. 共通指令の`enable_force_control`を有効にすると、各Beeは配分力FF＋方向別力誤差P＋制限付き積分をACCへ変換します。
   標準の協調指令ではStagingから離脱までPIを有効に保ち、フェーズに応じて力目標を変更します。
   observerはBeeが受ける力なので、力誤差は `commanded_contact_force + observed_external_force`。
   通常飛行制御による自己重力補償は重ねません。旧observer_blendによる全量外力補償は使いません。

`velocity` は測定した物体の接触点速度です。独立の目標速度追従には使わず、監視と
`velocity_damping`による相対運動の減衰だけに使用します（0で減衰を無効化）。
下流の`force_kp`は加速度/N、`force_ki`は加速度/(N s)、積分上限は加速度単位です。

接触前は法線に沿う接近軌道を追従し、探索・接触保持の押し付けは力目標で与えます。Preloadは下記の保持計画QPを使用します。支持移行は床反力が
残るため、部分荷重を自由落下加速度として扱いません。支持後も観測法線力の余裕を見て配分します。
物体PID積分はレンチ残差/加速度飽和で凍結、ローカル積分は出力飽和で凍結し、支持成立前・Unload以降・モード離脱でリセットします。
高さ参照は移動する下降目標に対しても速度を連続化し、送信する加速度を参照速度の差分と一致させます。

`metrics` の既存9要素に、力残差[N]、モーメント残差[Nm]、QP最大制約違反を追加しています。
さらにindex 12〜16にOSQP反復数、セットアップを含む計算時間[s]、primal残差、
KKT停留残差、相補性残差を追加しています。力制約の残差は元の単位で検査します。
生成manifestの `object_inertia` は実際に生成したXMLから取得します。新方式で欠落すると起動拒否します。
中央・各Bee・メッセージを同時にビルドしてください。旧版との混在はサポートしません。

制約: 代表点接触モデルであり、4球のCoP/接触トルク分布やロータごとの推力配分制約は解いていません。
推力L1上限は並進側の保守的な上限で、姿勢モーメントの可用性まで保証しません。
observerに混ざる空力/推力モデル誤差は残ります。接触剛性と遅れを含む安定性は試験で確認が必要です。

### 加速度上限とPreload配分の整合（2026-11-14）

中央とBeeは`referenceMotionValid`で同じ速度・加速度ノルムの受付条件を検査します。
物体の並進/角加速度を6変数の小QPで調整し、全Beeの接触点/重心オフセットと
測定角運動を含む同伴加速度を制限します。線形QPを維持するため、Euclidean球の内側の
26面の多面体を用います。各法線は{-1,0,1}³の非ゼロ方向を正規化したもので、面距離を
0.88645×(加速度上限−5e-5 m/s²)とすることで全頂点を受付球内に収めます。Beeの受付上限0.1 m/s²は変更しません。
空中で配分残差から同伴加速度を修正する場合も、その最終加速度の制約を力配分QPへ入れます。
力を配分した後に個別の加速度だけ切り詰める方式ではありません。実現不能/送信前不適合は停止します。

Preloadでは、6自由度の保持レンチを等式制約とする計画QPから各Beeの法線目標を決めます。
現在の法線/作用点、押付け/推力上限、固定誤差余裕を使用し、`preload_force_ratio`（0.9）まで
観測力が到達した時点でも予定接線力を支えられるよう計画側にも制約します。旧`force_offsets_`で
計画後の押付けを増減する処理はwrench経路で無効です。Beeごとの接触足roll/yaw調整は保持しています。
計画QPはまだ実測力の発生していない段階でも解けますが、実行は観測容量に制限されます。

Preload中に送るのは増加率を制限した法線目標で、予定接線支持はまだ印加しません。
全機が個別目標と、同じ計画ベクトルの観測容量条件を一定時間満たしてから荷重移行へ進みます。
目標到達は計画時に保存した法線スカラーと比較し、新しい法線で前周期の目標値を定義し直しません。
台が残りの重量を支える段階を自由落下として扱わず、接線力を含む実配分は実測法線容量と
前回の力からの変化率を守ります。解放時は個別の法線下限も減らし、内力を取り除きます。

実装/ビルド済み。C++単体テスト56件はすべて成功。固定版でのMuJoCo 10条件試験は6条件が完遂し、4条件に未解決事項が残ります（下表）。
開発途中の失敗・中断も含む証跡は作業記録の`logs/2026-11-14-motion-preload-fix/`に保存しています。
`preload_diagnostics`は各Beeについて[計画法線目標, 指令法線, 観測法線, 支持容量slack,
旧再射影による目標差, 保存目標との差]をN単位で並べます。試験runnerは`use_sim_time=true`を
確認してから初期化し、起動競合でwall timeとsim timeを混在させないようにしています。

力配分QPではOSQPの初期rhoを0.001とし、荷重移行中の弱い内力正則化で双対収束が停滞した記録入力を
回帰テストに含めています。250反復で停滞した場合だけ、Cholesky因子による変数変換と制約行の正規化を行い、
同じ目的関数・制約を別の座標で解き直します（再計算の初期rho0.1）。2回を合わせて4000反復と10msの残予算を共有します。
一方、対角の目的行列を持つ保持計画・運動制限・解放は、元の座標のままrho0.1・最大4000反復で解きます。
これらでは追加の座標/行正規化を行わず、力配分の弱い内力方向とは数値設定を分けます。
OSQP 0.6の初回時間判定は準備時間を含むため、その時間基準に換算し、準備時間を二重に差し引かないようにします。
正規化する行の倍率に合わせてソルバの許容誤差も厳しくし、元単位のprimal残差が膨らまないようにします。
変数・双対変数・primal残差を元の単位に戻し、OSQP_SOLVED または OSQP_SOLVED_INACCURATE の候補について、元単位の制約違反・primal残差・停留性・相補性の全条件を満たしたときだけ受理します。
再計算回数はmetrics index17です。運動投影側は配分側よりさらに2e-5 m/s²内側とし、
前段の数値残差が後段の定数制約を実行不能にしない余裕を持たせています。
任意の環境変数`BEE_GRASP_QP_FAILURE_FILE`を設定すると、失敗時だけ問題行列を指定ファイルへ
追記します。通常は無効で、再現用の隔離シミュレーションでのみ使用します。
実機試験・書込みは実施していません。

### 2026-11-14の固定版試験結果

`acceptance-07`はコード・設定・バイナリを固定し、各条件1回、合計10回を実施しました。
全試行で入力hash不変、設定は改修前と同じで、接触`impratio=500`です。
起動失敗だけ1回再起動する試験規則を設けましたが、この一巡では再起動は発生していません。
実機試験ではありません。開発中の成功試行だけを集めた成績でもありません。

| 条件 | 結果 | 到達点・停止理由 |
|---|---|---|
| 三角柱 | Complete | 1.5 m運搬・載置・解放 |
| 五角柱 | Complete | 0.4 m運搬・載置・解放 |
| 円柱 | Complete | 1.5 m運搬・載置・解放 |
| 箱 | Complete | 0.4 m運搬・載置・解放 |
| 球・側面把持 | Complete | 1.5 m運搬・載置・解放 |
| 楕円体 | Complete | 1.5 m運搬・載置・解放 |
| 円錐台 | Aborted | 荷重移行開始時の力QPが4000反復で未収束（status−2） |
| 岩 | Aborted | 荷重移行開始直後の力QPが4000反復で精度不足（status2） |
| 球・下側把持 | Aborted | 持上げ中に物体yaw上限。QP・送信制約監査は通過 |
| カプセル | Aborted | 支持移行時に物体傾き上限。接触前の台上段階で既に約40°傾いていた |

完遂6条件の力配分QPは最大350反復・3.612 msでした。送信されたwrenchモードの
同伴加速度は全記録で最大0.096951 m/s²で、受付上限0.1 m/s²以内でした。
完遂6条件の力/摩擦余裕/押付け/推力/速度/加速度と元単位KKT監査はすべて成功しました。
Preloadの観測入力でも、各機の目標到達・支持容量が約3秒連続して成立したこと、
その段階の接線支持指令がゼロだったことを確認しています。

完遂6条件の最大個別接触代表点ずれは2.30〜4.98 mm、運搬中の平均鉛直ずれの傾きは
0.0237〜0.0349 mm/sでした。これは代表点の幾何計測で、真の接触パッチ/CoPの滑り計測ではなく、
「滑りがゼロ」「全形状で安定」を実証した結果ではありません。

未解決の中心は岩・円錐台の支持移行直後の数値収束、球下側のyaw抑制、カプセルの台上初期配置です。
岩の記録ではPreload終端の送信合力Fzが−2.271 N、支持移行の最初の目標が+0.026 Nであり、
希望レンチの移行と観測力による進行条件の整合は次に調べる候補です。
力の変化率制約は維持しており、これが未収束の直接原因であるとの因果確認はまだしていません。

証跡は作業記録プロジェクトの`logs/2026-11-14-motion-preload-fix/aggregate.json`、
`acceptance-07/`内の各bag/summary/解析、`implementation.patch`、`gtest.xml`に保存しています。
全試験master/シミュレーションプロセスの終了を確認済みです。変更は未コミットです。

### 支持開始の目標連続化と追加試験（2026-11-14）

Preload終了時の指令接触力から合力・モーメント`w_start`を保存し、支持の進行率
`support_ratio`が0から1になる間、`(1-support_ratio)*w_start`を目標レンチへ加えます。
これにより、傾斜法線で下向きの合力がある状態から、ゼロ付近の支持目標へ飛ぶのを防ぎます。
物体の運動補正は従来どおり加え、進行率1では通常の重力支持/運動レンチになります。

観測力による進行ゲートも、同じ開始合力を基準にします。`mg-F_start_z>0`の場合、
許される進行率は`(filtered_Fz-F_start_z+support_force_lead)/(mg-F_start_z)`です。
支持開始時に、それまで休止していた力フィルタを現在の観測値で初期化します。
既存の1.5秒のrampと0.5 Nの先行量、摩擦/観測容量/推力/加速度/力変化率制約、
OSQPの設定・精度・4000反復/10ms予算は変更していません。

C++は負の開始合力での連続性、観測が止まったときの進行抑制、追従再開後の支持完了を
追加確認し、57件全成功。`candidate-01`は岩/円錐台0.5 kg、各1回、設定/コード/バイナリを
固定したMuJoCo試験です。前記`acceptance-07`とは別の版・別の成績です。

| 条件 | 追加試験の結果 |
|---|---|
| 岩 | Complete。支持移行6.261秒、1.5 m運搬・載置・解放まで完遂。全力QP最大400反復/3.876ms |
| 円錐台 | 開始時の停止は通過したが、支持移行5.821秒後、進行率約0.987で別のQP未収束。4000反復/8.321ms |

円錐台の新しい停止入力は目標力増分約0.0198 Nで、目標レンチの完全一致を含めても
実行可能でした。同じ目的/全制約を別ソルバで解くと元単位のKKT条件も満たします。
したがって目標連続化だけで収束問題全体が解決したとは言えず、数値解法側の課題が残ります。
岩の最大個別接触代表点誤差は5.69 mm、運搬中の平均鉛直ずれ傾きは0.0337 mm/s。
真の接触パッチの滑りや、反復試験での成功率を示す値ではありません。

実装・ビルド済み、未コミット。実機操作/書込みなし。証跡は作業記録プロジェクトの
`logs/2026-11-14-support-target-ramp/`（before、target_probe、gtest.xml、candidate-01内の
bag/summary/transfer_analysis/failure_feasibility）に保存しています。

### 固定値の誤差余裕

オンラインの推力上限は固定値とし、`controller.yaml` から次の値を読みます。
いずれも初期のシミュレーション用設計値で、実機の誤差上限を同定した結果ではありません。

| パラメータ | 設定値 | 意味 |
|---|---:|---|
| `allocation_normal_error` | 0.5 N | 実接触力と指令の法線成分の最大誤差 |
| `allocation_tangent_up_error` | 0.1 N | 世界Zを接平面へ射影したup方向の最大誤差 |
| `allocation_tangent_side_error` | 0.1 N | upに直交する接線side方向の最大誤差 |
| `allocation_observer_normal_error` | 0.5 N | 観測法線力の上振れ幅。指令との誤差とは別の条件 |
| `allocation_thrust_limit` | 32 N | 自重/自機加速/接触/補正を含む、世界座標での推力L1上限 |
| `allocation_thrust_reserve` | 3 N | Beeの力P/I/相対速度減衰が使う追加推力L1枠 |

接触座標で押し付けをf、接線をt_up/t_sideとすると、4本の摩擦制約を
`±t_up ±t_side - mu*f <= -(mu*epsilon_n + epsilon_up + epsilon_side)` に縮めます。
観測容量側も `±t_up ±t_side <= mu*max(0, observed_normal - observer_error) - epsilon_up - epsilon_side`
とし、右辺が負の場合を0へ丸めて成立したように扱いません。法線の指令上限は
`max_normal_force - allocation_normal_error`。Preload指令に含まれる誤差を二重に足さず、力変化率を守って到達します。
この条件は既知の代表法線に対する力誤差のモデルです。法線角度・接触点・CoPの不確かさは含みません。

荷重移行中から下降までは全誤差幅を使います。接地後のUnloadでは床へ荷重を戻す
`support_scale` に合わせて接触力誤差の制約余裕を減らし、Releaseで0にします。
接触を維持するロバスト条件を、接触力を0へ解放するフェーズに課し続けないためです。
これは床へ支持を移す際の制約の切り替えで、実際の観測誤差が小さくなるという仮定ではありません。
物理的な法線上限側の余裕、観測法線の上振れ幅、推力予約は維持します。
Preloadより前の接近経路は従来方式です。起動時は幾何から計算した初期押付けと法線誤差が上限内に収まるかも検査します。実際の保持計画は接触幾何を用いてPreloadで再計算します。

Preloadの設計荷重は1機あたり`m*(g+max(task.design_vertical_acceleration,
controller.reference_acceleration_limit))/N`とし、摩擦係数はtaskとQPの小さい方を使います。
垂直な面では共通指令を`max(normal_error,observer_error) +
(normal_force_margin*design_load + tangent_up_error + tangent_side_error)/mu`とします。
標準の三角柱では約5.349Nです。傾いた面では内向き法線力の鉛直成分も考慮し、
計画接触法線ごとの必要値の最大を使います。これは等分鉛直荷重の準備であり、
任意配置での6自由度つり合いや全アクチュエータ能力を保証する起動QPではありません。
Preload完了には従来の各機目標に対する観測比率・継続時間に加え、現在の各接触法線で
指令側のロバスト容量と観測側の保守的容量が設計荷重を支えられることを要求します。

OSQPは毎回現在の行列をスケーリングし、前回の力を初期値にします。polishingを有効にし、
絶対許容1e-6・相対許容0・最大4000反復・ソルバ時間上限10msです。
`SOLVED`以外は拒否し、元の行列でも制約違反/primal<2e-5、KKT停留/相補性<1e-4を確認します。
設定上限だけでは実行周期を保証しないため、`metrics`の実計算時間も試験で確認します。

QPの基本推力枠は `allocation_thrust_limit - allocation_thrust_reserve` です。
空中では配分残差から求める同伴加速度の修正も力変数のアフィン関数として制約に含め、
送信する最終加速度で予約が残っていることを再確認します。Bee側はP/I/相対速度減衰の
合計を同じ予約枠へ制限し、制限中は積分を凍結します。予約0はこの追加補正を0にする意味です。
既存の各方向加速度制限と半径保護も残し、最終ACCは重力を含む総推力L1上限へ制限します。
保護/飽和作動時に指令接触力の追従や上記誤差幅の成立を保証するものではありません。

この固定L1枠が全姿勢でロータ実現可能領域に内接することは未検証です。
機体姿勢ごとの能力評価や接触位置探索は行わず、物体レンチの誤差を厳密に0と保証する制約にもしていません。
中央の`config/grasp/controller.yaml`と各Beeの`config/controller/force_control.yaml`で、機体質量・総推力上限・追加補正の予約枠を一致させてください。実接触力の誤差幅はobserver追従残差だけから決められません。

Preload/OSQP修正前の固定余裕版MuJoCo三角柱試験（3機・0.5kg・1.5m）は235.966秒でComplete、
最終XY誤差1.95mmでした。C++41件成功。送信指令のロバスト摩擦・法線上限・推力予約と、
Beeの指令総推力をbagから検査しました。基本推力の最大L1値26.1904N、残り5.8096Nで、
予約3Nを確保。Beeの指令総推力の最大L1値は26.2401Nでした。
最大QP制約違反は約2e-5で既存の数値許容内です。最終版の通し試験1回の結果で、
誤差幅の同定、実機での保証、方式間の性能優位性の検証ではありません。
証跡は作業記録側の `logs/2026-11-14-fixed-margins/` に保存しています。
同版の追加3試験は1回Complete、2回Aborted（接触代表点ずれ/ADMM収束未達）でした。
この反復結果を受けて上記Preload/OSQPを修正しました。旧結果は現行版の試験結果として扱いません。

### 2026-11-14の三角柱試験

MuJoCoで3機・0.5kgの三角柱を1.5m運搬し、載置・解放・退避・Verifyまで通過して
207.460秒にCompleteとなりました。最終XY誤差は3.04mmです。最終版での通し試験は1回で、
実機試験や他形状の再検証は行っていません。C++35件、シーン生成Python13件のテストに成功しました。

開発中の7回はQP収束、荷重移行、下降参照、接触ずれ、解放残留などでAbortedでした。
修正内容と全試行は作業記録側の `logs/2026-11-13-wrench-control/` に保存しています。
安全閾値を緩めず、接地中の加速度変換、支持開始前の力積分、連続な下降加速度、
解放時の法線上限を修正して最終試験を実施しました。

以下の鉛直支持/observerブレンドの説明は `wrench_control: false` の旧経路です。

## 曲面接触の試験用拡張

この旧経路の法線/接平面計算は、現在は中央の`grasp_controller.cpp`で行います。
下流には合成した汎用の加速度指令を送り、法線や支持力の意味づけを持たせません。
メッセージを変更しているため、中央・各Beeは同じ版でビルドしてください。

中央はプリズムでは物体姿勢を含む面法線、球／楕円体では代表接触位置での
解析的勾配を使います。押し付け力の鉛直成分を差し引き、残る鉛直荷重を
接平面へ配分します。これは鉛直支持配分の拡張であり、任意配置の6D制約付き
wrench最適化・外乱余裕最大化・接触点探索を実装したものではありません。
極付近の接平面支持能力が不足する配置は拒否します。現在の面内モーメント補正、
機体の接触姿勢、複数の足の接触を1代表点とする近似には引き続き制約があります。

球／楕円体は4機、0.5 kg、接触開始高さは物体中心から-0.025 mの試験設定です。
3rdpartyの`mujoco_scene_composer.py`のsphere/ellipsoid対応も必要です。

```bash
roslaunch bee grasp_demo.launch robot_count:=4 scene_config:=$(rospack find bee)/config/mujoco/scene_sphere.yaml
roslaunch bee grasp_demo.launch robot_count:=4 scene_config:=$(rospack find bee)/config/mujoco/scene_ellipsoid.yaml
```

隔離したMuJoCoの回帰試験（実機masterには接続せず、未使用ポート・新規出力先を指定）:

```bash
python3 $(rospack find bee)/test/run_grasp_shape.py triangle --port 11451 --output /tmp/bee-triangle-new-run
```

`triangle`を`sphere`、`ellipsoid`、`cylinder`、`frustum`、`capsule`、`rock`、`sphere_support`へ変更できます。判定・終了時の位置情報は
`summary.json`、時系列は`flight.bag`、起動ログは`launch.log`に保存します。
`Complete`以外は成功とみなしません。これらはシミュレーション専用・開発用設定で、
実機への適用や曲面把持の安定性保証を意味しません。

### 2026-11-12の試験結果（開発中）

最終版のMuJoCo試験では三角柱と球が運搬・載置・解放までComplete。
楕円体（0.7×0.6×0.5m、短軸鉛直）は把持・運搬・載置・解放を通過しましたが、
荷重抜き〜解放中の位置ずれで最終XY誤差34.6mmとなり、30mmのVerify条件を満たせず
Abortedです。楕円体の全工程成功とは扱わないでください。以前の中間軸鉛直の配置では
解放後に転がり落ちたため、載置面と静止姿勢の選択も必要です。
起動時のEigen行列積assertは2026-11-13にカルマンフィルタのバイアス推定モード切替と
IMU予測の競合と特定し、`kalman_filter`の切替・予測・getterを同期する修正を実施しました。
関連9パッケージを再ビルドし、並行回帰5件×20周、4機の起動5回、下記3形状試験で
同じassertが再発しないことを確認しています。別リポジトリ`kalman_filter`の変更も必要です。

### 追加形状（2026-11-13、試験結果は下記参照）

円柱、円錐台、横向きカプセルを追加しました。いずれも4機・0.5kg・1.5m運搬設定です。
RVizは以下の入口で標準起動します。飛行中の別シミュレーションと重ねて起動しないでください。

```bash
roslaunch bee grasp_demo.launch robot_count:=4 scene_config:=$(rospack find bee)/config/mujoco/scene_cylinder.yaml
roslaunch bee grasp_demo.launch robot_count:=4 scene_config:=$(rospack find bee)/config/mujoco/scene_frustum.yaml
roslaunch bee grasp_demo.launch robot_count:=4 scene_config:=$(rospack find bee)/config/mujoco/scene_capsule.yaml
```

- 円柱: 半径0.30m、高さ0.60m。側面の半径方向法線を使用。固定円柱台座と異なり、運搬物には`free: true`を明示。
- 円錐台: 下半径0.34m、上半径0.26m、高さ0.60m。傾斜した側面の法線を使用。物体poseと配置高さは一様密度の重心基準。MuJoCoは128角形の凸meshで円周を近似し、制御法線・慣性は解析的な円錐台を使用（半径0.34mで円弧と弦の最大差は約0.10mm）。
- カプセル: 半径0.25m、全長0.90m（円筒部0.40m）、物体X軸に横向き。円筒側面と両端の半球に対応した法線を使用。`object_height`は直径と一致させ、`object_length`は半球を含む全長。

中央が物体座標系の法線とsupport-plane距離を計算し、既存の3D参照として各Beeへ送ります。
円柱・円錐台の対象は側面把持であり、底面/天面・稜線を使う把持は対象外です。
カプセルは端部も含む解析的な表面を使用しますが、接触点の自動探索はしません。
新3形状はmanifest v2を使い、未対応の旧ノードは起動時に拒否します。
既存プリズム・球・楕円体のmanifest v1も引き続き読めます。
本体のビルドと3rdpartyのscene composer更新が必要です。新形状の試験設定を追加したことと、
把持・運搬・解放の通し成功を確認したことは別です。楕円体の解放精度は未解決です。

### Eigen修正後の追加形状の運搬試験

- 円柱: 196.725秒でComplete。1.5m運搬・載置・解放・退避を完了し、最終XY誤差10.07mm。
- 円錐台: 目的地まで運搬したが、170.098秒のDestinationSettleで`mean contact slip limit`によりAborted。停止直前の物体Zは0.973390mで目標近傍を保持しており、落下が先に起きたのではない。接触配置の相対ずれが蓄積した。載置・解放工程は未実施。
- カプセル: 73.025秒に`object tilt limit`でAborted。記録phaseはPreload→Abortedで、コード上は荷重移行に入る最初の安全判定で停止。把持開始前の約20〜25秒から横ずれとbody-X軸回転が発生し、停止直前のrollは約-31.2°。持ち上げ・運搬は未実施。

3試験ともEigen assertなし。成功条件や安全閾値を緩和していません。各形状1回の結果であり、
円柱も繰り返し成功率・外乱耐性の保証ではありません。円錐台の相対ずれ、カプセルの開始前の
回転・配置/離陸干渉の切り分けは次の課題です。

### 岩形状と球の接触高さ比較（2026-11-13）

`scene_rock.yaml`は高さ0.60m、質量0.50kgの非対称凸多面体です。
`grasp_geometry.rock_geometry()`の固定15半空間から26頂点/48三角形を生成します。
広い接触面4枚と平らな底面を残し、辺と上下を非対称に面取りしています。
重心・full inertiaを体積積分し、物理mesh、RViz、制御のsupport-planeを同じ座標に揃えます。
manifest v3の`surface_vertices`と`contact_normals`はxyzを平坦化した配列です。
制御は計画接触面の法線を使い、辺を越えたら別面へ自動切替する方式ではありません。
未知の岩を認識して接触点を選ぶ機能や、凹形状への対応は含みません。

`scene_sphere_support.yaml`は既存球と同じ4機・半径0.30m・0.50kg・1.5m経路で、
接触高さだけを-0.025mから-0.120mに変更しています。
下半球へ横から接触し、内向き法線力の上向き成分を支持に使う比較です。
真下に水平な台を差し込む支持ではなく、両条件とも押し付け力を持つ把持です。
ゲイン、摩擦、安全閾値は同じです。形状ごとの反復試験は実機試験ではありません。

```bash
roslaunch bee grasp_demo.launch robot_count:=4 scene_config:=$(rospack find bee)/config/mujoco/scene_rock.yaml
roslaunch bee grasp_demo.launch robot_count:=4 scene_config:=$(rospack find bee)/config/mujoco/scene_sphere_support.yaml
# 標準7条件の反復（2026-11-15から円錐台を除外）: 未使用port群、新しい出力ディレクトリが必要。
python3 $(rospack find bee)/test/run_transport_repeats.py --repeats 10 --workers 3 --port 11501 --output /tmp/bee-repeat-new
python3 $(rospack find bee)/test/summarize_transport_repeats.py /tmp/bee-repeat-new
```

`protocol.json`に設定・ソース・バイナリのハッシュ、`schedule.json`に固定seedで混ぜた実行順、
`results.json`に失敗を含む全試行、`aggregate.json`に条件別集計を保存します。
成功はCompleteのみで、位置誤差の成功群集計にAborted後の落下位置を混ぜません。
接触ずれは`norm(mean(error_i))`と`max(norm(error_i-mean(error_i)))`を分け、
支持力分担はHold/Transport/DestinationSettleの指令値から集計します（実接触力の直接計測ではありません）。
同一初期条件で10回の再現性を調べる試験であり、外乱耐性の保証や一般的な方式の優劣を示すものではありません。

球の物理接触力は、別の後処理で`mujoco/grasp_contact_states`のtouch値、脚圧縮量、
FCのground truthと物体poseから復元します。球中心から各脚球中心への方向を法線とし、
各脚の法線力の鉛直成分を計算します。サンプル間隔50ms以下に限定し、4機それぞれの
時間サンプル平均を合計します。指令の代表接触点で計算した支持力とは区別します。
touchは接触法線力のスカラー和であり、この解析は接線力や外乱余裕の直接計測ではありません。
物理的な複数脚の荷重分担は、1機1代表点による制御モデルと一致するとは限りません。

起動前の失敗も全試行の分母へ含め、把持開始（Staging）後の成否を併記します。
全機HOVER待ちの失敗、仮想FCの初期化・通信監視、解放後の状態鮮度監視は、
接触滑りや物体落下とは別に分類します。3並列で実施した反復群から、単独実行時の
成功率や並列負荷との因果関係を断定することはできません。

### 各10回の反復結果（2026-11-13）

04:53〜06:32 JST、上記8条件×10回=80試行を完了。質量0.50kg、運搬距離1.50m、
三角柱は3機、それ以外は4機。3個の独立したlocalhost masterでheadless MuJoCoを
並列実行しました。固定seed 61113は試行順のシャッフル用で、外乱は加えていません。
予備試験は10回に含めず、失敗した回を成功するまでやり直すこともしていません。
計測対象81ファイル（設定・ソース・実行バイナリ）の開始/終了ハッシュは一致。
全80回でEigen assertion・segfault等の検出0、解析失敗0。Completeは計47回です。

成功は把持、運搬、載置、解放、退避、最終VerifyまでのCompleteです。
下表のXY誤差は**成功した回のみ**の平均と範囲で、失敗を含む全試行の精度ではありません。

| 条件 | Complete / 全試行 | 運搬開始 | 成功時XY誤差 平均［範囲］mm | 未完遂の内訳 |
| --- | ---: | ---: | ---: | --- |
| 三角柱 | 9/10 | 9/10 | 6.37［1.12–11.61］ | 起動前1 |
| 球・側面 | 7/10 | 7/10 | 4.89［1.23–11.73］ | 起動前2、Approach中の物体入力監視1 |
| 楕円体 | 3/10 | 10/10 | 29.17［28.83–29.40］ | 解放後Verifyの位置条件7 |
| 円柱 | 9/10 | 10/10 | 6.67［3.29–14.10］ | 解放・退避後の状態監視1 |
| 円錐台 | 0/10 | 10/10 | — | 目的地付近で共通接触配置ずれ上限10 |
| カプセル | 0/10 | 0/10 | — | 傾斜上限4、Staging timeout4、起動前2 |
| 岩風凸多面体 | 10/10 | 10/10 | 12.95［9.83–14.63］ | なし |
| 球・下側 | 9/10 | 9/10 | 6.05［4.05–7.97］ | 起動前1 |

- 岩は同じ固定形状で10回成功。未知の岩、ランダムな形状群、自然石の密度・摩擦を検証したものではありません。
- 楕円体は全10回で運搬・解放まで通過。失敗7回の最終XY誤差は30.14–35.97mmで、30mm条件を超えました。成功3回も閾値近傍です。
- 円錐台は9回がDestinationSettle、1回がTransport末尾（目的地まで約10mm）で停止。`norm(mean(error_i)) > 0.06m`の判定であり、載置/解放前です。停止後の降下を停止原因の落下と混同しません。
- カプセルは荷重移行開始時の傾斜上限または接近準備で停止。body-X軸対称形状の回転を現在の傾斜監視がどう扱うか、離陸前後の初期配置/干渉が未解決で、運搬不能を証明したわけではありません。

#### 球：挟む位置を下げる効果

比較は赤道近傍（計画接触高さ-25mm）と下半球（-120mm）です。
両方とも内向きの押し付けを伴い、下側条件も「真下に台を置くだけ」の支持ではありません。
運搬まで到達した側面7回・下側9回は全てComplete。起動/入力監視の失敗を含む7/10対9/10だけで、
把持方式の力学的な成功率の差と判断することはできません。

| 指標 | 球・側面（保持データ7回） | 球・下側（保持データ9回） |
| --- | ---: | ---: |
| 全脚の法線力合計の平均 | 11.55 N | 11.72 N |
| 法線力の鉛直成分合計（上向き正） | **-1.28 N** | **+1.93 N** |
| 上記の物体重量4.905Nに対する割合 | -26.0% | +39.4% |
| 共通接触配置ずれ：試行最大値の平均 | 40.96 mm | 25.44 mm |
| 差動接触配置ずれ：試行最大値の平均 | 2.18 mm | 6.12 mm |
| 物体傾斜：試行最大値の平均 | 1.18° | 2.32° |
| 成功時XY誤差の平均 | 4.89 mm | 6.05 mm |

力はHold/Transport/DestinationSettleのtouch＋幾何復元、配置ずれ/傾斜は
SupportTransfer〜Descendの値です。複数脚を1代表点とする指令上の配分値ではありません。
各回の4機平均を合計した後、試行間を等重みで平均しています。

下半球では内向きの法線力が上向き成分を持つため、重さを接線摩擦で支える負担を減らせます。
今回の側面条件では上側の足（left、平均で球中心より約64mm上）に法線荷重が偏り、
押し付け力の合計がむしろ下向きでした。下側条件では同じ足も平均約38mm下に移り、
約1.93N、重量の約39%が上向き法線成分になっています。単に「側面だから常に下向き」なのではなく、
この複数脚の接触分布による結果です。下側でも残りの重量支持には摩擦等が必要です。

**重力支持の摩擦負担と共通接触ずれでは下側が有利**という結果です。ただし差動ずれ・傾斜は
下側の方が大きく、XY誤差の分布も重なっています。低摩擦、重心ずれ、突風/外力、上向き外乱、
荷重上限を掃引していないため、全方向の外乱余裕や一般的な優劣はまだ確定しません。
force closureの成立判定と有限駆動力での外乱余裕は別です。
参考：[Modern Robotics: Force Closure](https://modernrobotics.northwestern.edu/nu-gm-book-resource/12-2-3-force-closure/)。
touchの解釈は[MuJoCo 2.3.7 sensor/touch](https://mujoco.readthedocs.io/en/2.3.7/XMLreference.html#sensor-touch)に従います。

#### 起動・状態監視の失敗と再現資料

把持開始前の6件は、仮想FCのmotor数0が2件（三角柱/球）、離陸時の初期XY差超過が2件
（球/カプセル）、仮想FC指令監視のfailsafeが2件（球下側/カプセル）。
球sphere-04はApproach中に物体poseの入力監視で停止し、Preloadへは進んでいません。
bagには停止付近で時計より60ms未来のheaderがあり、50msの未来時刻判定に触れた可能性がありますが、
中央の判定時receiptは未記録なので確定ではありません。円柱cylinder-04は解放・退避後のVerifyで
bee1のlocal state missing/stale/not HOVERにより停止。これらを形状の滑りと分けて集計しています。
試験中に起動処理や監視閾値を変更していません。

原データと後処理は`/home/kitagawa/ドキュメント/ChatGPT/bee/logs/2026-11-13-repeat-transport/`へ保存。
`trials/`に80回分のbag、scene、launch/roslog、summary/analysisとprotocol/schedule/completionを保存し、
`aggregate.json`、`failure_details.json`、`sphere_comparison.json`も同ディレクトリにあります。
`results.png`は全試行の成否・成功時精度・球の法線支持成分、`geometry.png`は形状/接触点の図です。
`frozen-inputs.tar.gz`には計測対象81ファイルのスナップショットを保存しています。
実行時の元出力先は`/tmp/bee-transport-repeats-20261113-01`で、JSON内のdirectoryはこの元パスです。
本体develop/bee基点06ec6b75f、3rdparty基点df752f2、kalman_filter基点5c7a410の未コミット変更込みの試験です。
PCビルド/単体試験/MuJoCo検証であり、実機・FW書込み・実機動作確認は行っていません。

## RViz付き自動デモ（シミュレーション専用）

3rdpartyの`develop/hwinterface`を更新し、`catkin build bee mujoco_ros_control`後に実行します。

```bash
source /home/kitagawa/ros/jsk_aerial_robot_ws/devel/setup.bash
roslaunch bee grasp_demo.launch
```

MuJoCo・RViz・C++協調制御を起動し、停止状態の全機を確認、ARM、離陸、全台HOVER確認後に中央のstartサービスを呼びます。標準タスクは把持→運搬→載置を自動実行します。Python補助ノードは起動操作のみで、把持制御はC++です。`headless:=false`でMuJoCo画面も表示、`launch_rviz:=false`でRVizなしにできます。

Ctrl-Cまたは起動待ちタイムアウトでシミュレーション全体を終了します。飛行中の機体、旧デモ、別の`grasp_sim.launch`と重ねて起動しないでください。実機用ではありません。

シミュレーションの起動入口は`grasp_sim.launch`です。MuJoCo、各Beeのbringup、中央制御を別々のlaunchに分けています。中央の`bee_cooperative_grasp_node`がphase管理・物体制御・協調参照生成を行い、各Beeの`BeeController`へ共通の`BeeCommand`で目標速度・加速度と力目標を送ります。BeeControllerが自機の速度とobserverを使って追従・外力補償し、既存の飛行制御ループ内でWORLD/COGの加速度・姿勢目標を適用します。標準起動では独立したtrackerノードや自分宛ての`uav/nav`再送はありません。

```text
grasp_sim.launch
├─ mujoco_world.launch
│  └─ prepare_mujoco_scene.py → exec MuJoCo（共有の1プロセス）
├─ multi_robot_bringup.launch
│  └─ bringup.launch × 台数（各機の引数を直接指定）
│     ├─ aerial_robot_base_node
│     │  └─ BeeController（通常制御／力追従）
│     └─ controller_spawner → attitude_controller
└─ cooperative_grasp.launch
   └─ bee_cooperative_grasp_node
```

`multi_robot_bringup.launch`はシミュレーション用です。実機では従来どおり各ロボット上で`bringup.launch`を個別に起動します。`grasp_real.launch`は作っていません。

## ファイルの入口

| 役割 | 実装 |
| --- | --- |
| シーン生成・物体／台座／初期配置 | `script/mujoco_scene_generator.py` |
| シミュレーション全体の起動 | `launch/mujoco/grasp_sim.launch` |
| 共有MuJoCoの起動 | `launch/mujoco/mujoco_world.launch` |
| 各Beeの起動 | `launch/mujoco/multi_robot_bringup.launch` → `launch/bringup.launch` |
| MuJoCo起動前のシーン生成・パラメータ待機 | `script/prepare_mujoco_scene.py` |
| 中央ノード・ROS入出力・実行ゲート | `src/cooperative_grasp/grasp_node.cpp` |
| phase管理・目標生成 | `src/cooperative_grasp/phase_manager.cpp` |
| 制御の更新順序・支持移行・物体姿勢・目標レンチ・各機体の指令合成 | `src/cooperative_grasp/grasp_controller.cpp` |
| 各Beeの追従・外力補償・モード切替・通信監視 | `src/control/bee_controller.cpp`（`BeeController`が所有） |
| 共通参照を既存の飛行制御へ適用 | `src/control/bee_controller.cpp` → `src/bee_navigation.cpp` |
| 中央→各Bee／各Bee→中央のメッセージ | `msg/BeeCommand.msg` / `msg/ControllerStatus.msg` |
| 外レンチ変換・集約・接触位置・速度推定 | `src/cooperative_grasp/grasp_state_estimator.cpp` |
| レンチ配分とOSQP求解 | `src/cooperative_grasp/wrench_allocator.cpp` |
| 制御状態・出力の監視 | `src/cooperative_grasp/phase_manager.cpp` |
| 状態・設定・目標・指令の型 | `grasp_state_estimator.h`、`phase_manager.h`、`grasp_controller.h` |
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
- 分散追従の`SupportTransfer`では開始時の物体高度を保持し、高度PI速度補正を行います。この段階にも速度・傾斜・接触すべり監視を適用し、保持高度から上下に`max_drop`（標準3 cm）を超えたら中止します。支持完了前の高度ドリフトを基準高度へ取り込まないためです。
- `Hold`への進入は物体とreferenceの高さ・速度の安定で判断します。
- `Lift → Hold → Transport`は同じ制御器instanceを継続し、積分・フィルタ・referenceをresetしません。
- 台座接地を確認すると`Unload`へ移り、接地イベントにより制御器がZ積分と共通slip補正を解除します。
- `task/automatic_transport: false`では`Hold`で待ち、`~transport`で先へ進みます。
- `controller/contact_tangent_control_enabled`は、`Preload`から`Release`までの接線方向PD（面の中央への位置補正と相対速度の減衰）を切り替えます。現在の`controller.yaml`では比較試験のため`false`です。`true`で従来の制御に戻ります。接近・退避時の接線PD、物体yaw補正、共通XY移動、法線力・Z制御には影響しません。ノード起動時に読み込むため、変更後は再起動してください。
- 中止は`Aborted`へラッチし、同じnodeで再開しません。操作者の中止では上記の水平退避を行い、自動異常停止はその場での保持を指令します。LAND/STOP中は操縦モードを維持します。

## 押し付け力と接触高さの設定

以下の1.962Nと静的計算は`wrench_control: false`の従来経路です。
`wrench_control: true`は冒頭の誤差・加速込みPreload計算と追加の容量判定を使用します。

各機の共通目標力は `normal_force_margin * object_mass * (gravity + design_vertical_acceleration) / (robot_count * friction_coefficient)` で計算します。水平な物体の荷重を均等に支持する設計です。物体質量・台数はscene manifestを使用し、設計摩擦係数はシミュレータの接触パラメータから独立した`task/friction_coefficient`で指定します。モーメント補正による各機の力配分は別途従来どおり加算します。

- 標準設定は摩擦係数1.0、余裕率1.2、想定上向き加速度0 m/s²です。500 g・3台なら1.962 N/台になります。想定加速度0は静的支持に基づく設定です。
- 計算値はノードの`~normal_force_target`に公開します。計算値が`controller/max_normal_force`を超える場合は起動エラーにします。
- `task/normal_force`の固定値指定は廃止しました。残っている場合は、無視せず設定移行を求めるエラーにします。
- Preload完了は全機のフィルタ済みobserver法線力が`task/preload_force_ratio * normal_force_target`以上で、`task/preload_duration`だけ連続していることです。標準は90%・3秒（上記条件では1.7658 N以上）。指令値の到達は判定に使用しません。各機のモーメント配分後目標への収束判定ではなく、共通目標に対する各機の最低支持能力の確認です。
- 標準`controller.yaml`では`z_velocity_kp`と`z_velocity_kd`を0にし、支持移行前の接触相対高さPDだけを無効にしています。この間はZ速度0を指令します。支持移行後の物体高度制御・荷重支持・傾き補正は維持しています。旧中央加速度経路では接触高さをそろえるP項を`relative_z_kp: 8.0`に設定しています。既定の分散追従では、この項を使わず、接触配置は診断・安全監視に使います。位置を戻す補正はありません。速度差のD項は`relative_z_kd: 0.0`、共通すべり補正は`slip_kp/kd: 0.0`で無効のままです。相対高さ・すべりの計測と安全判定は継続します。従来のゲインは高さ差が8.0/2.0、共通すべりが5.0/2.0です。設定は起動時に読み込まれます。

## シーンとの境界

`MujocoSceneGenerator`はROS masterなしで、同じフラットなシーン設定からXML・composer YAML・制御用manifestを生成します。旧`mujoco_multi_module.launch`（現在はアーカイブ済み）の設定キーと既定値を継承しています。

`mujoco_world.launch`の起動前処理が生成完了後、manifestを`/bee_grasp/scene`へ一括登録します。起動時は`available: false`にします。中央制御は`~scene_namespace`で指定したmanifestを読み、各Beeのbringupのprivate parameterを直接参照しません。

ROS launchのinclude順には実行順序の保証がないため、`prepare_mujoco_scene.py`をMuJoCoノードの`launch-prefix`に指定しています。この小さな前処理がシーンを生成し、全台のモデル・motor/servo・AerialRobotHWSim設定を待ち、既存の初期化猶予3秒を置いてから`exec`でMuJoCoへ切り替わります。Pythonから子launchは起動しません。各Beeの標準controller spawnerはcontroller managerの出現を待ちます。MuJoCoを含む全ノードの終了処理はroslaunchが担当します。

manifestには`schema_version`、利用可否、機体名、確定した物体質量・寸法・初期pose、各面の角度・中心からの距離、待機距離、目的地の位置が入ります。密度指定時も生成側で確定した質量を渡します。ゲインや描画設定は含めません。物体なし・物体面数と機体数が一致しないシーンも生成できますが、中央制御用manifestは`available: false`になります。

シーンの例は`config/mujoco/scene_triangle.yaml`、`scene_box.yaml`、`scene_pentagon.yaml`です。`grasp_sim.launch robot_count:=3/4/5`で対応するシーンを選びます。タスク設定は3台で`task.yaml`、4・5台で`task_multi.yaml`を選びます。目標押し付け力は、確定した物体質量と台数、タスクの設計摩擦係数・余裕率・想定鉛直加速度から計算します。`scene_config`・`task_config`等で別ファイルも指定できます。シーンYAML内の`robot_count`はlaunchの台数と一致する必要があり、不一致ならMuJoCo起動前にエラーにします。これらの明示的な設定選択以外に、旧デモの形状別チューニングをC++側で自動適用することはありません。

生成だけ行う場合:

```bash
rosrun bee mujoco_scene_generator.py \
  "$(rospack find bee)/config/mujoco/scene_triangle.yaml" \
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

`execute`の既定値は`false`です。この場合、計算した指令は`~preview/beeN/reference`（BeeCommand）、`~preview/beeN/nav`と`~preview/beeN/attitude`（旧形式の表示）に出します。中央は`/beeN/command`へ送信しないため、各Beeの共通参照処理も指令を引き継ぎません。`execute:=true`でも離陸や把持開始は自動では行いません。各Beeを既存の操作方法で離陸させ、全台のHOVERと入力がそろった後、`rosrun bee grasp_keyboard.py`で`g`を押すか、次のserviceで開始します。

```bash
rosservice call /bee_cooperative_grasp/start "{}"
```

操作は`g`が開始、`t`がHoldから運搬、`x`/Ctrl-Cが中止です。旧キーボードデモのXYZ/yaw連続操作はこの操作入口には含めていません。中央ノードはタスク設定に従って自動シーケンスを進めます。操作クライアントの終了自体は自律制御の停止条件ではありません。

実物体を使う場合は `object_model:=/absolute/path/to/object.yaml` を指定します。書式例は `config/grasp/object_triangle_example.yaml` と `config/grasp/object_sphere_example.yaml` です。球の半径は `surface_radii` の3軸、直径は `object_height`、質量は `object_mass` に指定します。質量は kg、重心まわりの慣性は物体座標系で kg·m² の行優先9要素、位置は世界座標の m です。例の数値は設計値です。物体の剛体姿勢は `/mocap/object/pose` に world 基準の `geometry_msgs/PoseStamped` で入力します。別名は `object_pose_topic` で指定できます。物体モデルを指定すると Motive 剛体 ID 10 の受信ノードも起動します。別の ID は `object_mocap_id` で指定できます。物体YAMLの `marker_to_model_xyz` はマーカー座標でのモデル重心位置 [m]、`marker_to_model_rpy` は同座標でのモデル姿勢 [rad] です。両方省略すると一致を仮定します。各機には `robot_description` が必要です。

実機用の単機 `bringup.launch` では機体ごとの `robot_id` を指定します。外力 observer と自重補償は全機で常時有効です。全機の位置姿勢と物体重心の姿勢は同じ世界座標系に揃えます。

中央は `roslaunch bee cooperative_grasp.launch object_model:=/absolute/path/to/object.yaml execute:=true` で指令配信を有効にします。起動直後は Idle です。開始サービスでは全機の HOVER、入力の鮮度、各機の追従器の準備状態と指令の購読を確認します。

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

旧`mujoco_grasp_scene.launch`、`mujoco_multi_module.launch`、旧Python運搬/MPC一式は[`archive/2026-11-16-legacy-grasp`](archive/2026-11-16-legacy-grasp/README.md)へ保存しました。現行の入口は`grasp_sim.launch` / `grasp_demo.launch`です。実機用旧入口と依存する基本デモも、同日後続の整理で廃止しました。

`~phase`、`~reason`を公開します。`~metrics`の順序は、法線力、Z reference位置・速度・加速度、旧Z積分加速度、鉛直目標力、旧共通Z加速度、支持ramp比、平均接触滑りです。分散追従では旧加速度項はゼロで、実際の参照と出力は`/beeN/command`と`/beeN/controller/tracking_status`を参照してください。CSVが必要な場合はこのtopic等を外部記録し、制御周期からファイルI/Oを分離します。

## 分散追従の制御分担

`controller/distributed_tracking: true`では中央が`/beeN/command`へ送信し、
各BeeのBeeControllerが運動・力を追従します。状態は`controller/tracking_status`で返します。
中央はstatusの準備完了・鮮度と自分が要求した校正の完了を確認します。
校正は各Beeで1回取得し、中央にも同じ基準を共有します。

上位のQP/物体PIDと下流の力PIを分け、位置の復元や接触判定などを下流へ追加しません。
把持固有の押込み保護は中央で判断し、汎用の加速度範囲へ変換して送ります。
標準設定では接近・押付け・支持・解放を通して各Beeが同じ力PIで3次元力目標を追従します。
`wrench_control: false`の旧支持/observerブレンドも中央で加速度へ変換します。

詳細と旧インターフェースからの移行は[共通指令追従](CONTROLLER_REFERENCE.md)を参照してください。

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

この節は配置誤差の定義だけを一般化した時点の記録です。当時のシーン生成・接近・ローカルの力制御は水平な側面法線と鉛直摩擦支持を前提としていました。その後、冒頭の「曲面接触の試験用拡張」で3D法線と接平面支持へ更新しています。ただし任意姿勢の接近、接触点探索、任意配置での全工程は現在も検証対象外です。

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
- 初期移行時は元のPythonデモと実機入口を保持しましたが、2026-11-16の整理で廃止しました。以下の初期試験結果は当時の記録です。
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
python3 "$(rospack find bee)/test/check_cooperative_grasp_ros.py" \
  --node /absolute/path/to/devel/lib/bee/bee_cooperative_grasp_node
```

ROS通信テストは自分専用のROS masterを起動し、終了時にテストプロセスを停止します。既存のmasterや実機には接続しません。

力補償LPFの切替と既存bagの振動解析は [FORCE_FEEDBACK_FILTER.md](FORCE_FEEDBACK_FILTER.md) を参照。既定では追加LPFは無効。
