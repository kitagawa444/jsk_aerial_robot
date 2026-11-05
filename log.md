# Bee / MuJoCo 物体操作 作業ログ

最終更新: 2026-08-07
対象リポジトリ: `jsk_aerial_robot`
作業ブランチ: `develop/bee`

## この文書について

このログは、Git履歴、現在のworking tree、保存されている実験CSV、および作業中の対話をもとに再構成したものです。過去の全コンソール出力が保存されていたわけではないため、古い試行の数値を後から推測して補ってはいません。成功した変更だけでなく、採用しなかった方針、失敗の原因、未解決点も残します。

## 現在の結論

- 3台のBeeを三角柱の3側面へ接近させ、各機の実際の4本足のゴム球で押し付けるところまでは安定して動く。
- 接触力フィードバックはMuJoCoのtouch/force-site値やばね圧縮量ではなく、各Beeの6軸momentum observerを使う。
- observer wrenchから、各Beeの接触代表点まわりの局所モーメントと、物体CoGまわりの合力・合モーメントを計算、publish、CSV記録できる。
- PRELOAD中は、3台の法線力配分と姿勢trimにより物体モーメントを小さくできる。1 N/台では、フィルタ後の合モーメントの平均ノルム約 `0.057 mN m` を確認した。
- 3 Nまたは4 Nで不安定化していた主要因は、持ち上げ中の鉛直摩擦が作る `Mx/My` を、ほとんど高さレバーアームのない法線力配分で消そうとしたことだった。LIFT中は法線力を等配分へ戻し、鉛直速度差で `Mx/My` を制御するよう分離した。
- 4 N/台、上昇指令 `0.10 m/s` では物体を最大約5.4 cm持ち上げた試行がある。新しい安全・HOLD制御では4 N/台と5 N/台の双方で約2 cmの持ち上げを反復確認した。
- `impratio=50`、連続高度回復力、差動高さ制御の組合せで、235.1 simulation秒の持続HOLDを確認した。微小creepとHOLD中のyaw driftは残るが、HOLD後のworld-XYZ並進操作まで動作確認済み。

## 関連コミット

この一連の作業開始時のHEADは `149eed5a`。`origin/develop/bee` の `5d950169` より後に、次の作業コミットがある。

| commit | 内容 |
|---|---|
| `eed90a59` | Beeのばね付き4本足、ゴム接触、把持物体、MuJoCo/Gazebo設定を追加 |
| `e953e308` | ACC modeへ入る際にXY PIDをresetし、過去の位置/速度I項が加速度指令を上書きしないよう修正 |
| `b51acb1d` | MuJoCoの把持状態・force site読み出しと外部wrench入力経路をsimulation側へ追加 |
| `1f5cf00f` | 三角柱、台座、RViz可視化、初期grasp demoを追加 |
| `9584e045` | 床・物体接触を安定化し、単機push demoを追加 |
| `149eed5a` | observer制御のpush/tilt/dual/triple操作デモを追加 |

takeoff問題の基準にしたのは `f99b305c` (`[bee] preserve base I terms outside wrench compensation`)。Beeと共通制御の整合を取り、Beetle側には同趣旨の `c3fd058f` がある。Ninjaについてはパッケージ追加履歴 `d0380888` は確認できるが、この作業列にはNinja専用の追加コミットは残っていない。

## 作業の軌跡

### 1. takeoff、ブランチ競合、ACC mode

- `f99b305c` と同じ考え方で、wrench compensationの外側にある基礎I項を壊さないようBee/Beetle系の制御を整理した。
- `origin/develop/bee` との競合を解消し、Gazeboでtakeoffまで確認した後に作業ブランチへ取り込んだ。
- ACC modeへ切り替える際、takeoffや位置制御中に蓄積したXYのI項が外部加速度指令を打ち消す問題を確認した。そのため `pose_linear_controller.cpp` でX/Y PIDをresetする変更を `e953e308` として残した。
- これはACC modeを直接feed-forwardインターフェースとして使うための変更であり、常時I項を消す変更ではない。

### 2. MuJoCo ROS controlと外部wrench

- `aerial_robot_simulation` のMuJoCo hardware interfaceへ、把持用force site値の読み出しと `mujoco/external_wrench` subscribeを追加した。
- external wrenchはシミュレータ中のBeeへ外力・外モーメントを直接加える試験用インターフェース。既存の地上ロボット向け機能を削らず、追加経路として実装した。
- その後、実機条件へ寄せるため把持デモではexternal wrenchを使わず、`uav/nav` のACC/VEL指令で機体を動かす方針へ変更した。インターフェース自体はsimulation側へ残してある。

### 3. 4本足のばね・ゴム構造

- Beeの4本足へ、slide joint、ばね、damping、先端ゴム球を追加した。
- `stiffness` は変位に比例して戻すばね力、`damping` は速度に比例して振動を減らす力として調整した。
- MuJoCoだけでなくGazebo用xacroにも対応を追加した。
- `ignore joint` で制御対象から除外する案を検討したが、機構として動く足はpassive jointとして物理計算へ残す考え方を採った。
- force siteは4本足の球付近へ配置し、各球の接触力を監視できるようにした。ただしforce site値は制御判定には使わない。
- 物体と接触していない不明瞭な仮想contact pointは廃止し、姿勢は `final base link rot` のroll約 `1.57 rad` により、実際の足を面へ向ける構成にした。

### 4. 把持対象とシーン

- 対象は一辺約 `0.8 m` の三角柱。3台を各側面法線方向に配置する。
- 床上では飛行後の側面アプローチが難しいため、高さ `0.65 m` の台座上へ置く構成にした。物体中心の初期Zは約 `0.8 m`。
- 物体質量は最終的に `0.2 kg`。重量は約 `1.96 N`。
- 物体・ゴム球などのsliding frictionは議論後に概ね `0.5` へ下げた。物体・台座の不要な自転を抑えるためtorsional frictionとfree-joint dampingも調整した。
- 台座上で物体が勝手に回転する、spawn直後に床上のBeeが傾き振動してずれる、といった接触問題をsolver/contactパラメータ、stiffness、damping、摩擦設定の調整で軽減した。

### 5. 描画と実行速度

- multi-module MuJoCoは複数機分のROSノード、sensor publish、制御、接触計算を同時に行うためsim timeが遅くなる。
- WSLではOpenGLがソフトウェア描画または遅い仮想GPU経路になりやすく、GUI描画が特に遅かった。
- `headless:=true gui:=false`、影・反射・vsync無効、低いrender FPSを用意した。
- headlessでもground truth/TFと専用visualizerを使い、RVizでBee、三角柱、台座を確認できる構成を追加した。

### 6. 初期grasp demoで採用しなかった方法

- 3台のroot bodyを直接座標操作して押し込む方法を試したが、物理系を飛び越えてめり込みと開始時ジャンプを作るため廃止した。
- external wrenchで直接押し付け・持ち上げる方法は、摩擦だけで持ち上がるかを切り分ける試験には使えるが、実機と条件が合わないため最終制御から外した。
- touch値、force-site値、slide joint圧縮量による接触判定も、MuJoCo固有情報で実機に持ち込めないため制御から外した。現在はCSV監視専用。
- デモ終了時に機体が無限遠へ飛ぶ、開始時に跳ぶ問題に対して、終了時のneutral stopと状態遷移を追加した。

### 7. push系デモ

- `mujoco_push_demo.py`: Bee1だけで三角柱を押し、台座から落とすデモ。
- `mujoco_tilt_lift_demo.py`: 法線方向に加えて斜め上方向の指令を与え、片側を浮かせる試験。法線・上向き成分をROS parameterで調整可能にした。
- `mujoco_dual_push_demo.py`: Bee1/Bee2が同時に押し、合成ベクトル方向へ物体が動くか確認するデモ。押す時間を延長した。
- `mujoco_triple_press_keyboard_demo.py`: 3台の共通法線力と全体Z速度をキーボードで操作するデモ。`keyboard.py` に近い操作感を目標とした。

### 8. force-siteからobserver feedbackへ

- 法線力制御はmomentum observerの推定外力を面法線へ射影する方式へ変更した。
- 接触前に取得するobserver baselineは、センサバイアス、モデル誤差、定常的な重力補償誤差を「接触力」と誤認しないために差し引く。
- 一方で、Z方向のI項や重力補償不足までbaselineで隠すのは正しくない。baselineは接触前の定常offset除去であり、機体制御の重力補償の代替ではない。
- observerの追従が遅い、特にZ方向の動的摩擦を小さく推定する傾向を確認した。force site合計との比較はログに残すが、制御入力には戻していない。

### 9. 物体相対位置制御

- 接触後に3台全体が横へずれると物体が回転するため、物体TFと各Beeの4球中心との相対位置を使う制御を追加した。
- 面法線方向は法線力制御、面内接線方向は相対位置PD、Z方向は物体相対高さPDという分担にした。
- `ground_truth` の座標値がworldでframe情報が欠落していても、全入力を同じworld座標として扱う限り、今回の相対差計算自体には致命的ではなかった。

### 10. wrenchとモーメント

- 各Beeのobserverは機体CoGまわりの6軸wrenchを出す。
- 力をBee CoGから4球中心へ移し、接触代表点まわりの局所モーメントを計算する。
- さらに接触代表点から物体CoGへのレバーアーム `r` を使い、物体が受けるモーメント `r x F` と局所モーメントを合成する。
- 物体が受けるのは接触点の力だけではなく、その力が物体CoGから離れて作用することで生じるモーメントも含む。
- 持ち上げ時は下側の球へ鉛直摩擦が偏りやすく、Bee側にも局所モーメント、物体側にも力の作用線によるモーメントが出る。したがって「常にモーメント0が唯一の正解」ではないが、意図しない回転を防ぐ目標として合モーメント0を採用した。

### 11. モーメント制御の試行

- 最初に制御を入れず、局所モーメントと物体CoGまわりの合モーメントをpublish/CSV記録した。
- PRELOADでは、`Mx/My` を3台の法線力のzero-sum配分、`Mz` を共通yaw trimで制御した。
- 各Beeの局所面内モーメントはroll trim、局所鉛直軸モーメントは個別yaw trimで抑えた。
- PI積分を強くした試行は長周期振動を作った。符号を反転した試行はモーメントを約3 mN mから40 mN mへ発散させたため不採用。
- deadband、rate limit、integral unwind、anti-windupを追加して安定化した。
- 1 N/台の安定試験では、3台目標 `0.9923 / 1.0059 / 1.0017 N`、合計ちょうど `3.0 N`。フィルタ後合モーメント平均 `(-0.0036, +0.0059, -0.0566) mN m`、平均ノルム `0.0570 mN m`。raw平均ノルム `0.0536 mN m`、瞬時raw RMS約 `2.92 mN m` で、瞬時値にはobserver noiseが残った。

### 12. 3 N/4 Nで不安定化した原因

- 3 N/台、上昇 `0.02 m/s` の旧制御ではBee2へ鉛直荷重が集中し、物体 `My` が約 `0.002 Nm` から `0.93 Nm` へ増加した。
- PRELOAD用の法線力配分器が、この鉛直摩擦由来の `Mx/My` を法線力で消そうとした。しかし法線力のZレバーアームは小さく、必要配分が飽和して約 `2.75 / 3.61 / 2.64 N` となり、さらに偏りを増やす正帰還になった。
- 対策としてPRELOADとLIFTを分離した。LIFTでは法線力offsetをzero-sumで等配分へ戻し、姿勢trimをfreezeし、鉛直摩擦由来の `Mx/My` は各BeeのZ速度offsetで制御する。
- 押し付け力はstep入力せず、既定 `0.30 N/s` でrampする。LIFT開始時はramp完了と等配分復帰を待つ。

## 2026-08-07 LIFT試験結果

すべて物体質量 `0.2 kg`、摩擦係数 `0.5`、headless MuJoCo。CSVは `/tmp` に出したため恒久成果物ではない。

| 条件 | 結果 | 主な観測 |
|---|---|---|
| 3 N/台, `+0.02 m/s`, timeout 2 s | 持ち上がらず安全停止 | object rise約0.03 mm、水平moment最大約0.0026 Nm |
| 4 N/台, `+0.02 m/s`, timeout 2 s | 持ち上がらず安全停止 | force-site上向き合計平均約0.985 N、重量未満 |
| 4 N/台, `+0.04 m/s`, timeout 2 s | 持ち上がらず安全停止 | force-site平均約1.326 N、最大約1.729 N |
| 4 N/台, `+0.04 m/s`, timeout 5 s | 持ち上がらず安全停止 | force-site最大約1.821 N、接触球は14--22 mm上昇 |
| 4 N/台, `+0.10 m/s`, 初期LIFT分離版 | 一時持ち上げ | object `0.7998 -> 0.8537 m`、約5.4 cm。force-site上向き約2.77 N、observer Fz約1.96 N。その後Beeだけが上へ滑り台座へ戻った |
| 4 N/台, `+0.10 m/s`, 強すぎる相対Z制御 | 不採用 | 摩擦を立ち上げる面上移動まで止め、持ち上がらなかった |
| 4 N/台, `+0.10 m/s`, HOLD高さ2 cm | 一時持ち上げ | 約2.3 cm上昇後HOLDへ移行したが、約1秒で落下。安全停止 |
| 4 N/台, 強いobject-Z PD | 不採用 | Beeが面上を約10 cm滑り、物体が台座から落下。ゲインを戻した |
| 5 N/台, `+0.10 m/s`, 安全版 | 一時持ち上げ | 約2.2 cm上昇。約3秒後、1機の面上すべり6 cmを検出してデモ終了 |

重要な観測:

- 4 N/台の理論摩擦上限は、単純な `mu * sum(N)` なら `0.5 * 12 = 6 N` で重量1.96 Nを上回る。しかし実際の鉛直力は、接触状態、球のすべり、機体応答、面内モーメントに強く依存する。
- 持ち上げ中のforce-site上向き合計は約2.7--2.8 N、observerによる物体Fzは約1.94--1.97 Nだった。observerは動的な鉛直接触を小さく見る一方、force-siteの単純和も物体へ作用する正味力と一致しない。
- 押し付け力を4 Nから5 Nへ増やしても持続保持は解決しなかった。単純な法線力不足ではない。
- 高度を戻すためBeeを上へ動かすと球が面上を滑る。絶対接触高さへ戻そうとすると、今度は高度回復指令を打ち消す。この共通モードと差動モードの分離が次の課題。

## 現在の `mujoco_triple_press_keyboard_demo.py`

### 制御状態

- `PRELOAD`: 法線力ramp、observer法線力feedback、接線位置制御、局所/物体moment制御。
- `LIFT`: 法線力を等配分へ戻し、PRELOAD用姿勢trimをfreeze。物体 `Mx/My` を3台のzero-sum Z速度offsetへ割り当てる。
- `LIFT_HOLD`: 持ち上げが1秒継続し、既定2 cmに達したら物体ZのPD holdへ移る。HOLD移行時の球中心相対Zを新しい基準として記録する。
- `FAULT`: 水平moment過大、5秒以内に3 mm上がらない、面上すべり過大、HOLD後の落下を検出したらZ指令を0にし、デモを終了してneutral stopを送る。

### 主な安全値・parameter既定値

- `~normal_force_ramp_rate = 0.30 N/s`
- `~lift_progress_timeout = 5.0 s`
- `~lift_minimum_object_rise = 0.003 m`
- `~lift_rise_confirmation_duration = 1.0 s`
- `~lift_hold_height = 0.02 m`
- `~lift_abort_moment_threshold = 0.05 Nm`
- `~lift_relative_contact_z_error_limit = 0.06 m`
- `~lift_hold_object_z_kp = 2.0`
- `~lift_hold_object_z_kd = 1.0`
- `~lift_hold_z_velocity_limit = 0.08 m/s`

### publish/CSV

- 各Beeのobserver local contact wrench
- 物体CoGまわりのobserver resultant wrench
- force-arm momentとlocal moment sum
- 3台へ配分した法線力目標
- LIFT用Z速度offset
- requested/active common normal force
- 物体pose、接触代表点、局所/物体moment、姿勢trim
- force-site Z合計と上向き合計（監視専用）
- `PRELOAD / LIFT / LIFT_HOLD`、fault、HOLD高度PD指令

## 起動方法

端末1:

```bash
source /home/kitagawa/ros/jsk_aerial_robot_ws/devel/setup.bash
roslaunch bee mujoco_multi_module.launch \
  headless:=true gui:=false launch_rviz:=false \
  enable_grasp_object:=true
```

端末2（安全にPRELOADから手動操作）:

```bash
source /home/kitagawa/ros/jsk_aerial_robot_ws/devel/setup.bash
rosrun bee mujoco_triple_press_keyboard_demo.py \
  _target_normal_force:=1.0 \
  _initial_z_velocity:=0.0 \
  _wrench_log_path:=/tmp/bee_triple_press.csv
```

キー:

- `w` / `s`: 3台共通の法線力を増減
- `[` / `]`: 共通world-Z速度を増減
- `SPACE`: Z速度を0
- `p`: 現在値表示
- `r`: 初期値へreset
- `x`: 終了

4 N/台・0.10 m/sは一時持ち上げを確認した試験条件であり、持続保持済みの安全条件ではない。試す場合はheadlessでもCSVとmoment faultを監視する。

## 未解決点と次に試す方針

1. Z方向を「3台共通モード」と「3台差動モード」に明示分解する。
   - 共通モード: 物体高度/速度または総上向き推定力を制御。
   - 差動モード: `Mx/My` と3台の接触高さ差だけを制御。
   - 現在のように各Beeが絶対相対Zを個別追従すると、共通の摩擦生成動作を消してしまう。
2. 物体高度PDだけでなく、observerの3台Z外力合計へ低帯域feedbackを入れる。ただし重力補償誤差とobserver delayを先に同定する。
3. 各BeeのZ速度指令だけでなく、Z加速度またはthrust余裕を使って静止摩擦を作れるか比較する。実機へ持ち込める信号だけを使う。
4. 面上すべりを許容する範囲と、球が三角柱上端へ到達するまでの幾何学的余裕を明示的に管理する。
5. 物体を一度持ち上げた後、4台目が底面へ入る案は最終保持の余裕を増やせる。ただし3台だけで底面アクセス高さまで安定保持する段階を先に完成させる。

## 2026-08-07 追試: 共通Z力・差動Z分離

### 実装

- 足ばねをMuJoCoの `1500 N/m, 20 Ns/m`、Gazeboの `600 N/m, 20 Ns/m` から、両方とも `1000 N/m, 15 Ns/m` へ変更した。4 N/台を4球へ均等配分した場合、1球約1 N・圧縮約1 mmを狙う。
- 3台のobserverから物体へ作用するworld-Z合力を合成し、低域filter後の値を共通Z制御へ使用する。
- 持ち上げ開始時の合力目標上限を `mg + 0.4 N`、HOLD時の基準を `mg = 1.962 N` とした。
- 物体高度誤差・Z速度から合力目標を連続生成し、2 cmへ近づくほど余剰力を減らす。2 cm到達後も同じ構造で高度を保持する。
- 力制御器の出力は物体面に対する共通すべり速度として扱い、Beeへ送るworld-Z指令は `object vz + common relative vz + contact differential vz + moment differential vz` とした。
- 3台間の接触高さ差PDと `Mx/My` 配分は独立したzero-sum速度offsetとし、それぞれ3台合計が0になるよう毎周期補正する。
- HOLD移行時は持ち上げ区間の積分と共通相対速度をresetする。
- HOLD目標より15 mm落下、平均接触面すべり6 cm、3台間高さ差25 mm、水平moment 0.05 Nmを安全停止条件とした。

### 試験結果

条件は法線力4 N/台、物体0.2 kg、摩擦設定は従来どおり、持ち上げ相対速度上限0.06 m/s。

- object Zは約 `0.800 -> 0.847 m`、最大約4.7 cm持ち上がった。
- 3台接触高さ差は平均約1.1 mm、最大約6.0 mm。
- `Mx/My`用offsetと接触高さ用offsetは、どちらも3台合計誤差 `1e-8 m/s` 以下。
- filtered水平momentは平均約0.0027 Nm、最大約0.0074 Nm。
- 3台の偏りや水平moment発散は見られず、共通/差動分離は意図どおり機能した。
- ただしobject ZはHOLD移行後に上下振動し、最終的に台座へ戻った。持続hoverは未達。

### 追試で不採用になった調整

- 総Z力Pゲイン `0.05`: 落下回復時の相対速度が約0.015 m/sに留まり、遅すぎた。
- 総Z力Pゲイン `0.30`: observer/contact応答遅れに対して強すぎ、上下振動を増やした。
- 持ち上げ上限0.10 m/s: 2 cm到達時の上向き運動量が大きく、合力目標を下げても制動が間に合わなかった。
- 1秒継続後にHOLDへ遷移: 物体が既に6--12 cm上がってから切り替わるため廃止し、2 cm到達時の即時遷移へ変更した。
- observer合力制御出力をそのままworld-Z速度とする方法: 物体下降時にBeeが逆向きへ動き、共通すべりを増やすため廃止した。現在はobject Z速度をfeed-forwardする。

### 次の課題

現在の支配要因は差動バランスではなく、observer Z合力と摩擦力の位相遅れを含む共通高度ループである。次は次のいずれかが必要。

1. observer Z合力filter、機体Z速度応答、物体加速度まで含めた遅れを同定し、共通ループ帯域をそれ以下へ制限する。
2. object Zの2次系軌道を先に生成し、必要合力 `m(g + a_ref)` をfeed-forward、observerは低帯域補正だけに使う。
3. VEL modeではなくACC modeで共通Z加速度を与え、物体速度feed-forwardと外力補償を分離する。

force-site値は引き続き評価ログ専用で、上記制御には使っていない。

## 2026-08-07 追試: Z ACCと物体加速度feed-forward

### 実装

- `FlightNav` に `target_acc_z` を追加し、共通の `BaseNavigator` とBee固有の `BeeNavigator`（通常nav/assembly nav）で `pos_z_nav_mode = ACC_MODE` を受けられるようにした。
- navigation内部にXYとは独立したZ制御modeを追加した。Z ACC中はZ位置・速度誤差をPIDへ渡さず、Z PIDを毎周期resetして `target_acc_z` だけを出力する。
- Bee MuJoCoで有効な重力feed-forwardは、Z ACCへ入った時点で直ちに `g` を負担する。したがって `target_acc_z = 0` は機体hover、正値は重力補償後の正味上向き加速度になる。
- 物体Z参照を速度・加速度・jerk制限付きで生成し、位置/速度追従項も加速度上限内に制限する。
- 物体に必要な鉛直力を `Fz_ff = m_object * (g + a_object_ref)` とし、低域observer補正を加えた後、3台のBee質量で割って共通Z加速度へ変換する。
- observer Z合力は時定数0.5 sでfilterし、既定のP/I補正は `0.08 / 0.01`、補正力上限は0.30 Nとした。observerを高速な主feedbackには使用しない。
- 3台の接触高さ差PDと物体 `Mx/My` 補正を、速度offsetからZ加速度offsetへ変更した。各系統はrate limit後にも平均値を引き、3台合計を常に0にする。
- CSVへ物体Z参照の位置・速度・加速度、物体加速度指令、observer補正力、共通/差動Z加速度を追加した。

### 検証

- `catkin build aerial_robot_msgs aerial_robot_control gimbalrotor bee --no-status` を含む依存21 packageが成功した。
- `python3 -m py_compile` と `git diff --check` が成功した。
- 手動でBee1へ `pos_z_nav_mode=ACC_MODE, target_acc_z=0.10` を送った結果、`/bee1/debug/pose/pid` のZは `total=0.10`、P/I/D項はすべて0になった。takeoff/hoverで残ったZ積分がACC指令へ混入しないことを確認した。
- 法線力4 N/台、物体0.2 kg、物体速度上限0.06 m/sの3台試験では、object Zが `0.800 -> 0.844 m` まで上昇した。2 cm到達後は軌道速度をほぼ0へ減速し、合力目標は約1.96 Nへ戻った。
- HOLD終盤の接触高さ加速度offset例は `+0.0109, -0.0127, +0.0018 m/s^2`、`Mx/My`加速度offset例は `+0.0019, -0.0027, +0.0008 m/s^2` で、丸め誤差内でそれぞれ合計0だった。
- observerのfiltered Z合力は最終的に約1.96 Nへ収束したが、物体は台座付近へ戻り、Beeの平均接触面だけが上へ約6 cm滑って安全停止した。0.01 m/s・1 cm持ち上げでも同じ傾向であり、持続hoverは未達。
- 後半の物体Z参照は約0.8116 m、参照速度0.001 m/sまで減速していたため、今回の失敗は参照軌道が上へ走り続けたためではない。observer合力がmgを示していても、実際には物体が台座支持へ戻り、Beeが面上を滑る状態を区別できないことが残る弱点である。

### 起動時の注意

multi-module起動時、MuJoCoのclock/sensor開始と3台のbase node初期化が重なると、Bee2/3がEigenのmatrix assertionで落ちる既存の起動競合が再現した。backend開始後に対象Beeの`bringup.launch`だけを再起動するとodom/observerが復帰し、上記試験を実行できた。Z ACC制御に入る前の現象であり、本変更の制御試験とは分けて扱う。

## 2026-08-07 追試: Bee Z重力feed-forward

### 実装

- `GimbalrotorController` に `controller/gravity_compensation` を追加した。既定値は `false` とし、既存のgimbalrotor機体と実機Beeの挙動は変更していない。
- MuJoCo用 `BeeControl_sim.yaml` だけで重力補償を有効化した。PID出力は加速度単位なので、姿勢変換前のworld-Z加速度へモデルの `g` を加える。allocationの並進項に `1/m` が含まれるため、最終的なロータ推力では `mg` に相当する。
- takeoff中は従来どおりZ I項で支持し、HOVERへ遷移した後にI項から重力feed-forwardへ同量ずつ移すbumpless transferとした。既定の移行速度は `2.0 m/s^3` で、約5秒かけて `g` まで移行する。
- I項に残っている正の加速度を超えて移行しないため、HOVER遷移が早い場合も負のI項や推力ジャンプを作らない。
- allocatorとmomentum observerには同じ補償済み加速度指令が渡る。observer内の既存重力項 `N = mg` は変更していない。

### 試験結果

`catkin build gimbalrotor bee --no-status` は全対象パッケージ成功。続いて3台の通常grasp手順を、法線力1 N/台・持ち上げ指令なしで実行した。

- 3台ともtakeoffからHOVERへ入り、姿勢変更、同時接近、PRELOADまで完了した。
- HOVER後のZ I項はBee1/2/3で約 `0.0096 / 0.0071 / 0.0074 m/s^2`。従来のように約 `g` をI項へ保持せず、モデル誤差分だけになった。
- 接触前のobserver CoG-Z baselineは3台とも約 `-0.021 N`。
- PRELOAD中のobserver法線力は約 `0.99 / 1.03 / 0.98 N` で安定し、物体高さは `0.800 m` を維持した。
- 低法線力での確認範囲では、重力feed-forward移行に伴う高度ジャンプ、接触力発散、observer Zバイアス増加は見られなかった。

続けて従来の追試条件、法線力4 N/台・LIFT相対速度上限0.06 m/sでも確認した。

- object Zは `0.800 -> 0.846 m` まで上昇し、2 cm上昇時にLIFT HOLDへ正常遷移した。
- 法線力は3台とも約4.0 Nで安定し、observer Z合力はHOLD中に約 `1.94 N` と物体重量 `mg = 1.962 N` 近傍まで追従した。
- ただしHOLD遷移時の上向き運動量で一度 `0.846 m` までovershootした後に下降し、hold目標より15.2 mm低下して安全停止した。持続hoverは依然未達。
- よって機体自身の重力をZ I項から分離する変更は正常に機能したが、物体HOLD失敗の残りの支配要因は、共通Z力/高度ループの制動と位相遅れである。

単体Beeだけを直接takeoffさせる比較では、補償ON/OFFの両方で既存の横方向不安定が再現したため、重力補償評価には採用しなかった。3台grasp demoの通常staging処理を通した試験では上記のとおり安定した。

## 2026-08-07 追試: 持続HOLDとMuJoCo摩擦constraint

### 実装

- 物体重量feed-forwardを0から`mg`まで1.5秒でrampし、ramp完了後にだけ物体Z軌道を進めるようにした。静止接触へ急に重量を載せない。
- 持ち上げ開始時は`mg + 0.4 N`を上限とするbreakaway marginを使い、HOLDでは重量支持へ戻す。
- HOLD時の固定0.4 N回復力のON/OFFを廃止した。物体の高さ誤差とZ速度から連続的な回復力を計算し、`0..0.4 N`へ制限した上で2 N/sのrate limitを掛ける。既定ゲインは80 N/m、4 Ns/m。
- 3台平均の`contact_z - object_z`を保持開始時に記録し、共通面上すべりをPD補正する。3台間の相対高さ差は合計0の差動加速度で補正する。
- 差動高さ制御の既定値を`Kp=8.0`、`Kd=2.0`、各機上限0.30 m/s^2、高さ差停止閾値40 mmへ更新した。
- scene composerでMuJoCoのtop-level `option`をYAMLから設定可能にした。Bee multi-moduleは`cone=elliptic`、`solver=Newton`、`impratio=50`を既定とした。composerを使う既存sceneは`option`を省略すれば従来動作のまま。
- CSVへ力ramp、HOLD feed-forward scale、連続回復力、共通すべり誤差/速度/補正加速度を追加した。

### 比較試験と不採用調整

条件は物体0.2 kg、法線力4 N/台、持ち上げ速度上限0.01 m/s、目標上昇20 mm、物体摩擦0.5。

- 共通すべりfeedbackを接触開始直後から有効化: ゴム球の初期荷重変位まで打ち消し、物体が台座を離れなかった。物体が20 mm上昇した時点で基準をlatchする方式へ変更した。
- 法線力を6 N/台へ増加: observer鉛直力と滑りはほぼ改善せず、法線力不足は支配要因ではなかった。
- 物体摩擦を0.5から0.8へ増加: 改善は見られなかった。Coulomb係数よりMuJoCoの軟らかい摩擦constraintによるcreepが支配的だった。
- 共通すべり補正上限を0.08から0.5 m/s^2へ拡大: Z支持feed-forwardと競合して物体が台座まで落下したため不採用。相対位置feedbackだけを強くしても支持力との両立にはならない。
- HOLD回復力0.4 Nのhysteresis ON/OFF: 物体高さが約0.817--0.823 mで周期振動し、各周期で足が上へずれるため廃止した。
- `impratio=5`: 接触高さ差強化後、物体を約46 simulation秒保持した後に平均接触面すべり60 mmで停止。
- `impratio=10`: 約93秒保持して同じ60 mmすべりで停止。
- `impratio=20`: 約186秒保持して同じ60 mmすべりで停止。値を上げるごとにcreep rateがほぼ反比例した。

### 最終確認

`impratio=50`、連続回復力、強化した差動高さ制御を組み合わせた試験では、HOLDを235.1 simulation秒継続したところで手動終了した。

- 物体ZはHOLD全体で`0.817805..0.823000 m`。固定回復力で見られた継続的な上下振動は消えた。
- 終了時のobserver鉛直合力は1.9619 Nで、物体重量1.962 Nと一致した。
- 終了時の連続回復力は0.1307 N、鉛直力目標は2.0947 Nだった。
- 平均接触面の累積すべりは30.2 mm。安全閾値60 mmには達せず、3台間高さ差、水平moment、物体落下のfaultも発生しなかった。
- よって数分単位の持続HOLDは達成した。一方でMuJoCo上の微小creepはゼロではなく、長時間を無制限に保持する保証には至っていない。実機ではゴムの静止摩擦、変形、温度依存を別途同定する必要がある。

## 2026-08-07 追試: HOLD中のkeyboard並進操作

### 実装

- HOLD後に`i/k`でworld-X、`j/l`でworld-Yの物体目標速度を0.01 m/s刻みで操作可能にした。XY合成速度上限は0.05 m/s。
- 物体の推定world-XY速度に対するP制御を行い、上限0.10 m/s^2、rate limit 0.30 m/s^3の共通加速度を3台へ加える。法線力、物体相対接触位置、zero-sum差動Z制御とは分離している。
- `m`でXY速度を0にし、SpaceでXYZ速度をすべて0にする。
- HOLD後の`[`/`]`はworld-Z目標速度を0.005 m/s刻みで変更する。Beeを直接動かさず、物体HOLD目標高度を速度制限付きで移動する。既定の上下移動範囲は初期HOLD高さから±0.10 m。
- 目標速度と共通XY加速度をROS topicおよびCSVへ追加した。

### 試験結果

法線力4 N/台、`impratio=50`で確認した。

- `i`を2回入力してworld-X目標を+0.02 m/sとした。47.797 simulation秒で物体Xは`-0.0152 -> 0.8708 m`（平均約0.0185 m/s）へ移動した。
- `m`で停止後、物体は約15 mmの制動距離でほぼ停止した。
- `[`を1回入力してworld-Z目標を+0.005 m/sとした。19.974秒で物体Zは`0.8197 -> 0.9192 m`へ移動し、上限到達後は`0.9197 m`で保持した。
- 試験終了まで物体落下、3台高さ差、momentのfaultは発生しなかった。終了時共通すべりは25.4 mm。
- Y方向へ約40 mmの低速driftが残った。HOLD中の物体yaw momentが未制御であるため、次の課題は物体yaw角・yaw角速度を含む低帯域yaw制御である。

## 2026-08-07 追試: LIFT/HOLD中の物体yaw保持

### 原因と実装

- 従来はLIFTへ入るとPRELOAD用のobject-Mz yaw trimと各Beeの局所yaw trimをfreezeしていた。さらに各面の法線を現在の物体yawから計算するため、残留Mzで物体が回ると3台も回転後の面へ追従し、絶対yawの復元力がなかった。
- LIFT開始時の物体yawを目標としてlatchし、物体yaw誤差、yaw角速度、低域filter後のobserver Mzから共通の循環接線加速度を生成する制御を追加した。
- 3台へ同じスカラー値を各面の接線方向へ与える。正三角形では3本の接線力の並進合力は0になり、物体中心まわりのMzだけが加算される。
- yaw角度・角速度を主feedback、observer Mzをdeadband付き低帯域補正として使用する。加速度上限0.12 m/s2、rate limit 0.30 m/s3、HOLD中のyaw誤差0.35 rad超過を安全停止条件とした。
- yaw目標、誤差、角速度、observer Mz補正、循環接線加速度をROS topicとCSVへ追加した。

### 検証

法線力4 N/台、物体0.2 kg、`impratio=50`、持ち上げ速度上限0.01 m/sで、HOLDを105.2 simulation秒継続して手動終了した。

- 物体yaw目標は0.000065 rad。yaw誤差は最大0.01944 rad（1.11 deg）で頭打ちとなり、終了時0.01229 rad（0.70 deg）まで減少した。
- 終盤20%のyaw rate絶対値平均は0.000207 rad/sで、継続回転は止まった。
- 従来のyaw無制御HOLDではfiltered Mz絶対値平均が0.0329 Nm、終盤0.0222 Nmだった。新制御では全HOLD平均0.000569 Nm、終盤0.000219 Nmとなった。
- 物体XY変位は105.2秒で約`+2.25 / -4.49 mm`、物体Zは`0.81767..0.82229 m`。yaw補正による大きな並進移動、HOLD fault、物体落下はなかった。
- 定常接線加速度が約-0.0065 m/s2必要なため、yaw角には約0.7 degの定常偏差が残る。継続回転の防止は達成しており、偏差をゼロへ寄せる場合は小さいyaw積分項を追加する余地がある。

## 2026-08-07 追試: keyboard XYZ+yaw統合操作とZ上昇停止の修正

### Z上方向が効かなくなる原因と修正

- HOLD中の物体Z目標は、初期HOLD高さから`~translation_z_range`以内へ制限していた。従来値は±0.10 mで、上限へ達すると`target_z_velocity`を通知なしで0へ戻していた。このため、上昇操作が途中から効かなくなったように見えていた。
- 既定範囲を±0.30 mへ広げた。上限または下限へ達した場合は安全のため速度指令を0へ戻す動作を維持しつつ、ROS warning、command表示、CSVの`translation_z_limit_active`で明示するようにした。
- CSVへ物体Z目標`translation_target_z`も追加し、指令が止まったのか、実機体/物体が追従していないのかを区別できるようにした。

### yaw操作

- HOLD後に`q/e`で物体yaw目標速度を±0.02 rad/s刻み、最大±0.10 rad/sで操作可能にした。yaw目標角を速度指令で積分し、既存の物体yaw角・角速度・observer Mz feedbackが追従する。
- SpaceでXYZとyawの速度指令を同時に0へ戻す。速度を止めた時点のyaw目標角は保持する。
- yaw速度指令とyaw状態をROS topicおよびCSVへ追加した。共通並進加速度topicのZには物体Z軌道加速度も出力する。

### 統合試験

法線力4 N/台、物体0.2 kg、`impratio=50`で、XYZ並進、yaw回転、停止後HOLDを連続して確認した。

- `X=+0.02 m/s、Z=+0.005 m/s、yaw=+0.02 rad/s`を42.30 simulation秒入力し、物体はX方向へ+0.789 m、Z方向へ+0.209 m移動した。Zは旧上限の+0.10 mを越えて追従した。
- 続いて停止し、`Y=+0.02 m/s、Z=-0.005 m/s、yaw=-0.02 rad/s`を39.20秒入力した。物体はY方向へ+0.727 m、Z方向へ-0.196 m移動した。
- yaw角速度の後半平均は正転時+0.01983 rad/s、逆転時-0.01996 rad/sだった。回転中は駆動トルクを作るため4.3--5.4 degの定常yaw誤差が生じたが、Space後は約0.001 radまで収束し、yaw rateもほぼ0になった。
- 最終停止後129.4秒間、物体Zは約0.835 mを維持した。全試験で`lift_fault=0`、Z制限到達なし、物体落下なしだった。

## 2026-08-08 自動把持・運搬・設置・release

### 実装

- MuJoCo sceneへ任意位置・高さ・半径の`place_pedestal`を追加した。既存デモへ影響しないよう`spawn_destination_pedestal`の既定値はfalseとした。
- `mujoco_triple_transport_place_demo.py`を追加し、`LIFT -> TRANSPORT -> DESTINATION_SETTLE -> DESCEND -> UNLOAD -> RELEASE -> RETREAT -> UNWIND -> VERIFY -> COMPLETE`を自動実行するようにした。
- TRANSPORTでは物体world-XY位置誤差から速度目標を作り、既存の共通XY加速度制御を使用する。DESCENDでは物体Z目標を5 mm/sで台座上面まで下げる。
- UNLOADでは鉛直支持feed-forwardを3秒かけて0へ下げ、物体重量を台座へ移す。その後、法線力を4 N/台から0 Nへrampし、observer法線力が十分低下したことを確認してから3台を放射方向へ退避させる。
- 退避後はBeeのrollを`pi/2 -> 0`へ戻し、物体poseと速度が目的地で安定したことを確認して完了する。制御feedbackは従来どおりobserverと物体poseを使用し、force-siteとばね圧縮量はCSV診断のみである。
- CSVへ`task_phase`を追加し、各状態の開始・終了を追跡可能にした。目的地台座はRViz markerにも追加した。

### headless MuJoCo検証

同一条件で2回連続して全状態を完了した。2回目のCSVでは次の結果だった。

- 物体を`(0.000, 1.000, 0.800) m`付近から持ち上げ、TRANSPORTを43.7 simulation秒行って目的地`(0.800, 1.000) m`へ移動した。
- 目的地到着時のXY誤差は約8 mm。50 mm持ち上げた状態から約10.8秒で台座へ下降した。
- UNLOAD後、法線力を約6.1秒で`4.0 -> 0.0 N/台`へ下げた。observer法線力は約0.04 N以下となり、接触解除後に3台が退避した。
- 最終物体poseは`(0.7996, 0.9999, 0.7983) m`で、目標に対するXY/Z誤差は約0.4/1.7 mm。物体yawは0.0138 rad、全工程で`lift_fault=0`だった。
- 最初のMuJoCo起動ではBee2 base node初期化時に既存のEigen matrix assertionが一度発生し、必要topic待機で終了した。launchをクリーンに再起動した後の2試行は正常に完了しており、transport/place状態機械内の失敗はなかった。

起動は以下の2端末で行う。

```bash
roslaunch bee mujoco_multi_module.launch headless:=true launch_rviz:=false spawn_destination_pedestal:=true
rosrun bee mujoco_triple_transport_place_demo.py
```

### 500 g物体・長距離運搬・落下時フェイルセーフ

- 三角柱の既定質量を`0.20 -> 0.50 kg`、目的地の既定X位置を`0.80 -> 1.50 m`へ変更した。運搬timeoutも`120 -> 180 simulation秒`へ延長した。
- 運搬中は物体速度・yaw角速度、2台以上のobserver法線力消失、Bee接触面と物体の相対Zずれ、物体poseおよび各Bee odometryの更新停止を監視する。
- 異常検出時は物体poseを制御基準として追跡する処理を即座に切る。各Beeの異常検出時のworld位置とyawを固定目標として4秒間繰り返し送り、rollを2秒かけて直立へ戻した後にデモを終了する。
- フェイルセーフ状態は`/mujoco_triple_transport_place/phase`へ`FAILSAFE_HOLD`、停止完了後は`FAILED`として出力する。
- `FAILSAFE_HOLD`へ入った後は通常の落下判定・observer更新・力/モーメント制御を再実行せず、固定world位置指令だけを送る。これにより二重faultが安全保持時間を打ち切らないようにした。
- 重量増加時の定常Z追従誤差があっても台座へ確実に荷重を渡せるよう、設置時のBee側制御目標を台座面より10 mm下へ置く。物体poseは直接操作せず、接触後は従来どおり鉛直feed-forwardを徐々に抜く。

#### 検証

- 500 g物体を初期位置`(0.000, 1.000, 0.799) m`から目的地X=1.50 mまで運搬し、全状態を完了した。TRANSPORTは約81.2 simulation秒、最終poseは`(1.4977, 1.0020, 0.7984) m`、目的地XY誤差は約3.1 mm、全工程で`lift_fault=0`だった。
- 設置プリロード追加前は物体Z=0.806 mで下降が止まりtimeoutした。この際、`FAILSAFE_HOLD -> FAILED`が動作することを確認した。追加後はZ=0.800 mでUNLOADへ入り、法線力を4 N/台から0へ落として正常に設置できた。
- 異常系では試験時だけ物体速度上限を0.005 m/sへ下げ、TRANSPORT開始直後に意図的に発火させた。物体追従を切った`FAILSAFE_HOLD`を指定8 wall秒維持して`FAILED`へ終了した。終了後の3台の並進速度は最大約0.0008 m/sで、落下物体の追跡や発散はなかった。

## 採用しない方針の一覧

- root body直接操作: めり込み、ジャンプ、非物理的。
- external wrenchを最終把持制御に使用: 実機条件と一致しない。
- touch/force-site/ばね圧縮量を接触判定・feedbackへ使用: MuJoCo固有で実機移植できない。
- 持ち上げ中の `Mx/My` を法線力配分だけで打ち消す: 小さいZレバーアームにより飽和し、正帰還になる。
- 3台の絶対接触高さを強く固定: 摩擦生成に必要な共通面上移動まで抑える。
- 高ゲインobject-Z PDだけで落下を回収: Beeだけが上へ滑り、接触喪失とクラッシュを招く。
- 押し付け力を単純に増やすだけ: 5 N/台でも持続保持できず、問題の本質は共通/差動Z制御と接触幾何にある。

## 2026-08-07 整理コミット

今回の未コミット差分を、次の機能単位で履歴へ整理した。

- `6c2b7f5`（`aerial_robot_3rdparty`）: scene YAMLのglobal MuJoCo option対応。
- `fb00a2fb`: Z ACC navigation interfaceとZ PIDの切り離し。
- `7952124e`: Bee MuJoCo用の重力加速度feed-forward。
- `868be2ef`: ばね足とMuJoCo contact solverの調整。
- `67e24a41`: observer把持、加速度持上げ、持続HOLD、keyboard XYZ並進。
- `log.md`: 本文書。コード変更とは分離したdocumentation commitとして保存する。

各コミット前に差分checkを行い、最終状態で`catkin build bee --no-status`の依存21 package成功とPython構文checkを確認した。

## 2026-08-08 実物EPSに合わせた縦貫通穴

- 物体を中実の三角柱から、Z方向に同心・同方向の正三角形貫通穴を持つ一体EPSへ変更した。外側一辺0.80 m、内側一辺0.52 m、高さ0.30 mで、Beeが接触する3側面には穴や継ぎ目を設けない。
- 96倍EPSの既定密度を`1000/96 = 10.4167 kg/m^3`とし、外側体積から内側体積を引いて質量を求める。既定体積は0.048012 m3、計算質量は0.50013 kgとなる。
- 同心三角柱の差分について、解析的な重心と慣性をscene composerで設定する。重心は幾何中心のままで、MuJoCoの凸包が穴を塞がないよう物体を3個の凸な台形壁meshへ分割した。
- `object_density`、`object_inner_triangle_side`をlaunch引数化した。`object_mass <= 0`では密度計算を使い、正値を指定した場合だけ実測質量で上書きできる。
- RViz markerにも上下面だけが開いた縦貫通穴を反映した。headless MuJoCoで生成モデルが読み込まれ、0.50013 kgのinertialと3個のcollision geomを持つことを確認した。
- composed sceneの初期カメラを`grasp_prism`へ合わせた。単体Bee modelの古い`statistic center/extent`を引き継いで物体が視野外になる問題を避け、`scene_camera_extent`で表示範囲を調整できる。
- MuJoCoと同じ外側/内側三角形から、発注添付用の`robots/bee/mujoco/grasp_prism_eps_order_spec.svg`を作成した。上面図、Z方向貫通、接触面、法線壁厚80.8 mm、4球接触範囲を記載している。

### 等幅・等高への寸法修正

- 「足が正方形配置なので物体を正方形にする」という解釈は誤りだったため、正方形モデルの試作変更は取り下げた。断面は正三角形のまま、横に平たい形状を避けるため外側一辺とZ高さをともに600 mmへ変更した。
- 500 gと96倍EPS相当密度を維持するため、Z方向の同心三角形貫通穴を一辺418.626 mmとした。殻体積は0.048 m3、面法線方向の壁厚は52.358 mmである。
- MuJoCo GUIへBee 3台とともにspawnし、inertial mass 0.500000 kg、3個のmesh collision geom、台座上で物体姿勢がほぼ単位quaternionのまま静止することを確認した。
- 把持・運搬デモ側に旧寸法の一辺0.80 m、物体中心Z=0.802 m、高さ0.30 mが独立した既定値として残っていたため、新モデルでは接触半径が約58 mm外側、接触高度が約148 mm下側へずれていた。デモ固有の明示的な上書きは維持しつつ、通常時は`bee_mujoco_multi_bringup`の実際の物体寸法・位置・台座設定を一元参照するよう変更した。

## 2026-08-09 物体roll/pitch姿勢制御

### 実装

- 既存の持ち上げ中Mx/My制御を、observerモーメントだけでなく物体姿勢・角速度も使う外側姿勢ループへ拡張した。LIFT開始時の物体上向き軸とroll/pitchを目標としてlatchする。
- quaternionの時刻差からworld座標の物体角速度を計算する。目標上向き軸と現在上向き軸の外積をworld-X/Y傾き誤差とし、`-Kp*tilt_error-Kd*angular_rate-Km*observer_Mxy`から目標Mx/Myを生成する。
- 目標Mx/Myは既存の接触位置allocationで3台のZ加速度へ変換する。差動Z加速度の合計は0を維持するため、共通Zの物体重量feed-forwardを変更しない。
- 既定ゲインは姿勢`Kp=0.015 Nm/rad`、角速度`Kd=0.030 Nms/rad`。物体傾きが0.30 radを超えた場合は把持失敗として停止する。
- `/mujoco_triple_press/lift_tilt_state`、`object_angular_velocity`、`lift_tilt_moment_command`を追加した。CSVにも物体roll/pitch、角速度、傾き誤差、目標モーメントを記録する。
- MuJoCoのtouch、force-site、ばね圧縮量は従来どおり診断専用で、制御feedbackには使用しない。

### A/B試験

600 mm角・高さ600 mm・200 gの中空形状、押し付け4 N/台、持ち上げ20 mm、XY速度0、60 simulation秒で比較した。

- roll/pitch姿勢制御なしでは傾きRMS 0.00820 rad、roll/pitch角速度標準偏差0.00624/0.00593 rad/sだった。
- 既定の姿勢制御ありでは傾きRMS 0.00666 rad、roll/pitch角速度標準偏差0.00510/0.00572 rad/sとなった。傾きRMSは約19%、roll角速度は約18%低下した。
- `Kp=0.03、Kd=0.08`の強ゲイン、およびobserver Mx/Myゲインを0.50から0.20へ下げた組合せは改善しなかったため採用しない。
- 新形状の物体Zは制御なし/ありの双方で約90--100 mmの大きな周期振動が残った。roll/pitchループ固有の発散ではなく、共通Z支持制御側の別問題であり、今回の変更だけでは新形状の安定保持を達成していない。

旧ソリッド形状（一辺800 mm、高さ300 mm、200 g）でも同じ制御で回帰試験した。60 simulation秒保持し、物体Z変動幅5.18 mm、傾きRMS 0.00091 rad、Mx/My RMS 0.00171/0.00178 Nmで、既存の安定保持を維持した。各試験末尾のTRANSPORT timeoutはXY速度を意図的に0へ設定したことによる。

## 2026-08-09 新形状の共通Z持続保持

### 原因

- 600 mm角・高さ600 mm・200 gの中空形状では、物体Z応答に約0.45秒の遅れがある一方、位置PDは`Kp=6.0`、追従加速度上限は`0.15 m/s^2`だった。さらに保持回復力の比例項が最大0.4 Nまで変化し、約0.35 Hz、90--100 mm幅のlimit cycleを作っていた。
- 離床用0.4 N marginを50 mmの保持高度まで出し続けていた。200 g物体には約2 m/s2の追加加速度に相当し、HOLD切替時に大きな上向き速度が残った。
- HOLDで`target_z_velocity=0`へ戻すと内部Z軌道速度も1 mm/sへ落ち、物体は目標へ到達しているのに内部referenceだけが数cm下に残った。
- 相対接触高さ補正が下向き最大値へ張り付き、比例型の保持回復力と相殺した。ゲインを上げると定常偏差ではなくZ振動が再発した。

### 変更

- 共通Zの物体位置ループを`Kp=1.0`、`Kd=1.5`、追従加速度上限`0.04 m/s2`へ低帯域化した。HOLD後も内部軌道速度上限`0.01 m/s`を維持する。
- 離床marginは物体が3 mm上昇した時点で解除する。HOLD進入には3 mmの許容差を持たせるが、幾何学的な最終目標高度は変更しない。
- 高さ誤差へ直接掛けていた保持回復PDを無効化し、`Ki=0.8 N/(m s)`の片側・最大0.4 Nの低帯域積分へ変更した。離床margin解除直後から不足支持力を学習するため、HOLD閾値直下で停止する形状でも所定高度へ到達できる。
- 相対接触高さ制御はHOLD遷移時にも必ず基準を再ラッチする。既定値は`Kp=5.0`、`Kd=2.0`、加速度上限`0.03 m/s2`とし、絶対Z積分が静的な相殺分を補う。
- UNLOAD開始時は保持回復積分ゲインも0へ切り、支持feed-forwardのramp-downを妨げないようにした。

### headless MuJoCo検証

- 新形状（一辺/高さ600 mm、内側一辺532 mm、200 g）、4 N/台で120 simulation秒HOLDした。後半40秒の物体Zは平均1.00383 m、reference 1.00430 m、変動幅1.15 mm、Z速度標準偏差0.019 mm/sだった。傾き誤差平均は0.00247 rad、yaw rate標準偏差は0.00026 rad/sで、`lift_fault=0`を維持した。
- 旧ソリッド形状（一辺800 mm、高さ300 mm、200 g）でも既定の3 mm進入許容のまま60 simulation秒HOLDした。後半20秒のZ変動幅は4.12 mm、最終Zは0.85038 m、referenceは0.85058 m、傾き誤差平均は0.00110 radだった。
- 試験末尾の`transport timed out`は保持観察のため`transport_velocity_limit=0`を指定した意図的な終了であり、把持・Z制御faultではない。

### 不採用試験

- 相対滑り補正を完全に無効化すると、離床後に約160 mmオーバーシュートして落下判定へ入った。
- 保持回復`Kp`を20から40 N/mへ上げると20--30 mmの周期振動が再発した。
- 相対滑り補正上限を`0.03 -> 0.015 m/s2`へ下げると後半の減衰が不足し、約44 mm幅の振動になった。逆に`0.05 m/s2`へ上げるとHOLD直後の下向き補正が強すぎて落下した。
- HOLD進入許容だけを10 mmへ広げる案は旧形状を保持できたが、目標より10 mm低い時点で運搬状態へ移るため不採用とした。前HOLD積分で所定高度近傍まで持ち上げてから遷移する方式へ置き換えた。

### 離床追加力の独立ramp化

- 従来は`mg`と離床用0.4 N marginを同じ1.5秒のforce rampで増やし、3 mm離床した時点でmarginを即時に除去していた。離床追加力を独立状態へ分離し、`0.05 N/s`で増加、離床後は`0.10 N/s`で減少させるようにした。
- 保持回復積分の上限を`lift_hold_recovery_force_limit=0.4 N`として分離した。これにより離床marginを小さく、または低速にしても、HOLD中の静的不足力を補う能力は失われない。
- CSVへ`lift_breakaway_force`を追加し、booleanだけでなく実際に加えた鉛直力を記録する。
- 現在の既定形状（一辺/高さ600 mm、内側一辺418.626 mm、500 g）で試験した。物体は追加力0.0125 Nで離床し、0.4 N上限へ到達しなかった。60 simulation秒HOLDし、後半20秒のZ変動幅は3.76 mm、最終Zは0.99815 mだった。
- HOLD直後10秒のZ変動幅は19.2 mm残った。HOLD進入時の上向き速度が5.68 mm/s、内部referenceが物体より11.4 mm下に残っていたためで、離床marginのステップではない。次の低減対象はHOLD進入時の速度条件とreferenceのbumpless transferである。

## 2026-08-09 LIFT-HOLD-TRANSPORT連続制御

### 変更

- 初期Z速度指令を、無限に継続する速度指令ではなく有限高さ軌道の開始トリガとして扱うようにした。`mg` ramp完了時に開始高さ、最終高さ、軌道速度上限を同時にlatchし、以後は同じjerk/加速度制限付きreferenceをLIFT、HOLD、TRANSPORTで継続する。
- HOLD到達時に行っていたZ速度指令の急停止、observer積分・補正値のreset、observer filter時定数/Kp/Ki/補正上限の切替、鉛直feed-forward倍率の切替を廃止した。observerは全区間で同じ低帯域補正を使い、旧`lift_hold_observer_*` parameterは統一ゲインのfallbackとして互換性を残した。
- HOLDを制御モードではなく状態判定へ変更した。物体Z誤差3 mm以下、物体Z速度3 mm/s以下、内部reference誤差0.5 mm以下、reference速度1 mm/s以下が1秒継続した場合だけ`lift_hold_reached`を立てる。このflagはTRANSPORT操作の許可にだけ使用し、Z制御状態には触れない。
- common-slip feedbackの開始をHOLD flagから切り離し、物体上昇量だけで開始するようにした。既定では最終高さの3 mm手前でlatchするため、HOLD判定前に補正が立ち上がる。
- 共通XY速度feedbackもHOLD後だけでなくLIFT開始から有効にした。目標速度0の間は物体の横速度を減衰し、安定待ち中に把持編隊全体が流れるのを防ぐ。TRANSPORT開始時は同じfeedbackへ目標速度を連続的に与える。
- 長いLIFT安定待ち中に生じるゴム球の初期なじみ量をTRANSPORTの接触喪失量へ混ぜないよう、自律デモの安全監視だけをTRANSPORT開始時の相対接触高さへrebaseした。common-slip制御の基準値や制御出力は変更しない。

### headless MuJoCo検証

現行の一辺/高さ600 mm、内側一辺418.626 mm、500 g物体、法線力4 N/台で、目的地X=1.5 mの把持・運搬・設置デモをクリーン起動して実行した。

- HOLDは物体Z誤差2.8 mm、Z速度0.2 mm/sが1秒安定した後に成立した。LIFT中の物体XY移動量は10.1 mmだった。
- LIFT最終sampleからTRANSPORT最初のsampleで、物体Z目標と内部referenceは不変、feed-forward倍率は1.0のままだった。共通Z加速度差は`-0.000007 m/s2`、observer補正差は`+0.000007`、保持回復力差は`+0.000110 N`で、通常のrate更新分以外の段差はなかった。
- TRANSPORT中の物体Zは`0.99725..0.99999 m`。1.5 m先の到着時は`(1.4995, 1.0003, 1.0000) m`だった。
- 続くDESCEND、UNLOAD、RELEASE、RETREAT、UNWIND、VERIFYも通過し、`COMPLETE`へ到達した。最終物体poseは`(1.5023, 1.0009, 0.9499) m`、`lift_fault=0`だった。
- `catkin build bee --no-status`は依存21 packageすべて成功した。既存の`spinal/ApplyWrench`とclock skew warningだけが残る。最終変更後のPython構文checkと`git diff --check`も成功した。

## 2026-08-10 物体質量別の回帰試験

連続Z制御の回帰試験を、現行の一辺/高さ600 mm、内側一辺418.626 mmの中空三角柱に対して200 g、350 g、500 gの3条件で行った。物体質量以外は同じで、押し付け法線力は4 N/Bee、接触摩擦係数は0.5、運搬目的地はX=1.5 mとした。MuJoCoはheadlessでクリーン起動し、各Beeの`aerial_robot_base_node`が存在し、`attitude_controller`が`running`であることを確認してからデモを開始した。

| 物体質量 | HOLD | 1.5 m運搬/載置 | LIFT中XY移動量 | TRANSPORT中Z範囲 | 運搬中最大roll/pitch傾き | 最終pose [m] |
| --- | --- | --- | ---: | ---: | ---: | --- |
| 200 g | 成功 | `COMPLETE` | 36.3 mm | 0.99719--0.99998 m | 0.231 deg | (1.4888, 0.9935, 0.9499) |
| 350 g | 成功 | `COMPLETE` | 26.0 mm | 0.99728--1.00001 m | 0.369 deg | (1.4921, 0.9944, 0.9499) |
| 500 g | 成功 | `COMPLETE` | 16.0 mm | 0.99719--0.99996 m | 0.349 deg | (1.4994, 0.9941, 0.9499) |

- 3条件ともDESCEND、UNLOAD、RELEASE、RETREAT、UNWIND、VERIFYまで通過し、`lift_fault=0`で終了した。
- LIFT最終sampleからTRANSPORT最初sampleでは、3条件とも`lift_reference_z/velocity/acceleration`に段差はなかった。共通Z加速度の差は`-8e-6..+1e-6 m/s2`、observer補正の差は`0..14e-6`で、HOLD成立による制御切替ステップは見られなかった。
- 軽い200 gはLIFT中のXY移動量が最も大きく、500 gの約2.3倍だった。完走には影響しなかったが、軽量物への並進外乱感度として今後も監視する。
- 最初の一括試験は`/bee3/ground_truth`の出現だけを起動完了条件にしたため、controller起動前にデモが始まりHOVER timeoutとなった。これは物体制御前の無効試験とした。また350 gの再起動1回目でBee3制御ノードが初期化中に終了したため、完全停止後に再起動した。以後の重量別評価は3台のノードとcontroller状態を確認した有効試験のみを使用する。

## 2026-08-12 離床追加力の削除

- 旧制御は`m(g+a_ref)`とは別に、最大0.4 Nの開ループbreakaway marginを離床時だけ加えていた。上向き軌道加速度がすでに重力以上の目標力を生成するため、重複する開ループ項が必要かを検証した。
- 削除前に`lift_breakaway_force_margin=0`とし、200 g、350 g、500 gのすべてで離床と全工程の完走例を確認した。500 gは別の1試行で目的地下降中に平均接触面滑り60.3 mmで安全停止したが、これはHOLDとTRANSPORT完了後で、離床可否とは無関係だった。
- `lift_breakaway_force_margin/ramp_rate/release_rate/release_rise`、active/force状態、離床高さでの解除処理、CSVの2列を削除した。この時点の物体Z目標力は`m_object * (ramp * g + a_ref) + hold_recovery_force`としたが、後節で`hold_recovery_force`もZ軌道PIDのI項へ統合した。

### 削除後のheadless MuJoCo回帰試験

| 物体質量 | 結果 | LIFT中XY移動量 | TRANSPORT中Z範囲 | 最大平均接触面滑り | 最終pose [m] |
| --- | --- | ---: | ---: | ---: | --- |
| 200 g | `COMPLETE` | 19.1 mm | 0.99717--0.99996 m | 24.4 mm | (1.5057, 0.9932, 0.9499) |
| 350 g | `COMPLETE` | 56.2 mm | 0.99722--1.00000 m | 39.3 mm | (1.4749, 1.0038, 0.9499) |
| 500 g | `COMPLETE` | 12.6 mm | 0.99718--0.99995 m | 59.6 mm | (1.4980, 1.0036, 0.9499) |

- 3条件ともHOLD、TRANSPORT、DESCEND、UNLOAD、RELEASE、RETREAT、UNWIND、VERIFYを通過し、`lift_fault=0`で終了した。CSVにbreakaway関連列が残っていないことも確認した。
- 350 gの載置後X誤差は25.1 mmで、VERIFY許容30 mm内だが他の2条件より大きい。また500 gの滑り59.6 mmは安全閾値60 mmに近い。これらは離床追加力とは別の運搬/下降制御の再現性課題として残る。
- 起動待ち中にBee2制御ノードが終了した200 gの1試行は、物体制御前の無効試験とした。最終評価では3台のcontrollerが`running`で、各odometryとobject poseの実データが届いていることを確認してから実行した。

## 2026-08-12 物体Z軌道PIDへのI項統合

- 旧`hold_recovery_force`はP/Dが使う移動中の`lift_reference_z`ではなく、最終HOLD高さと物体Zの誤差を離床中から積分していた。そのため開ループbreakaway力ではないが、Z軌道PIDとは別の追加力生成器になっていた。
- P/I/Dを同じ移動reference誤差に統一した。実装式は`e_z=z_ref-z_object`、`e_v=v_ref-v_object`、`a_I += Ki*e_z*dt`、`a_object=a_ref+Kp*e_z+Kd*e_v+a_I`、`Fz_target=m_object*(ramp*g+a_object)`である。
- 既定値は`Kp=1.0`、`Kd=1.5`、`Ki=0.20`、P/D加速度上限`0.04 m/s2`、I加速度上限`+/-0.08 m/s2`とした。I項が上限にある方向の誤差は積分せず、反対方向の誤差だけを受け入れるconditional integrationをanti-windupに使う。
- I項はLIFT、HOLD、TRANSPORTでresetしない。物体が目的地台座に接地したことを高さと速度で確認した時点でのみ0にし、積分も無効化する。次のLIFT開始で再有効化し、UNLOAD中に残留上向き加速度を出さない。
- CSVの`lift_hold_recovery_force`を`lift_trajectory_integral_acceleration`へ置き換え、力[N]ではなくPIDが生成するI加速度[m/s2]を記録するようにした。observer Z外力は従来どおり別の低帯域・有界補正とし、Z PIDのI項とは混ぜない。

### Ki比較とheadless MuJoCo回帰試験

- 500 gで`Ki=0.15`を2回試し、どちらもHOLDと1.5 m運搬は完了したが、下降終端で平均接触面滑りが60.1/60.2 mmとなり60 mm安全閾値で停止した。
- `Ki=0.10`では必要な定常I値に達するまでが遅く、HOLD成立時の滑りが`19--20 -> 27 mm`へ悪化し、下降中に61.0 mmで停止したため不採用とした。
- `Ki=0.20`では500 gのHOLD成立時滑りが14.7 mm、全工程の最大滑りが54.1 mmへ減少し、載置とVERIFYまで完了した。

| 物体質量 | 結果 | HOLD時I加速度 | HOLD->TRANSPORTのI差 | TRANSPORT中Z範囲 | 最大平均接触面滑り | 最終pose [m] |
| --- | --- | ---: | ---: | ---: | ---: | --- |
| 200 g | `COMPLETE` | 0.03725 m/s2 | +0.000028 m/s2 | 0.99728--1.00000 m | 21.4 mm | (1.5030, 0.9871, 0.9499) |
| 350 g | `COMPLETE` | 0.00827 m/s2 | +0.000029 m/s2 | 0.98885--0.99999 m | 27.1 mm | (1.4921, 1.0012, 0.9499) |
| 500 g | `COMPLETE` | 0.03522 m/s2 | +0.000027 m/s2 | 0.99730--1.00001 m | 56.5 mm | (1.4794, 0.9882, 0.9499) |

- 3条件とも`lift_fault=0`で、DESCEND、UNLOAD、RELEASE、RETREAT、UNWIND、VERIFYを通過した。HOLD->TRANSPORTでI項にreset/段差がなく、最終CSV sampleでは接地後resetによりI項は3条件とも0だった。
- 350 gのTRANSPORT中に一時的なZ=0.98885 mまでの低下があり、最終的には回復して完走した。重量に対する単調な傾向ではなく、接触の初期なじみによる試行間ばらつきとして引き続き監視する。
- 最終500 g試験の最初の起動ではBee2の`flight_state`と`uav/cog/odom`が配信されず、制御開始前の`wait_for_data`で終了した。これを無効試験とし、controllerに加えて各Beeの`flight_state/ground_truth/uav/cog/odom/grasp_forces/grasp_contact_states`とobject poseの実データを確認したクリーン再起動で最終値を取得した。

## 2026-08-27 MPC接触位置の動的モデル反映

### 変更

- 従来のwhole-body MPCは、初期把持時に計算した物体重心から各Beeの4球代表中心までの腕と、各Bee重心までの腕を全工程で固定していた。球が面上を滑ると、実際の力の作用線とMPCのモーメント予測が一致しなくなる構成だった。
- 各再計画時に、現在の物体pose、Bee odometry、observerのCG odometryから両方の腕を物体座標で再計算するようにした。初期接触位置は滑り監視・既存の局所位置補正用の基準として固定したままであり、動力学に使う接触位置だけを現在値へ追従させる。
- acadosモデルへ18次元のonline parameter（接触腕9成分、機体重心腕9成分）を追加し、全shooting nodeへ毎回設定する。solver生成・Cコードcompileは初期化時の1回だけで、接触位置更新による再生成はない。
- CasADi backendもNLP parameterへ同じ18成分を追加した。SciPy backendはsolve開始時に更新されたnumpy幾何を使う。3 backendの外部APIを共通化し、接触腕と重心腕の片方だけを更新する操作や非有限値は拒否する。

### 検証

- planner単体テスト5件に成功した。追加テストでは、接触腕のZを10 mm変えた後に同一acados solver instanceで解けること、planner内部の幾何が更新されること、同じ非対称接触力に対する予測角加速度が変わることを確認した。
- `python3 -m py_compile`、`git diff --check`、`catkin build bee --no-status`に成功した。catkinは依存21 packageすべて成功し、既存の`spinal/ApplyWrench` warningだけが残った。
- real-time-factor 1.0、500 g物体のheadless MuJoCo試験では、離床、LIFT HOLD、TRANSPORTへの遷移に成功した。動的幾何更新中のacados solveは主に1.2--1.7 msで、物体はZ約1.002 mを維持しながらX=0.274 mまで運搬した。MPC例外、幾何更新エラー、infeasibleはなかった。
- 同試験はその後Bee1 momentum observerが1.00 s staleとなり、既存の安全監視がFAILSAFE_HOLDを作動して終了した。動的接触モデル起因の停止ではないが、1.5 m運搬・載置までの完走確認は未達である。
- real-time-factor 0の再試験は、simulation timeがwall timeより約7--8倍速く進み、observer/contact loopの実時間条件が崩れてLIFT HOLD滑り判定となったため、有効な性能評価には使わない。

### 2026-08-28 再評価

- 500 g、real-time-factor 1.0でクリーン起動し、3台の`attitude_controller`が`running`であることを確認して再試験した。planner単体テスト5件も再実行してすべて成功した。
- 既定`observer_timeout=1.0 s`ではLIFT/HOLDに成功し、TRANSPORTでX=0.179 mまで進んだ後、前回と同じく`bee1 momentum observer is stale (1.00 s)`でFAILSAFE_HOLDとなった。停止直前までacados solveは主に1.2--1.7 ms、物体Z変動幅は1.50 mm、最大roll/pitchは0.062 deg以下であり、MPC失敗ではなかった。
- 停止後に外部subscriberで測った3台の`estimated_external_wrench`は約100 Hzだった。別の再試験では3台observerと`/clock`をrosbagへ記録し、observerの最大simulation-time間隔はBee1 50 ms、Bee2 72 ms、Bee3 59 msだった。publisher停止ではなく、WSL上のPythonデモプロセスに約1秒のcallback遅延または単調時刻の不連続が生じ、1秒watchdogに触れている可能性が高い。
- 制御性能を分離するため、コードの既定値は変更せず、実行時だけ`_observer_timeout:=2.0`として再試験した。この条件では前回の停止位置を越えたが、TRANSPORT 44.1 s、X=0.926 mで物体がHOLD目標より30.0 mm低下し、既存の落下安全判定で停止した。最終poseは`(0.9256, 0.9009, 0.9728) m`だった。
- timeout 2秒試験のTRANSPORT中、物体のroll/pitchは最大0.056/0.071 deg、yaw変動幅は0.101 degで、回転安定性は良好だった。一方でYは`1.000 -> 0.901 m`、Zは`1.003 -> 0.973 m`へ移動した。3台すべての物体座標系接触腕Zは約16.5 mm増加し、球/機体が物体に対して上へ滑ったことを示す。
- 動的接触位置はこの約16.5 mmの変化を各MPC再計画へ反映できており、solver再生成やinfeasibleは発生しなかった。しかし現在のOCPは接触位置そのものの許容範囲や滑り速度を制約せず、更新された位置を新しい有効接触として追従するだけである。したがってモデル整合性は改善するが、滑り防止にはならない。
- 固定接触位置だった直前のacados試験`bee_transport_place_acados_retry4.csv`も、TRANSPORT終端で`x=0.932 m, z=0.9725 m`まで低下していた。今回の`x=0.926 m, z=0.9728 m`とほぼ同じであり、動的接触位置化だけでは支配的なZ滑りを改善していない。Y偏差は固定時9 mm、今回99 mmだが、各1試行のため動的化による悪化とはまだ断定しない。

### 再評価後の判断

- 動的接触位置パラメータ化は、実接触腕でモーメントと機体加速度を予測するために必要であり、実装自体は維持する。
- ただし「現在位置をモデルへ入れる」だけでは把持維持条件にならない。次の主要課題は、初期接触位置からの面内変位/速度を状態機械の安全判定だけでなく、少なくとも接触可行領域としてOCPへ入れるか、物体位置追従を担う外側ループと接触力実現誤差を見直すことである。
- WSL評価では1秒observer timeoutが実データ欠落なしでも再現性よく誤作動した。安全閾値を単純に恒久拡大する前に、callback ageをCSVへ記録し、ROS message stamp/simulation timeとwall monotonic ageを分離して監視する必要がある。

## 2026-08-28 MPCモデル外乱の閉ループ化

### 変更

- 従来のwhole-body MPCにもmomentum observerによる内側の接触力補正はあったが、`planned_force - observed_force`は最適化後の各Bee加速度指令へ加算されるだけで、次のMPCが使う物体運動方程式には戻っていなかった。このためMPC自身は毎回同じ公称モデルを解き、持続する実現誤差を接触力配分へ織り込めなかった。
- mocap object twistの差分から実測並進/角加速度を求め、直前に適用したMPC sampleの予測並進/角加速度との差をinnovationとする6軸外乱推定器を追加した。推定値は低域化・ノルム制限し、次回horizon全nodeで一定の加法外乱加速度として物体の並進/回転運動方程式へ渡す。
- acadosのonline parameterを18から24次元へ拡張し、現在接触腕9、現在Bee-CG腕9、外乱並進3、外乱角3をsolver再生成なしで更新する。CasADi NLP parameterとSciPy numpy dynamicsにも同じ外乱を実装した。
- 台座反力を把持外乱として学習しないよう、支持力ramp中は推定器をresetして速度履歴だけ同期する。既定値は時定数0.75 s、並進外乱上限0.10 m/s2、角外乱上限0.50 rad/s2とした。
- 推定外乱をROSの`planner/external_linear_acceleration`、`planner/external_angular_acceleration`へpublishし、実測加速度と推定外乱の12成分を既存CSVへ追加した。legacy strategyでは追加列が0となり制御は変わらない。

### 検証

- acados単体テスト5件に成功した。現在接触位置と6軸外乱を同時更新しても同じsolver instanceを維持し、numpy dynamicsの加速度差が指定外乱と一致し、解が制約内であることを確認した。
- CasADi/sqpmethodとSciPy backendも6軸外乱付き静止問題を制約内で解いた。Python構文checkと`catkin build bee --no-status`も成功し、catkinは依存21 packageすべて成功した。既存の`spinal/ApplyWrench` warningだけが残った。
- 500 g、real-time-factor 1.0、`observer_timeout=2.0 s`でheadless MuJoCo試験を行った。最初の起動はBee2推定ノードがMPC開始前にEigen行列次元assertで終了したため無効とし、3台の制御ノードを確認したクリーン再起動の結果だけを評価した。
- 有効試験では離床、HOLD、TRANSPORTに成功し、acados solveは主に1.1--2.4 msだった。推定外乱はMPC計画へ反映され、終盤の下向きZ外乱が上限`-0.10 m/s2`へ達すると、計画鉛直合力は公称mg=4.905 Nから約4.965 Nへ増加した。
- 物体はX=1.014 mまで運搬した後、ZがHOLD目標から30.2 mm低下して安全停止した。外乱を持たない動的接触位置版の前回停止X=0.926 mより約88 mm先まで進んだが、1.5 m運搬・載置には未達である。
- TRANSPORT中、物体Zは`1.0027 -> 0.9723 m`、Yは`1.000 -> 0.936 m`となった。3台の接触代表中心は物体に対して約17.9--18.1 mm上へ移動しており、支配的な問題は引き続き面上滑りである。推定外乱上限到達後も低下したため、今回の小さいモデル補正だけでは接触力実現不足を相殺しきれない。

### 判断

- 「実状態と公称モデルの差を次のMPCへ渡す」経路は実装できた。従来の後付けobserver補正とは異なり、MPC自身が外乱込みの物体加速度制約を満たすよう接触力を再配分する。
- ただしmomentum observerの物体Z力合計は終盤も約5.08 Nと重量以上を示す一方、mocap物体は約5.2 mm/sで下降していた。observer力だけをそのままMPC外乱へ変換すると誤った支持判定になるため、今回のmocap運動残差を主にする構成は維持する。
- 次の調整候補は外乱上限を段階的に上げることと、外乱推定とは別に接触面内相対速度/可行領域を把持維持条件へ入れることである。単純な上限拡大はBeeだけが上へ滑る可能性もあるため、現在のCSVで相対接触高さと同時評価する必要がある。

## 2026-08-30 古典制御とMPCの比較、および論文方針

### MPCが古典制御に及ばなかった理由

- 現在`legacy`と呼んでいる制御は単一のPIDではない。物体の有限高さ軌道、`m(g+a_ref)` feed-forward、同一referenceに対する物体ZのP/I/D、低帯域observer力補正、3台共通の接触面滑り抑制、3台間の差動接触高さ制御、局所モーメント制御、物体roll/pitch/yaw制御を連続的に組み合わせた階層制御である。
- 一方、現在のwhole-body MPCは各Beeの4球接触を1個の代表接触力へ縮約し、その力がそのまま実現されることを前提とする。ゴム球のばね圧縮、4球間の荷重偏り、静止摩擦から動摩擦への遷移、接触力の実現遅れはOCP状態に含まれない。MPC動作中は成功実績のあるlegacyのZ PID、共通slip、局所/物体モーメント制御を外し、計画力をBee加速度へ変換した後にobserver/admittance補正を加える構成だった。
- 500 gの有効ログ`bee_unified_z_pid_final_500_clean.csv`と`bee_mpc_disturbance_500g.csv`を比較した。TRANSPORT中の共通Z加速度平均はlegacy/MPCとも約0.898 m/s2であり、平均支持指令不足が主因ではなかった。しかしlegacyの範囲は`0.898507--0.898646 m/s2`、標準偏差`1.54e-5 m/s2`だったのに対し、MPCは`0.876747--0.914664 m/s2`、標準偏差`6.37e-3 m/s2`だった。MPCの変動幅は約273倍、標準偏差は約414倍である。
- 目標鉛直合力の変動幅もlegacyの0.000064 Nに対し、外乱補償付きMPCは0.119 Nだった。MPCは短いhorizonごとに物体Z加速度を主に`+/-0.02 m/s2`の制約境界付近で変化させ、平均力が十分でもゴム球を微小に除荷・再荷重して静止摩擦を崩したと判断する。
- legacyではTRANSPORT中の物体Zが`0.99730--1.00001 m`に収まり、1.5 m運搬・載置まで完了した。MPCでは`1.00367 -> 0.97235 m`へ低下し、X=1.014 mで落下安全停止した。
- MPC終盤のmomentum observer物体Z力は約5.08 Nで、500 g物体の重量4.905 Nを上回っていたが、mocap物体は約5.2 mm/sで下降していた。したがってobserverの力だけでは、柔軟接触で実際に物体へ伝達された支持力を判定できない。mocap運動残差をMPC外乱へ戻す実装は停止距離を延ばしたが、接触モデルの構造誤差を解消するには至らなかった。

### 採用方針

- 論文と今後の実機検証は、完走実績のある`legacy`制御を中心に進める。`~control_strategy`の既定値も引き続き`legacy`とする。whole-body MPC実装、動的接触位置、6軸モデル外乱、テストと診断は比較対象・将来拡張として残し、削除しない。
- 把持へのMPC自体が不適切なのではない。MPCは物体軌道、目標物体wrench、摩擦・推力制約、3台への力配分に向く。一方、今回の柔軟な4球接触を5 HzのMPCだけで高速安定化するのはモデル誤差が大きい。将来再利用する場合は、MPCを上位の物体wrench計画へ限定し、legacyのZ PID、force servo、slip抑制を高速inner loopとして残す階層構成が妥当である。

### 現在の制御構成の整理

- 各Beeのmomentum observerは各機体のcontroller内で動作し、Bee重心における合成6軸外レンチとCG odometryをpublishする。中央のPythonデモノードは3台分を集約し、接触前baselineを除去して座標変換し、作用反作用と接触腕を用いて物体の合力・合モーメントを計算する。4球それぞれの力をobserverから分離しているわけではなく、force-siteとばね圧縮量は診断専用である。
- 中央制御が生成する物体目標加速度・角加速度は、`F_object_des=m_object(a_object_des+g)`および剛体回転方程式を通じて目標物体wrenchを決めることと等価である。ただし3方向の法線押し付け力は物体上でほぼ相殺され、合成物体wrenchには現れない。したがって中央が決めている量は「目標物体wrench＋摩擦能力を作る内部把持力」である。
- 論文上は、各機体での分散外レンチ推定、中央での物体wrench統合と内部力配分、各機体のローカル加速度・姿勢追従からなる階層型wrench-based cooperative controllerと整理する。完全分散制御ではなく、低レベル分散・物体レベル集中型である。
- 論文の主張候補は、精密な個別接触力センサや4球接触モデルを使わず、momentum observer、mocap物体状態、受動ばね付きゴム球、共通/差動wrench feedbackによって、3台の空中ロボットによる爪なし摩擦把持、運搬、載置、解放を実現したことである。200/350/500 g回帰、Z積分・共通slip・姿勢モーメント制御のアブレーション、MPC比較を評価軸とする。

### legacy実行方法

MuJoCoはonline observer/controlの実時間条件を維持するため`real_time_factor:=1.0`で起動する。WSL上で確認された1秒watchdogの誤作動を避ける評価時設定として、デモだけ`observer_timeout:=2.0`を明示する。

```bash
roslaunch bee mujoco_multi_module.launch \
  headless:=true launch_rviz:=false \
  spawn_destination_pedestal:=true object_mass:=0.5 \
  real_time_factor:=1.0
```

```bash
rosrun bee mujoco_triple_transport_place_demo.py \
  _control_strategy:=legacy \
  _observer_timeout:=2.0 \
  _wrench_log_path:=/tmp/bee_legacy_transport_500g.csv
```

## 2026-09-02 機体数・形状・重量を変えたlegacy制御試験

### 追加した構成

- 既存の3台・三角柱に加え、4台・直方体および5台・正五角柱を同じlegacy階層制御で扱えるようにした。各Beeは対応する側面法線上に配置し、物体中心から側面までの距離を形状から計算する。
- MuJoCo scene composerへ正五角柱の凸mesh、体積、質量、正確な正多角柱慣性を追加した。五角柱の既定寸法は一辺0.60 m、高さ0.50 m、質量0.80 kgである。
- 5台用launchとtransport/placeデモを追加した。5台の法線力・Z差動加速度を個別topic/CSVへ出し、既存の最小ノルム配分、物体roll/pitch/yaw制御、支持移行、解放手順をそのまま一般化した。
- 4台以上の同時起動では、shared backend開始直後に`aerial_robot_base_node`がEigen行列積assertで落ちる試行があった。各Beeのcontroller初期化完了フラグを追加し、さらにbackend開始前の既定猶予をparameter準備3秒、backend準備1秒とした。この猶予を使った4台・5台試験では起動失敗はなかった。

### headless MuJoCo試験結果

全条件で目的地を初期位置からX方向へ0.40 m移し、把持、台座支持から摩擦支持への移行、約50 mm離床、運搬、下降、荷重移行、解放、退避、VERIFYまで実行した。`real_time_factor=1.0`、摩擦係数0.5、legacy制御を使用した。

| 機体・物体 | 質量 | 結果 | 最終XY誤差 | LIFT中最大上昇 | 把持中最大abs(roll/pitch/yaw) | LIFT時間 | TRANSPORT時間 |
| --- | ---: | --- | ---: | ---: | --- | ---: | ---: |
| 3台・三角柱（一辺0.60 m、高さ0.60 m） | 0.20 kg | `COMPLETE` | 1.76 mm | 50.05 mm | 0.142 / 0.209 / 1.426 deg | 59.57 s | 25.60 s |
| 4台・直方体（0.60 x 0.60 x 0.50 m） | 0.50 kg | `COMPLETE` | 2.77 mm | 47.55 mm | 0.133 / 0.095 / 1.354 deg | 21.55 s | 24.96 s |
| 5台・正五角柱（一辺0.60 m、高さ0.50 m） | 0.80 kg | `COMPLETE` | 2.53 mm | 49.01 mm | 0.032 / 0.152 / 1.101 deg | 14.30 s | 24.69 s |

- 最終poseは順に`(0.398606, 1.001072, 0.949916)`、`(0.397778, 1.001650, 0.899867)`、`(0.397504, 1.000419, 0.899861) m`だった。全条件で解放後も目的地台座上に残った。
- 定常法線力は3台条件で約3.86 N/台、4台条件で約5.87 N/台、5台条件で5.85--5.88 N/台だった。0.80 kg五角柱の台座支持荷重7.85 Nも連続的に把持側へ移せた。
- 3台・200 gだけLIFT収束が長い。物体上昇後の共通Z/slipループがHOLD判定へ収束するまで約60秒を要したためであり、運搬時間や最終精度は他条件と同程度だった。機体数または重量に対して性能が単調に悪化する結果ではなかった。
- 評価ログは`/tmp/bee_transport_place.csv`、`/tmp/bee_four_box_transport_place.csv`、`/tmp/bee_five_pentagon_transport_place.csv`に保存した。

## 2026-09-20 develop/beeへ復帰・実機用3台把持入口

- DAコミットad851f0aを専用branchへpushした後、ユーザー指示でdevelop/beeへ復帰。切替時にlog.mdのみ競合したため、`grasp log before returning from differential-allocation to develop/bee`という名前でlog.mdだけstashし、上記9/2の把持記録を復元。stashはバックアップとして保持。他の未コミット把持差分は維持。DAコミットはこのbranchへ取り込んでいない。
- `real_grasp_demo.py`を追加。従来の3台キーボードobserver/ACC古典制御を継承し、接近・押し付け・キー操作による持ち上げとHOLD後のXYZ/yaw操作を対象にした。自動運搬/設置、4/5台、MPCの実機化は対象外。
- 実機はbaselink/CoG estimator odometryとCoG外レンチ、物体mocap PoseStampedを利用。namespace/物体topicを設定可能にし、ground_truth、接触force-site/touch、外力注入の購読/出力を作らない。物体姿勢の原点は幾何中心、座標は共通worldを要求し、TFやマーカー原点補正は上流で行う。物体質量は実測object_massを支持制御にも反映。
- 実機用control YAMLは重力補償が既定OFFのため、Z ACCで積分項を切り離すと機体重量の支持が消える。bringupへ明示opt-inの`grasp_gravity_compensation`を追加し、実機実行前にもparameterを確認。通常起動の既定値・PIDゲインは変えていない。起動前設定と再ビルドが必要。
- 既定はexecute=falseの入力確認のみ。実行にはconfiguration_verified=true、対話TTY、全台HOVER、指令subscriber接続が必要。デモはarm/takeoff/landを送らない。実測geometryとtopic/frameは未提供のため、YAMLは未検証の例示値・実行無効のまま。
- フレーム・Quaternion・finite値・source時刻/受信時刻を検査。制御中の入力途絶/制御ループheartbeat停止、HOVER離脱、加速度上限超過で中止ラッチ。HOVER継続機体にXYZ速度0を送りACCを解除、姿勢を急に巻き戻さず、LAND/STOP機体は介入しない。PC/通信/FC故障には独立RC/FC watchdogが必要。中止による物体落下リスクは残る。
- `test_real_grasp_demo.py`の14件のオフラインmock試験に合格（入力異常、check-only無指令、namespace、sim_time/外力禁止、ACCゲート、中止ラッチ、heartbeat、MuJoCo interface維持等）。全Python構文、XML/YAML、git diff --check通過。入力確認launchのexecute=falseと実機重力補償opt-inのparameter展開を確認。実機飛行・シミュレーション飛行は今回未実施。
- 操作手順・インターフェース契約・限界は`robots/bee/config/REAL_GRASP.md`。既存未コミット差分を含め、今回はコミットしていない。
- `catkin build bee --no-status`は依存を含む21 packageすべて成功（2分21秒）。既存ApplyWrench message生成・空generate_messages・Gazebo非推奨warning、CMAKE_PREFIX_PATH差異および微小clock skewのwarningが残った。実機FC firmwareのビルド/書込み・通信互換性試験は行っていない。
