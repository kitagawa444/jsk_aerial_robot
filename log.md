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
