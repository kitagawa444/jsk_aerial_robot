# 接触力応答の同定

2026-11-16以降、Beeへの指令は汎用の`command`です。
同定runnerは中央の共通参照の3D力を励起します。把持中の法線は `acceleration_orientation` のX軸です。
下流設定は`--reference-tracking-config`、標準ファイルは`config/controller/force_control.yaml`です。
従来の`--controller-config`にある下流の旧設定キーもrunnerでは変換して保存します。
操作・校正の仕様は[共通指令追従](CONTROLLER_REFERENCE.md)を参照してください。

`identify_grasp_force_response.py` は、シミュレーション／実機共通の bag 解析器です。
指令を送らず、既存の `BeeCommand` と `ControllerStatus` から、各 Bee の
法線・接線上向き・接線横向きの閉ループ応答を推定します。
結果をゲインや制約へ自動反映する処理はありません。

## 同定するもの

- 入力 `u`: Bee が物体へ与える指令力 `BeeCommand.force` [N]。
- 出力 `y`: `-tracking_status.external_force` [N]。observer は機体が受ける力なので符号を反転。
- 解析座標: 各区間開始時の法線を固定し、内向き・世界鉛直を接平面へ射影した上向き・横向きの3軸。
- モデル: `tau * dy/dt + y = gain * u(t - delay) + offset`。
  `tau` は変化量の約63%へ進む時定数、`delay` はモデル上のむだ時間。
  モデル上の90%／95%到達時間は `delay + 2.303*tau`／`delay + 2.996*tau`。
  ステップの最終変化量への到達時間であって、真の力が指令値の95%へ到達する保証ではありません。

これは**内側制御、接触、observer、外力フィルタをまとめた「指令→観測値」の応答**です。
現在の外力フィルタの時定数0.2秒も含みます。真の接触力だけの時定数、接触剛性、
物理的な通信遅延を分離した結果ではありません。observer のバイアスと真の接触力の
誤差は、この記録だけでは区別できません。

得られた値はその押付け量・姿勢・接触・荷重・PIゲイン近傍の小信号モデルです。
現在の解析は `enable_force_control` と `acceleration_bounds` が有効な
区間を対象とし、Preload の旧法線制御と混ぜません。Pのみ／PIの混在も拒否します。
接平面の上向きが定義できない鉛直法線は対象外です。

現在の中央は支持準備が終わるまで `force_integral_enabled=false`、保持中はtrueです。
シミュレーションrunnerの `--force-mode p` ではHoldに入った時点から積分を無効にして
整定・励起し、支持移行中と同じ積分なしの力制御も測れます。既定値は `pi` です。
P版も保持状態での試験であり、支持移行の接触状態・荷重変化全体を再現した試験ではありません。

## ファイルと依存

| ファイル | 用途 |
| --- | --- |
| `script/force_response_model.py` | NumPy/SciPyだけで動く数値モデル・fit |
| `script/identify_grasp_force_response.py` | bagから時刻整合、学習、検証、図・CSV・JSON・YAMLを出力 |
| `script/record_grasp_force_response.py` | 実機でも使える受動的な記録。publish／arm／service指令なし |
| `config/grasp/force_identification.yaml` | 名前、手動解析区間、条件メタデータの設定例 |
| `script/grasp_force_identification_sim.py` | MuJoCo専用の小振幅入力relay |
| `test/run_force_identification.py` | 専用localhost masterとMuJoCoの起動・記録・終了 |

ROS1環境をsourceし、`rosbag`, `rospy`, 生成済み `bee.msg`, NumPy, SciPy,
Matplotlib, PyYAMLを使います。解析はROS masterを必要としません。

## シミュレーションの再実行

```bash
source /home/kitagawa/ros/jsk_aerial_robot_ws/devel/setup.bash
BEE_PACKAGE="$(rospack find bee)"
python3 "$BEE_PACKAGE/test/run_force_identification.py" \
  --output /tmp/bee-force-id-new --port 12311
OPENBLAS_NUM_THREADS=1 python3 "$BEE_PACKAGE/script/identify_grasp_force_response.py" \
  /tmp/bee-force-id-new/flight.bag --output /tmp/bee-force-fit-new
```

出力先は毎回新しいディレクトリを指定します。専用runnerは4台・0.5kg直方体で離陸、
把持、持上げ、Holdまで進め、Holdの120秒後から励起します。ゆっくりした姿勢整定を待つ
ための値で、`--settle-time`で変更できます。既存の運搬参照は無効です。
**このrunnerはシミュレーション内で飛行を開始します。実機へ転用する実行入口ではありません。**
localhostかつ11311以外のmaster、`use_sim_time`、全機のMuJoCo HW pluginを確認します。
完了後に自身が起動したbagとlaunchを終了します。運搬・着地成功を評価する試験ではありません。

標準入力は最大0.16N、スルーレート1N/s、1ブロック2.5秒、各軸13ブロックです。
法線は全機で同じ追加押付け、接線は対称配置で符号を交互にして合力・モーメントを
おおむね相殺します。一般配置の厳密な零空間入力ではありません。
法線は基準の押付けを減らしません。法線上限、摩擦、観測法線力による支持容量、
推力余裕、力の変化率を検査し、逸脱時は元の指令を送り、その試験区間を不合格にします。
入力チェックを通ったことだけでは、滑りゼロや実機の安全を証明しません。

入力の初め20秒でパラメータを学習し、残り12.5秒の再入力で、再fitせず予測を検証します。
生成launch・設定・制御バイナリ等のハッシュと実行パラメータを `summary.json` に残します。

## 実機へ再利用する手順

現時点では実機試験を行っていません。2026-11-12のモーター回転禁止指示は継続中です。
以下の記録・解析ツール自体には飛行／励起を始める機能はありません。

1. 対象機体、書込み済みファームの識別情報と照合状況、ホストのコード版、物体質量、
   接触姿勢・足材、電圧、PIゲイン、observer/filter設定を記録します。
   recorderのパラメータ保存だけでは実機ファームの一致は確認できません。
   指令と観測のheader時刻は同じ時計に同期させます。別PCの時計のずれは
   見かけのむだ時間に混ざるため、通信・制御の遅れと取り違えないようにします。
2. 別途許可された保持試験で、普段の指令系を通じて小さな法線／接線入力を順に与えます。
   初めからシミュレーションと同じ振幅が妥当とは扱わず、押付けと摩擦・推力の余裕に
   合わせて決めます。自由飛行中・接触前・接触が変わる区間では同じモデルをfitしません。
   他軸をできるだけ一定にし、正負の繰返し（法線は押付けを下回らない）を取り、
   学習用と検証用の両方に入力変化を含めます。変化がないbagでは同定できません。
3. その間を受動的に記録します。例の `bee1 ... bee4` は実際のnamespaceに合わせます。

```bash
python3 "$BEE_PACKAGE/script/record_grasp_force_response.py" \
  --output /tmp/bee-hardware-force-record-new \
  --source hardware --names bee1 bee2 bee3 bee4 \
  --label "airframes_firmware_payload_contact_condition" --duration 150
```

4. `force_identification.yaml` をコピーし、`robot_names` と条件を記入します。
   マーカーがないbagでは `segments` に実際の入力区間を指定します。
   `time_origin: bag_start` ならbag開始からの秒数、`absolute` ならheader時刻です。
   `validation_after` は区間開始から何秒後を未使用検証にするかです。
   `integral_enabled` は記録中の実際のモードに合わせます。

```bash
OPENBLAS_NUM_THREADS=1 python3 "$BEE_PACKAGE/script/identify_grasp_force_response.py" \
  /tmp/bee-hardware-force-record-new/flight.bag \
  --config /absolute/path/to/filled_force_identification.yaml \
  --output /tmp/bee-hardware-force-fit-new
```

5. `results.json`／`model.yaml` の各 `accepted` と `reasons`、実測・予測の図を確認します。
   同定値だけでなく振幅・静的押付け・姿勢・電圧が変わった試験でも検証してから使います。
   シミュレーション値を実機の確定値としてコピーしません。

## 解析の検査と出力

- 指令の符号・world座標・有効モード・校正済み観測を検査します。
- header時刻で整合し、入力には未来の指令を使いません。時刻巻戻しは拒否します。
  同一時刻の重複と、現実装で制御更新なしのときに出るゼロ観測を除外します。
  対象は押付け済み区間なので、外力ノルム0.1N以下も除外します。
  150ms以上の欠損を含む窓はfitせず、区間を分ける必要があります。
  再サンプルの刻みだけでなく、元の指令／観測の周期と最大間隔も出力します。
  補間を細かくしても実測の時間分解能は上がりません。
- 固定座標から法線が5度以上変わる、他軸の入力変化が対象軸の半分を超える、
  入力が見送られた区間は、独立軸モデルとして受理しません。
- 励起不足、fit未収束、探索上限到達、時定数がサンプル周期より小さい、
  むだ時間との強い相関、低い検証一致度等も不合格です。
- 検証一致度は `100*(1 - ||y - prediction|| / ||y - mean(y)||)`。
  60%以上を暫定受理基準にしています。「60%の安全性」や誤差上限ではありません。
  高さ制御など他ループも動く閉ループ記録のため、独立軸モデルには残余の相互作用があります。
- 局所の最小二乗標準誤差は診断値です。残差の時間相関、姿勢や機体間の差を含む
  厳密な信頼区間／ロバスト制約値として使いません。
- `beeN_* .csv` に入力・観測・予測・検証区分、`beeN_response.png` に比較図を保存。
  出力には元bagのSHA256、解析コードのSHA256、隣接した記録メタデータも含めます。
  その解析に使ったコード／設定のコピーとライブラリ版も残します。
  解析スクリプトの終了コード0はファイル生成完了を意味し、全モデル受理は意味しません。

```bash
OPENBLAS_NUM_THREADS=1 python3 "$BEE_PACKAGE/test/test_force_response_model.py"
python3 "$BEE_PACKAGE/test/test_force_identification_sim.py"
OPENBLAS_NUM_THREADS=1 python3 "$BEE_PACKAGE/test/test_force_response_bag.py"
```

## 2026-11-14のシミュレーション結果

4台・0.5kg直方体のHoldで、P／PIを各1回、各方向6パルスを記録しました。
最初の4パルスを学習、後の2パルスを未使用検証とし、計24モデルが上記の暫定基準を通過しました。
Pも保持中に積分を無効にして測った結果で、支持移行そのものの試験ではありません。

| モード | 軸 | 時定数 [s] | むだ時間 [s] | モデル95%到達 [s] | 未使用区間fit |
| --- | --- | --- | --- | --- | --- |
| P | normal | 0.215–0.218 | 0.145–0.148 | 0.788–0.801 | 94.1–94.3% |
| P | up | 0.116–0.122 | 0.237–0.243 | 0.592–0.603 | 86.6–87.1% |
| P | side | 0.132–0.134 | 0.214–0.216 | 0.610–0.618 | 92.3–92.6% |
| PI | normal | 0.219–0.224 | 0.137–0.145 | 0.799–0.810 | 93.7–94.1% |
| PI | up | 0.124–0.127 | 0.235–0.236 | 0.608–0.617 | 87.1–87.6% |
| PI | side | 0.135–0.139 | 0.215–0.224 | 0.624–0.631 | 92.4–92.5% |

範囲は4機の最小～最大であり、信頼区間ではありません。元の入力／観測の代表周期は約25–28msです。
PとPIの小さな差を統計的な性能差とは解釈しません。接線応答には小振動が残るため、
一次遅れモデルの95%到達は「その後ずっと±5%に収まる整定時間」ではありません。

機体別の数値と条件は [force_response_sim_box.example.yaml](../config/mujoco/force_response_sim_box.example.yaml) に保存しました。
このファイルは既存launchから読み込まれず、ゲイン／力制約を変更しません。
完全なbag・CSV・図・fit結果・使用ソースは作業記録フォルダの
`logs/2026-11-14-force-identification/experiment-05`, `experiment-06-p`,
`analysis-05-final`, `analysis-06-p` に保存しています。
予備試験の不受理データも分離して残してあります。実機・他形状・大振幅での同定は未実施です。

## MPCへ使う場合

現状の中央は、その周期の目標レンチを力へ配分し、観測が追いつくまで支持の進行を待ちます。
MPCでは、物体の位置・速度・姿勢と力の立ち上がりを予測し、例えば今後1〜2秒の
押付け・支持・加速をまとめて決め、先頭の指令だけを実行して次周期に解き直します。
将来の摩擦・推力制約を考慮して「先に押付けを作り、それから支持や加速を増やす」
計画ができる点が主な違いです。予測時間1〜2秒は説明例であり、今回検証した設定ではありません。

今回のモデルは観測値への応答です。これを物体へ働く真の力としてそのまま使うと、
observerフィルタの遅れを物体力学へ混ぜます。物体の運動まで予測するMPCでは、
力生成の状態と既知の観測フィルタを分けるか、その誤差を含むモデルとして別途検証します。
各Beeの現在の力PI制御を残して、中央だけをMPCにする構成は可能です。

線形化したMPCなら通常はより大きなQPを繰返し解きます。現在の配分QPの数値未収束は、
MPCに置き換えるだけでは解消しません。今回はMPC実装・自動チューニング・実機適用はしていません。
参考: [MathWorksのMPCの説明](https://www.mathworks.com/help/mpc/gs/what-is-mpc.html)。

力補償の比較では `--controller-config` で試験設定を分離できる。追加の補償出力記録と振動解析は [FORCE_FEEDBACK_FILTER.md](FORCE_FEEDBACK_FILTER.md) を参照。
