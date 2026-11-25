# 力補償フィルタの比較と振動の確認

各Beeの力PI出力LPFは、現在`controller/reference_tracking/force_filter_tau`で設定する。
設定ファイルは`config/controller/force_control.yaml`。旧設定名は`controller/contact_force_filter_tau`だった。
既定の `0.0` は無効。正の値 [s] では、力誤差のP＋I出力だけを一次LPFへ通す。
自機加速度、接触力FF、速度誤差による減衰は、この追加LPFを通らない。
既存の外力観測LPF（`controller/reference_tracking/external_filter_tau: 0.2`）は別に残る。
YAMLは起動時の初期値。実行中は`controller/set_force_control_parameters`サービスで変更でき、
`controller/force_control_parameters`に現在の適用値をlatched配信する。
rosparamだけ変更しても反映されない。使用例・状態保持の仕様は[共通指令追従](CONTROLLER_REFERENCE.md#力補償パラメータのオンライン変更)を参照。

```
自機加速度 + 指令力/m + LPF(力PI) + 速度減衰 → 合算補正・軸・全推力の制限
```

フィルタ状態はworld座標の加速度として保持し、wrenchモード退出/再進入、
停止、保護半径による介入でリセットする。Iは無効指定時にゼロ、出力飽和時に更新を停止する。
フィルタを入れても接触安定性は保証されない。既存観測LPFとの位相遅れ、接触・機体・内側制御を
含めて評価する。固定の推力上限/補正余裕も維持されるが、それ自体は振動抑制ではない。

## 記録

`/beeN/controller/force_feedback`（`ForceFeedback`）を実際の制御更新時だけ出力する。
力診断は各Beeの `/beeN/controller/force_feedback` で記録する。


- `force_error`: Beeが物体へ与える指令−観測 [N]。observerは反対符号なので内部ではtarget＋external。
- `pi_unfiltered`, `pi_filtered`, `integral`, `velocity_damping`: world座標の加速度 [m/s²]。
- `correction_saturated`, `acceleration_limited`, `radius_override`: 制限/保護介入。
- 前二つのPI値と速度減衰は合算制限前。最終出力はtracking_statusのcommanded_acceleration。

受動 `record_grasp_force_response.py` もこのtopicを記録する。
実機の既存controllerバイナリは更新していないため、新topicが存在するとは限らない。

## 比較試験の再利用

`test/run_force_identification.py --controller-config /absolute/controller.yaml` で、
リポジトリの既定設定を編集せずに別設定を比較できる。runnerが設定を試験先へコピーする。
シミュレーション専用の入口であり、専用localhost masterとMuJoCoを検査する。
詳細と基本実行例は [FORCE_IDENTIFICATION.md](FORCE_IDENTIFICATION.md) を参照。

比較ではP/I/LPFを一度に変えず、例えば次の条件を分ける。

| 条件 | 法線/接線P | I | 補償LPF時定数 |
| --- | --- | --- | --- |
| 基準PI | 0.4 | 0.1 | 0 |
| Pのみ | 0.4 | 0 | 0 |
| 小さいP | 0.2 | 0 | 0 |
| P＋LPF | 0.4 | 0 | 0.1 s |
| PI＋LPF | 0.4 | 0.1 | 0.1 s |

0.1秒は比較用の試験値で、実機への推奨値ではない。
`normal_force_kp` はPreloadでも使われるため、小さいP条件では接触取得中のゲインも変わる。
120秒のHold整定後に同じ入力を与えて比較するが、接触状態が完全に同一とは保証しない。

## 既存bagの読み取り

```
source /home/kitagawa/ros/jsk_aerial_robot_ws/devel/setup.bash
OPENBLAS_NUM_THREADS=1 python3 /absolute/bee/script/analyze_force_oscillation.py \
  /absolute/flight.bag --output /absolute/new-analysis \
  --controller-config /absolute/recorded-controller.yaml
```

新しいdiagnostic topicがなくても、指令・tracking_status・任意のMuJoCo足センサを解析できる。
保存設定を使う。bagの隣にcontroller.yamlまたは同定runnerのsummary.jsonがある場合は省略可能。
namespaceはbagから検出し、`--names bee1 bee2 ...` で明示もできる。
ROS masterや指令publishは不要。

- 十分に整定したHold後半、運搬中、同定励起の各方向を別々に集計する。
- headerのsource時刻で整合し、指令には未来値を使わない。旧statusのゼロ/重複時刻を除外する。
  150ms超の欠測を含む区間は不合格。観測は図/スペクトル用に線形補間する。
  10msの解析格子は、約25msの元観測の分解能を上げるものではない。
- 平均、線形トレンド除去後RMS、0.1–0.5/0.5–2/2–8HzのPSD積分、代表ピーク周波数を出力する。
  励起中のPSDは意図した入力成分も含む。これを自励振動の振幅と解釈しない。
- 励起の各ブロックの1秒後〜終了0.1秒前について、力−指令の標準偏差も比較する。
  ブロック内の一定偏りを除いた揺れの指標で、厳密な整定時間/減衰比ではない。
- MuJoCoサイトセンサは足の重量/慣性を含む。DC差から接触力やobserverの偏りを直接断定しない。
- 補償の再構成 `m*(commanded_acceleration-reference.acceleration)-target` は非同期指令の
  時刻整合や制限の影響を含む。新しいdiagnostic topicがある場合は直接記録も比較する。
- スペクトル一致/coherenceや閉ループの成功だけでは、振動原因や実機の安定余裕を特定できない。

`test/test_force_oscillation.py` と `test/test_force_oscillation_bag.py` に、既知正弦波のRMS/周波数、
区間分離、欠測拒否、DC不変性、合算補償再構成/ゼロ観測除外の検査を残した。


## 2026-11-15の比較結果

過去の5bag（P/PI同定、三角柱/直方体/岩運搬）を解析し、0.5kg直方体で新規6条件を各1回比較した。
全6条件でHold後の励起を完了、入力の余裕検査による拒否0。支持移行の4チェックも全条件成功。
励起/Hold中に、合算補正制限・加速度制限・保護半径の介入は記録されなかった。
C++60件、解析の数値/合成bag5件、既存の同定protocol5件も成功。実機書込み/試験は未実施。

| 条件 | 支持方向に残る揺れ [N] | 横方向に残る揺れ [N] | 支持方向のモデル95% [s] | 法線方向のモデル95% [s] |
|---|---:|---:|---:|---:|
| 現行PI | 0.0494 | 0.0321 | 0.620 | 0.806 |
| Iなし | 0.0503 | 0.0302 | 0.598 | 0.793 |
| 法線・接線P半分／Iなし | 0.0258 | 0.0234 | 0.831 | 1.045 |
| P＋LPF 0.1秒 | 0.0405 | 0.0235 | 0.623 | 0.787 |
| 現行PI＋LPF 0.1秒 | 0.0394 | 0.0230 | 0.631 | 0.797 |
| 接線Pだけ半分／Iあり | 0.0306 | 0.0282 | 0.845 | 0.808 |

残る揺れはステップ1秒後〜次の切替0.1秒前のサイト力−指令の標準偏差を、12ステップ×4機で平均。
サイトセンサは足の重量/慣性を含む。応答95%は既存の閉ループ一次遅れ近似の値で、
真の接触力の整定時間ではない。72個の軸別応答モデルは従来のfit基準を通過した。

- Iを切るだけでは過渡振動の大きな改善はなかった。保持中の横方向の観測力誤差平均は
  現行PI約0.0024N、Iなし約0.0082N。これはobserverに対する誤差であり、真の接触力精度ではない。
- 現行PI＋0.1秒LPFは支持方向の尾部揺れを約20%、横方向を約28%減らした。
  最初の行き過ぎは大きく減らず、観測波形では増える場合もある。LPF追加だけで安定保証とはしない。
- 法線/接線Pとも半分にすると支持方向の揺れは約48%減るが、法線応答も遅くなる。
- 法線P0.4・接線P0.2・I0.1・追加LPF0では、支持方向の揺れが約38%減り、法線応答は維持。
  支持応答は約0.62→0.85秒へ遅く、横方向の改善は約12%でLPF条件より小さい。
  特定の設定が全ての方向/指標で優れるという結果ではない。

追加LPF機能は残し、標準設定の法線/接線P0.4・I0.1・追加LPF0を維持した。
各条件1回の名目シミュレーションだけで既定ゲイン/実機設定を置換していない。
追加遅延、接触剛性/減衰、他形状での候補設定の運搬、反復再現性は今後の検証対象。
完全なbag・試験設定・波形・解析ソース・結果は作業記録フォルダの
`logs/2026-11-15-force-feedback-filter/REPORT.md`、`new-*`、`*-verified`、`fit-*`、`support-*`に保存した。
中央QPや摩擦余裕の値は変更していない。


## 到達時間と整定時間を分ける解析

`script/analyze_force_settling.py` は `analyze_force_oscillation.py` のCSVを読み、
ステップ応答の立ち上がりと残留振動を別々に比較する。指令publishやROS masterは不要。

```
OPENBLAS_NUM_THREADS=1 python3 /absolute/bee/script/analyze_force_settling.py \
  /absolute/oscillation-analysis --output /absolute/new-settling-analysis --dwell 2.5
```

- 対象は各軸13ブロックの既存同定プロトコル。`--dwell` は実際のブロック時間と合わせる。
  他形式の実機入力にこの区切りを無条件で適用しない。入力区間を正しく抽出する必要がある。
- 正負のfull-amplitudeステップ（指令変化0.12N以上）を整合し、直前のDCを除去する。
- シミュレーションの足センサの単発スパイクを抑えるため、全信号に同じ50msの因果移動平均を適用。
  この測定処理の遅れは結果に含まれる。元のobserverの周期を超える精度は主張しない。
- 10%/90%の初回到達（50ms以上持続）、最大行き過ぎ、絶対誤差積分を算出する。
- 整定は指令の**変化分**の±10%に入り、その後窓の末尾まで（最低0.3秒）収まること。
  標準2.5秒ブロックでは2.4秒までを評価し、確認できない場合はnullとする。
  nullは発散の判定でも、実測の整定時間を外挿した値でもない。
- 同符号へ揃えた波形の中央値と、各ステップが整定した割合を両方出力する。
  中央値の成功は全ステップの保証ではなく、4機/複数ステップも独立反復の信頼区間ではない。
- 足サイトセンサは足の重量/慣性を含む。observerだけが速くなったことを、真の力の高速化と混同しない。
  最終値の偏りを隠すための事後ゲイン正規化はせず、指令の変化量を基準にする。

`test/test_force_settling.py` は既知の一次応答の到達/整定、持続振動の未整定、
正負ステップの整合、一定センサ偏りへの不変性を検査する。


## 2026-11-15の速度減衰・observer応答比較

追加4条件を0.5kg直方体で各1回比較した。P0.4/I0.1/力PI出力LPF0は共通、基準は前節の同一ソース/バイナリのPI試験を再利用。

| 条件 | 支持の尾部揺れ [N] | 横の尾部揺れ [N] | 支持のサイト90% [s] | 支持のサイト±10%整定 [s] | 支持の観測モデル95% [s] |
|---|---:|---:|---:|---:|---:|
| 基準：減衰3 / LPF0.2 / observer3 | 0.04941 | 0.03207 | 0.19 | 窓内で未確認 | 0.620 |
| 減衰6 / LPF0.2 / observer3 | 0.04908 | 0.03116 | 0.20 | 窓内で未確認 | 0.617 |
| 減衰9 / LPF0.2 / observer3 | 0.03458 | 0.02472 | 0.22 | 窓内で未確認 | 0.636 |
| 減衰6 / LPF0.1 / observer3 | 0.05819 | 0.03670 | 0.24 | 窓内で未確認 | 0.493 |
| 減衰6 / LPF0.1 / observer6 | 0.05182 | 0.04227 | 0.18 | 窓内で未確認 | 0.363 |


ここでLPFは力PI出力ではなく `tracker/external_filter_tau`、減衰は `contact_damping`。
observerは `/beeN/controller/momentum_observer_force_weight`（実験では起動launchで上書き）。

- 法線方向は減衰6・観測LPF0.1・observer6でサイト力の±10%整定が0.83→0.45秒、
  指令変化に対する行き過ぎが約18%減。全24ステップで窓内整定。ただし複数設定を同時に変更しており単独要因へ帰属しない。
- 同じ条件で支持方向の最初の行き過ぎは約29%減ったが、支持の尾部揺れは約5%、横は約32%増。
  支持/横の±10%整定は2.4秒窓で未確認。observerモデルの高速化と物理的な整定の改善を区別する。
- 減衰9のみは支持の揺れを約30%、横を約23%減らしたが、支持応答の高速化は確認できなかった。
- 観測LPFのみ短縮する条件は支持の揺れを約18%増やした。法線/接線を別々に調整する方針を次の検証候補とし、標準設定は維持。

全4試験・支持移行チェック成功、入力拒否0、実行中入力hash変更0、各12モデル受理。
Hold/励起中の補正制限・加速度制限・保護半径の介入率0。各条件1回の名目シミュレーションであり、
実機の無振動や安定余裕を保証しない。C++/標準設定の変更、再ビルド、実機書込み/動作確認なし。
今回の追加実装は受動の整定解析器・2件の数値検査・文書。試験設定/全bag/固定解析ソース/波形は
作業記録 `logs/2026-11-15-force-response-damping/REPORT.md` と各試験/解析ディレクトリに保存した。

2026-11-16以降の同定runnerは`--reference-tracking-config`で下流設定を指定できる。
旧`--controller-config`に下流ゲイン/LPFの旧キーがある場合は、runnerが共通設定へ移して保存する。
