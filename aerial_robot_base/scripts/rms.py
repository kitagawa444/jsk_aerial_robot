#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
from std_msgs.msg import Empty, Int8, UInt16
from aerial_robot_msgs.msg import PoseControlPid

import sys, select, termios, tty, math, time
import os

# === 追加: 可視化用 ===
import matplotlib
# 画面が無い環境でも保存できるように（Xがあるなら自動でGUIに切り替わることもあります）
matplotlib.use('Agg')
import matplotlib.pyplot as plt

msg = """
s: start to subscribe the topic regarding to control errors, and start to calculate the RMS
h: stop the calculate and output the RMS results (and save violin+box plot).
"""

# 生データ（各次元の誤差時系列）を溜める: x,y,z, roll,pitch,yaw
err_samples = [[] for _ in range(6)]

def cb(data):
    global start_flag, pose_cnt, pose_squared_errors_sum, err_samples
    if start_flag:
        pose_cnt += 1

        vals = [
            data.x.err_p,
            data.y.err_p,
            data.z.err_p,
            data.roll.err_p,
            data.pitch.err_p,
            data.yaw.err_p,
        ]

        # RMS 用の二乗和
        for i in range(6):
            pose_squared_errors_sum[i] += vals[i] * vals[i]

        # 可視化用にサンプルを保存（符号付きのまま）
        for i in range(6):
            err_samples[i].append(vals[i])

def getKey():
    tty.setraw(sys.stdin.fileno())
    select.select([sys.stdin], [], [], 0)
    key = sys.stdin.read(1)
    termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
    return key

def plot_violin_box(err_samples, rms, save_path):
    """
    6要素の誤差分布を描画（箱ひげ+バイオリン重ね）。
      - 0,1,2: [m] -> 左軸 (ax_m)
      - 3,4,5: [rad] -> 右軸 (ax_r)
      - バイオリンは箱ひげの上に重ねて描画（alpha↑, width↑）
      - 色: (x,roll)=青, (y,pitch)=緑, (z,yaw)=橙
      - 各カテゴリの真上に RMSE 注釈
    """
    import matplotlib
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt

    labels = ['pos_x','pos_y','pos_z','roll','pitch','yaw']
    # 色（x/roll, y/pitch, z/yawを同色）
    COL_X = '#1f77b4'  # blue
    COL_Y = '#2ca02c'  # green
    COL_Z = '#ff7f0e'  # orange
    pair_color = {0: COL_X, 3: COL_X, 1: COL_Y, 4: COL_Y, 2: COL_Z, 5: COL_Z}

    pos_m  = [1,2,3]    # m:  x,y,z
    pos_rd = [4,5,6]    # rad: roll,pitch,yaw

    fig, ax_m = plt.subplots(figsize=(10, 5.5), dpi=150)  # 左軸（m）
    ax_r = ax_m.twinx()                                   # 右軸（rad）
    ax_r.patch.set_alpha(0.0)

    # 描画ヘルパ（箱ひげ→バイオリンの順＝バイオリンを上に）
    def draw_one(ax, data_1d, x_pos, color):
        # --- 先に箱ひげ（下層） ---
        bp = ax.boxplot([data_1d], positions=[x_pos], widths=0.42, vert=True,
                        patch_artist=True, showfliers=False, zorder=1)
        for box in bp['boxes']:
            box.set(facecolor='white', edgecolor='#1F1F1F', alpha=0.95, zorder=1)
        for whisker in bp['whiskers']:
            whisker.set(color='#1F1F1F', zorder=1)
        for cap in bp['caps']:
            cap.set(color='#1F1F1F', zorder=1)
        for median in bp['medians']:            # 中央線は読みやすさのため更に上
            median.set(color='#E45756', linewidth=2.0, zorder=3)

        # --- 後からバイオリン（上層に重ねる） ---
        parts = ax.violinplot(
            [data_1d], positions=[x_pos],
            showmeans=False, showmedians=False, showextrema=False,
            widths=0.8  # ← 太め
        )
        body = parts['bodies'][0]
        body.set_facecolor(color)
        body.set_edgecolor('none')
        body.set_alpha(0.55)   # ← 濃いめ
        body.set_zorder(2)     # ← 箱ひげ(1)の上に配置

    # --- 左軸（m）: 0,1,2 ---
    for idx, xpos in zip([0,1,2], pos_m):
        if len(err_samples[idx]) > 0:
            draw_one(ax_m, err_samples[idx], xpos, pair_color[idx])

    # --- 右軸（rad）: 3,4,5 ---
    # twin軸の重なり順を揃える（右軸が上になり過ぎるのを防ぐ）
    ax_r.set_zorder(ax_m.get_zorder())
    ax_r.patch.set_visible(False)
    for idx, xpos in zip([3,4,5], pos_rd):
        if len(err_samples[idx]) > 0:
            draw_one(ax_r, err_samples[idx], xpos, pair_color[idx])

    # 軸/ラベル
    ax_m.set_xticks([1,2,3,4,5,6])
    ax_m.set_xticklabels(labels)
    ax_m.set_ylim(-0.1, 0.1)   # pos_x, pos_y, pos_z 用（m）
    ax_r.set_ylim(-0.3, 0.3)   # roll, pitch, yaw 用（rad）
    ax_m.set_ylabel('Error [m]')
    ax_r.set_ylabel('Error [rad]')
    ax_m.grid(True, axis='y', linestyle='--', alpha=0.35)
    ax_m.set_xlim(0.5, 6.5)
    ax_m.set_title('Control Error Distribution (Violin on top of Box)')

    # --- RMSE 注釈 ---
    trans_m = ax_m.get_xaxis_transform()   # (x: data, y: axes)
    trans_r = ax_r.get_xaxis_transform()
    for i, xpos in enumerate([1,2,3,4,5,6]):
        unit  = 'm' if i < 3 else 'rad'
        color = pair_color[i]
        txt   = f"RMSE {rms[i]:.3g} {unit}"
        ax    = ax_m if i < 3 else ax_r
        trans = trans_m if i < 3 else trans_r
        ax.text(xpos, 0.98, txt, transform=trans,
                ha='center', va='top',
                color=color, fontsize=9, fontweight='bold',
                bbox=dict(boxstyle='round,pad=0.2', facecolor='white', alpha=0.8, edgecolor='none'),
                zorder=4)  # 注釈は最前面

    plt.tight_layout()
    plt.savefig(save_path, bbox_inches='tight')
    try:
        plt.show(block=False)
    except Exception:
        pass
    plt.close(fig)



if __name__=="__main__":
    settings = termios.tcgetattr(sys.stdin)

    start_flag = False
    pose_cnt = 0
    pose_squared_errors_sum = [0.0] * 6

    rospy.init_node('rms_error')
    rospy.Subscriber("gimbalrotor/debug/pose/pid", PoseControlPid, cb)

    print(msg)
    try:
        while True:
            key = getKey()

            if key == 's':
                rospy.loginfo("start to calculate RMS errors")
                start_flag = True

                # リセット
                pose_cnt = 0
                pose_squared_errors_sum = [0.0] * 6
                err_samples = [[] for _ in range(6)]

            elif key == 'h':
                rospy.loginfo("stop calculation")

                # RMS 計算
                if pose_cnt > 0:
                        rms = [math.sqrt(v / pose_cnt) for v in pose_squared_errors_sum]
                else:
                        rms = [0.0] * 6

                rospy.loginfo("RMS of pos errors: [%f, %f, %f], att errors: [%f, %f, %f]",
                              rms[0], rms[1], rms[2], rms[3], rms[4], rms[5])

                # 画像保存
                ts = time.strftime("%Y%m%d-%H%M%S")
                out_dir = rospy.get_param('~plot_dir', os.getcwd())
                if not os.path.isdir(out_dir):
                        try: os.makedirs(out_dir)
                        except Exception: pass
                save_path = os.path.join(out_dir, "error_violin_box_%s.png" % ts)

                try:
                        if any(len(d)>0 for d in err_samples):
                                plot_violin_box(err_samples, rms, save_path)   # ← rms を渡す
                                rospy.loginfo("Saved violin+box plot: %s", save_path)
                        else:
                                rospy.logwarn("No samples collected. Plot is skipped.")
                except Exception as e:
                        rospy.logerr("Plot failed: %s", repr(e))

                # 停止＆リセット
                start_flag = False
                pose_cnt = 0
                pose_squared_errors_sum = [0.0] * 6
                err_samples = [[] for _ in range(6)]

            else:
                if key == '\x03':  # Ctrl-C
                    break

            rospy.sleep(0.001)

    except Exception as e:
        print(e)
        print(repr(e))

    finally:
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
