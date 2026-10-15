# セットアップ手順 (rc_car_driver / Raspberry Pi)

別の Raspberry Pi でも同じ環境を再現するための手順。上から順に実行する。

対象環境（検証時）:
- Raspberry Pi 4 Model B
- Ubuntu 24.04 LTS / kernel `6.8.0-xxxx-raspi`
- ROS 2 **Jazzy**
- ハードウェア PWM: GPIO12 = PWM0(ch0) = ステアリング, GPIO13 = PWM1(ch1) = スロットル

> Pi 5 へ移植する場合: PWM の sysfs チップ番号が変わる（Pi4=`pwmchip0`, Pi5=`pwmchip2`）。
> ノード/スクリプトの `pwm_chip` パラメータと、確認コマンドのチップ番号を読み替えること。

---

## 1. apt パッケージ

```bash
sudo apt update
sudo apt install -y \
  ros-jazzy-ackermann-msgs \
  python3-pip \
  python3-colcon-common-extensions
```

- `ros-jazzy-ackermann-msgs` … 指令メッセージ型 `AckermannDriveStamped`
- `python3-pip` … pip（既定では未導入）
- `python3-colcon-common-extensions` … `colcon build` 用

確認:
```bash
ros2 interface show ackermann_msgs/msg/AckermannDriveStamped >/dev/null && echo "ackermann_msgs OK"
```

## 2. pip パッケージ（venv は使わない・システム導入）

このマシンでは venv を使わない方針。Ubuntu 24.04 は PEP668 で外部管理のため `--break-system-packages` を付ける。

```bash
sudo pip install --break-system-packages rpi-hardware-pwm
```

確認:
```bash
python3 -c "from rpi_hardware_pwm import HardwarePWM; print('rpi-hardware-pwm OK')"
```

## 3. ハードウェア PWM オーバーレイ（config.txt）

`/boot/firmware/config.txt` の末尾に以下を**1行**追記する。GPIO12/13 を PWM0/PWM1 に割り当てる。

```
dtoverlay=pwm-2chan,pin=12,func=4,pin2=13,func2=4
```

追記コマンド例（重複追記しないよう grep でガード）:
```bash
LINE='dtoverlay=pwm-2chan,pin=12,func=4,pin2=13,func2=4'
grep -qxF "$LINE" /boot/firmware/config.txt || echo "$LINE" | sudo tee -a /boot/firmware/config.txt
```

## 4. 非 root で sysfs PWM を操作するための権限（udev + group）

`rpi-hardware-pwm` は `/sys/class/pwm` を読み書きする。root 実行を避けるため `gpio` グループに権限を与える。

```bash
# gpio グループ作成（既にあれば無視される）+ 自分を追加
sudo groupadd -f gpio
sudo usermod -aG gpio "$USER"

# udev ルール設置: pwm の sysfs を root:gpio 770 にする
sudo tee /etc/udev/rules.d/99-pwm.rules >/dev/null <<'EOF'
SUBSYSTEM=="pwm*", PROGRAM="/bin/sh -c '\
  chown -R root:gpio /sys/class/pwm && chmod -R 770 /sys/class/pwm;\
  chown -R root:gpio /sys/devices/platform/soc/*.pwm/pwm/pwmchip* && chmod -R 770 /sys/devices/platform/soc/*.pwm/pwm/pwmchip*\
'"
EOF
```

> udev を入れない場合は、ノード/スクリプトを `sudo` 実行する必要がある（非推奨）。
> `gpio` グループ追加は再ログイン or 再起動で反映される。

## 5. 再起動

config.txt のオーバーレイとグループ追加を反映するため再起動する。

```bash
sudo reboot
```

## 6. 動作確認（サーボ未接続のまま）

再起動後、PWM チップがエクスポートされ、2チャンネル使えることを確認する。

```bash
ls /sys/class/pwm/pwmchip0          # pwmchip0 が見える
cat /sys/class/pwm/pwmchip0/npwm    # => 2
ls -l /sys/class/pwm/pwmchip0       # グループが gpio・770 になっていること
groups | tr ' ' '\n' | grep -qx gpio && echo "gpio group OK"
```

すべて OK なら、ここで初めてサーボを配線する（→ 7）。

## 7. ワークスペースのビルド

```bash
cd ~/ros2_ws
colcon build --symlink-install
source install/setup.bash
# 常用するなら ~/.bashrc に: source ~/ros2_ws/install/setup.bash
```

---

## 配線（電源を切ってから）

| サーボ線 | 接続先 |
|---|---|
| 信号（橙/白） | **GPIO12 = 物理ピン32** |
| GND（茶/黒） | Pi の GND（例: 物理ピン6）※外部電源使用時も**共通グランド必須** |
| V+（赤） | **外部 5–6V 電源 / BEC**（Pi の 5V ピンからは給電しない＝ブラウンアウト回避） |

スロットル（ESC）は車体入手後に **GPIO13 = 物理ピン33** へ接続。

---

## 動作テスト

### フェーズ0: サーボ単体（ROS 不要）
```bash
python3 ~/ros2_ws/src/rc_car_driver/scripts/servo_test.py --sweep
```
サーボが中央↔両端にスイープすれば疎通 OK。

### フェーズ1: ROS 経由

毎回ワークスペースを source しておく（§7 参照）:
```bash
source /opt/ros/jazzy/setup.bash
source ~/ros2_ws/install/setup.bash
```

**駆動ノードの起動（2通り）**

```bash
# A) パラメータを config/vehicle_driver.yaml から読んで起動（推奨）
ros2 launch rc_car_driver vehicle_driver.launch.py

# B) 既定パラメータで直接起動
ros2 run rc_car_driver vehicle_driver
```
起動すると `Arming: holding neutral for 2.0s` → `vehicle_driver ready, listening on /ackermann_cmd` と出て、
両 ch がニュートラル(1500us)でアーミングされる。

**キーボード teleop（別ターミナルで）**
```bash
ros2 run rc_car_driver key_navigation
```
操作: `←/→`=ステア(左+/右−, 0.1rad刻み), `↑/↓`=速度(±0.2m/s), `Space`=停止(中央), `q`=終了(ニュートラル送信)。
※ key_navigation は矢印キーを cbreak で読むため**実ターミナル(TTY)が必要**。

**トピックへ直接指令（teleop を使わない確認）**
```bash
# 連続 publish でウォッチドッグ(>0.5s で中立復帰)を満たし続ける。
# --once は到達前にディスカバリが間に合わず効かないことがある。
ros2 topic pub --rate 20 /ackermann_cmd ackermann_msgs/msg/AckermannDriveStamped \
  "{drive: {steering_angle: 0.5, speed: 0.0}}"
```

**机上での PWM 確認（サーボの代わりに duty を見る）**
```bash
watch -n0.2 cat /sys/class/pwm/pwmchip0/pwm0/duty_cycle   # ステア(ch0/GPIO12)。ns。中央=1500000, 端=1000000/2000000
watch -n0.2 cat /sys/class/pwm/pwmchip0/pwm1/duty_cycle   # スロットル(ch1/GPIO13)
```

**停止**: ノードを `Ctrl-C` で終了すると `shutdown()` が走り、両 ch を中立に戻してから出力解放(`enable=0`)する。

### フェーズ2: 車体入手後の較正（未実施）
ESC を GPIO13 に接続し `throttle_neutral_us` で完全停止を較正。`max_speed_mps`・後進挙動を確認。
タイヤ直進点で `steering_neutral_us`、最大舵角で `max_steering_angle_deg`(度指定)/レンジ(`steering_min_us`/`steering_max_us`)を `config/vehicle_driver.yaml` に較正。

---

## トラブルシュート

| 症状 | 確認 |
|---|---|
| `/sys/class/pwm/pwmchip0` が無い | config.txt の overlay 行 / 再起動したか |
| `npwm` が 1 や 0 | overlay が `pwm-2chan` か（`pwm` 単独だと1ch） |
| Permission denied | udev ルール / `gpio` グループ / 再ログインしたか / `ls -l /sys/class/pwm/pwmchip0` |
| `No module named rpi_hardware_pwm` | 手順2を `--break-system-packages` 付きで再実行 |
| サーボが動かない | 電源（外部5–6V）/ 共通GND / 信号線がGPIO12か / `servo_test.py --channel` 番号 |
| サーボがブーンと震える/位置が定まらない | 電源容量不足（Piの5Vから取っていないか）/ GND未接続 |
