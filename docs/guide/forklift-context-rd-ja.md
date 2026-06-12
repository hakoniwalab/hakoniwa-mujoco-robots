# Forklift Context Save/Restore と RD-light

この文書は、トップ README から分離した forklift の advanced note です。

forklift sample には MuJoCo context save/restore が含まれます。長時間実験の継続、中断/再開、障害復旧、RD-light handoff 実験の前提機能として位置付けています。

## Context Save/Restore

狙い:

- 長時間実験の継続。
- 中断/再開の運用性向上。
- 障害復旧の支援。
- 将来の RD ownership handoff に向けた前提機能。

保存対象:

- フォークリフトサブツリーの物理状態。全体世界ではありません。
- 制御状態: `phase`, `target_v`, `target_yaw`, `target_lift`, `step`。
- フォークリフト actuator の `ctrl[]`。
- `mjData.act`。
- lift / drive 制御の PID 内部状態。

保存対象外:

- 荷物、棚、その他外部オブジェクトの状態。
- Python controller など外部プロセスの内部状態。

復元は同一モデル XML と同一 MuJoCo version を前提にします。現在使う MuJoCo version は [`MUJOCO_VERSION.txt`](../../MUJOCO_VERSION.txt) で管理します。

## State Format

現在の state file format は `v8` です。

実装は互換性のため古い format も読みますが、新しい evidence は現行 format で取得してください。

`v8` では、フォークリフトサブツリーの動力学に加えて actuator / PID 内部状態も保存境界に含めています。fork/lift 文脈が不足した復元で差分が出たため、保存境界を広げています。

## 環境変数

- `HAKO_FORKLIFT_STATE_FILE`: state file path。
- `HAKO_FORKLIFT_STATE_AUTOSAVE_STEPS`: autosave 間隔。
- `HAKO_FORKLIFT_MOTION_GAIN`: forklift motion gain。
- `HAKO_FORKLIFT_TRACE_FILE`: trace CSV path。既定は `./logs/forklift-unit-trace.csv`。
- `HAKO_FORKLIFT_TRACE_EVERY_STEPS`: trace sampling interval。既定は `10`。
- `HAKO_CONTROLLER_MODE`: `asset` または `external`。`control.bash` は `asset` が既定。
- `HAKO_CONTROLLER_ASSET_NAME`: controller asset 名。
- `HAKO_CONTROLLER_DELTA_USEC`: controller tick period。

例:

```bash
HAKO_FORKLIFT_STATE_FILE=./tmp/forklift-it.state \
HAKO_FORKLIFT_STATE_AUTOSAVE_STEPS=1000 \
./src/cmake-build/main_for_sample/forklift/forklift_unit_sim
```

## ログ

- `logs/forklift-unit-run.log`: C++ 実行ログ。
- `logs/control-run.log`: Python controller ログ。
- `logs/forklift-unit-recovery.log`: `START`, `AUTOSAVE`, `END` を含む監査ログ。
- `logs/forklift-unit-trace.csv`: 客観評価用 trace。

成功判定の目安:

- `START restored=yes`
- `Resume control phase=2`
- 復帰後の `AUTOSAVE` でも復元 phase が維持される。

## 連続性チェック

trace CSV から plot を生成します。

```bash
python -m python.plot_forklift_continuity \
  --csv logs/forklift-unit-trace.csv \
  --output logs/forklift-unit-continuity.png \
  --window-sec 8
```

plot では `pos_x`, `body_vx`, `yaw`, `lift_z`, `phase` について、停止前後の session を重ねて確認できます。

実運用での合格目安:

- `logs/forklift-unit-recovery.log` に `START restored=yes` がある。
- restart 後の数百 ms で、軌跡が停止前カーブへ再収束する。
- `phase` の連続性が resume 後の autosave まで維持される。

Phase1 evidence workflow で使った strict acceptance:

- `mean(|delta body_vx|) <= 1e-3`
- `max(|delta body_vx|) <= 1e-2`
- `max(|delta pos_x|) <= 1e-3`
- `phase` continuity: identical
- `max(|delta lift_z|) <= 1e-4`

## 復旧確認の実測事実

`forklift_unit` restart test で確認済み:

- 実施日: 2026-02-23
- MuJoCo: evidence 取得時点では v3.5.0
- State format: `v8`

現在このリポジトリが使う MuJoCo version は [`MUJOCO_VERSION.txt`](../../MUJOCO_VERSION.txt) で別途管理します。

観測内容:

- `logs/forklift-unit-recovery.log`: `START restored=yes ... step=4000 ... target_v=0.700000 ... target_lift=0.171200 ...`
- `logs/forklift-unit-run.log`: `Resume control phase=1 target(v,yaw,lift)=(0.7, -0, 0.1712) step=4000`
- 共通 step `4010..4860` の数値比較では、当時の evidence run で速度と lift target の recorded metric がゼロ差でした。

Evidence package:

- `evidence/official-resume-2026-02-23-v8/`

## 結合テスト

3 つの terminal を使います。

1. Simulator:

```bash
bash forklift-unit.bash
```

2. Controller:

```bash
FORWARD_GOAL_X=5.0 HOME_GOAL_X=0.0 GOAL_TOLERANCE=0.03 bash control.bash
```

3. Start trigger:

```bash
hako-cmd start
```

Resume test:

1. simulator を `Ctrl+C` で停止。
2. 同じ command で simulator を再起動。
3. 同じ引数で controller を再実行。
4. logs で `restored=yes` と期待する `phase` を確認。

## Evidence Workflow

推奨手順:

1. restart なしの baseline を実行し、plot 生成後に artifacts を退避。
2. stop/restart resume test を実行し、plot 生成後に artifacts を退避。
3. 2 つの evidence folder を比較。

logs と plots の退避:

```bash
bash evidence/move-logs-to-evidence.bash phase1-baseline-01
bash evidence/move-logs-to-evidence.bash phase1-resume-01
```

各 evidence folder には、logs、trace CSV、continuity plots、`meta.txt` を保存します。

## RD-light Handoff

RD-light は forklift assets 向けの advanced single-node ownership handoff demo です。RD-full ではありません。

このリポジトリは RD に必要な data-plane physics execution base を提供します。

- MuJoCo による高忠実度実行。
- PDU I/O。
- 連続性の前提としての context save/restore。

このリポジトリの scope 外:

- RD-full control-plane semantics。
- global commit-point finalization。
- distributed epoch guarantees。
- `d_max` guarantees と drift repair。
- bridge rewiring completion confirmation。

Entry points:

- `forklift-1.bash`: initial owner。
- `forklift-2.bash`: initial standby。

より詳細な RD transition notes は [`rd-design.md`](../../rd-design.md) にあります。
