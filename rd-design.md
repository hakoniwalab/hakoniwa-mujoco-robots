# RD Design (Single-Node RD-Light for MuJoCo Forklift)

## 1. 目的

本ドキュメントは、`hakoniwa-mujoco-robots` における **単一ノード RD-Light** の実装設計を定義する。  
狙いは、`RuntimeStatus`（所有権状態）と `RuntimeContext`（handoffコンテキスト）で、
2つの独立 MuJoCo アセット間の実行権委譲を安全に行うこと。

## 2. スコープ

### 対象

- 同一ノード内で動く2アセット（owner / standby）の切替
- `RuntimeStatus` による ownership 状態遷移
- `RuntimeContext` による MuJoCo コンテキスト受け渡し
- 位置閾値による切替トリガ
- owner->standby / standby->owner の継続実行

### 対象外

- 分散ノード間通信（遅延吸収・再送制御・経路再構成）
- RD制御プレーン（Conductor/Bridge）フル機能の代替
- `d_max` 保証そのもの（RDコア側意味論）

## 3. ノードID・アセット名規約

同一ノードで同種アセットを2重起動するため、Hakoniwaアセット名は一意とする。

- 規約: `<base>-<id>`
- 例: `forklift-1`, `forklift-2`

実装上の設定手段は **CLIオプションではなく環境変数**。

- `HAKO_ASSET_NAME`（例: `forklift-1`）
- `HAKO_RD_LITE_NODE_ID`（例: `1`）
- `HAKO_RD_LITE_PEER_NODE_ID`（例: `2`）
- `HAKO_RD_LITE_INITIAL_OWNER`（`1` or `0`）

注意:

- 同一名アセットの登録は衝突するため禁止
- `RuntimeStatus/RuntimeContext` 上の owner_id と asset-name は同じID体系で対応付ける

## 4. 用語と責務境界

- `RuntimeStatus`
  - ownership truth（制御面）
  - 主要: `status`, `epoch`, `curr_owner_node_id`, `next_owner_node_id`, `config_hash`
- `RuntimeContext`
  - handoff payload（データ面）
  - 主要: `epoch`, `owner_id`, `context[]`, `config_hash`

要点:

- `RuntimeStatus` = 誰が owner かの真実
- `RuntimeContext` = handoff 時に引き渡す実データ

## 5. アーキテクチャ（1ノード）

同一EUに対して、独立した2つの MuJoCo アセットを起動する。

- Asset-A: 初期 owner（例: `forklift-1`）
- Asset-B: 初期 standby（例: `forklift-2`）

```text
Asset-A (Owner) ----write----> RuntimeStatus
Asset-A (Owner) ----write----> RuntimeContext
Asset-B (Standby) --read-----> RuntimeStatus/RuntimeContext
Asset-B (new Owner)-write----> RuntimeStatus (activating/stable)
```

## 6. 状態遷移ルール

`RuntimeStatus.status`:

- `OwnerStable`
- `OwnerReleasing`
- `OwnerActivating`

遷移手順:

1. `OwnerStable(curr=A)`
2. Aが切替条件成立で `epoch++`
3. Aが `RuntimeContext(epoch, owner=A, payload)` を書く
4. Aが `OwnerReleasing(curr=A, next=B, epoch)` を書く
5. Bが `status=OwnerReleasing && next=B && epoch一致` を検知
6. Bが context 復元成功後 `OwnerActivating(curr=B, epoch)` を書く
7. Bが `OwnerStable(curr=B, epoch)` を書く
8. Aは `curr!=A` を観測して standby 化

## 7. 一貫性条件

- 遷移開始（`OwnerReleasing`）は **現在ownerのみ** 実施
- 復元は `RuntimeStatus.epoch == RuntimeContext.epoch` のときのみ実施
- 受け側起動条件:
  - `status == OwnerReleasing`
  - `next_owner_node_id == self`
  - `curr_owner_node_id != self`
  - `epoch` 一致

## 8. コンテキスト仕様

`RuntimeContext.context[]` には `HakoniwaMujocoContext` のシリアライズバイト列を格納する。

現行実装:

- 保存: `save_forklift_state_with_control()`
- 復元: `restore_forklift_state()`
- 送信サイズ上限: `HAKO_RD_LITE_MAX_CONTEXT_BYTES`（既定4096）
- 上限超過時: fail fast（release拒否）

### 8.1 PDU定義

本リポジトリ設定に次の2チャネルを追加済み:

- `RuntimeStatus` (`hako_msgs/ExecutionUnitRuntimeStatus`)
- `RuntimeContext` (`hako_msgs/ExecutionUnitRuntimeContext`)

### 8.2 RuntimeContext `pdu_size`

`ExecutionUnitRuntimeContext` の型ベースサイズを `16` bytes とし、PDU総サイズは:

- `pdu_size(RuntimeContext) = 24 (MetaData) + 16 (base) + MAX_CONTEXT_BYTES`

現行設定:

- `MAX_CONTEXT_BYTES = 4096`
- `pdu_size = 4136`

## 9. 切替トリガ

位置ベース切替:

- owner(A) 側: `pos_x >= (release_x - tolerance)` で A->B
- owner(B) 側: `pos_x <= (home_x + tolerance)` で B->A

実行時パラメータ:

- `HAKO_RD_LITE_RELEASE_X`
- `HAKO_RD_LITE_HOME_X`
- `GOAL_TOLERANCE`

実デモでは「前方1m境界」を handoff point として利用。

## 10. 実装済みの安定化機能

### 10.1 ownership チャタリング抑制

- `HAKO_RD_LITE_SWITCH_TIMEOUT_SEC`（既定 2.0s）
- release/activate直後は再releaseを抑制

### 10.2 RuntimeStatus 読み出し一時失敗耐性

- `runtime_status` unreadable時は致命失敗にせず、
  「前回owner状態（または初期owner）」で継続

### 10.3 standby 表示/物理/干渉制御

- standby表示: 半透明 + tint（視覚的識別）
- standby物理: ON/OFF切替（既定ON）
- standby干渉: 他剛体とは非干渉、地面接触は維持
- 追加オプション:
  - `HAKO_RD_LITE_STANDBY_ALPHA`
  - `HAKO_RD_LITE_STANDBY_TINT`
  - `HAKO_RD_LITE_STANDBY_PHYSICS`
  - `HAKO_RD_LITE_STANDBY_WELD`（既定OFF）
  - `HAKO_RD_LITE_STANDBY_ZERO_DYNAMICS`（既定ON）

### 10.4 owner->standby 遷移時の同期

- ownerを失った側は、handoff保存直後のstateをローカル再適用
- standby時は `ctrl/act/qvel/qacc` を抑制してドリフト低減

## 11. 失敗時動作（現行）

- 復元失敗:
  - standby側は owner化しない（`OwnerActivating` 未書込み）
- epoch不一致:
  - 待機継続（古いcontext適用を拒否）
- `RuntimeStatus` unreadable:
  - 前回状態で継続

## 12. 未実装 / TODO

- `RuntimeContext` ペイロード専用ヘッダ（`magic`, `format`, `checksum`, `mujoco_version`）
- `OwnerReleasing` タイムアウト時の明示ロールバック（`OwnerStable(curr=self)`）
- commit-point 駆動との厳密連携
- 分散ノード版への昇格（`hakoniwa-rd-core` 連携）

## 13. 成果判定

- 同一ノードで A->B->A を複数回切替できる
- split-brain（同時owner）が発生しない
- handoff後も運動継続（位置/速度/phase）が破綻しない
- ログで `epoch` の単調増加と ownership 遷移が確認できる

---

本設計は **RDフル実装の代替ではなく前提実装（RD-Light）**。  
将来は `hakoniwa-rd-core` の commit-point 駆動と接続し、分散版へ拡張する。
