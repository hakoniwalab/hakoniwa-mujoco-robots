# RD Design (Single-Node RD-lite for MuJoCo Forklift)

## 1. 目的

本ドキュメントは、`hakoniwa-mujoco-robots` における **1ノード内 Runtime Delegation 相当**（RD-lite）の設計を定義する。  
狙いは、`RuntimeStatus`（所有権状態）と `RuntimeContext`（コンテキスト本体）を使って、2つのアセット間で実行権を安全に切り替えること。

## 2. スコープ

### 対象

- 同一ノード内で動く2アセット（`owner` / `standby`）の切替
- `RuntimeStatus` による ownership 状態遷移
- `RuntimeContext` による MuJoCo コンテキスト受け渡し
- フォークリフト位置などの条件に基づく切替トリガ

### 対象外

- 分散ノード間の通信遅延吸収、再送制御
- RD制御系（Conductor/Bridge）フル機能の代替
- `d_max` 保証そのもの（これはRDコア側意味論）

## 2.1 ノードIDとアセット名規約

同一ノードで同種アセットを2重起動するため、Hakoniwaアセット名は一意にする。

- 規約: `<既存の名前>-<ID>`
- 例: `forklift-1`, `forklift-2`
- 起動オプション:
  - `--node-id=<ID>`
  - `--asset-name=<base>-<ID>`（未指定時は自動生成）

注意:

- 同一名アセットは登録衝突するため禁止
- `RuntimeStatus` / `RuntimeContext` の `owner_id` と `asset-name` は同じID体系で対応付ける

## 3. 用語と責務境界

- `RuntimeStatus`
  - 役割: 「誰がOwnerか」「遷移中か」を示す制御情報
  - 主フィールド: `status`, `epoch`, `curr_owner_node_id`, `next_owner_node_id`
- `RuntimeContext`
  - 役割: 引き継ぐ実データ（MuJoCo state + 制御状態）を運ぶ
  - 主フィールド: `epoch`, `owner_id`, `context[]`

要点:

- `RuntimeStatus` = ownership truth（制御面）
- `RuntimeContext` = handoff payload（データ面）

## 4. アーキテクチャ（1ノード）

同一ノード上で、同一EUに対して2アセットを起動する。

- Asset-A: 初期Owner（例: `forklift-1`）
- Asset-B: standby（例: `forklift-2`）

```
Asset-A (Owner) ----write----> RuntimeStatus
Asset-A (Owner) ----write----> RuntimeContext
Asset-B (Standby) --read-----> RuntimeStatus/RuntimeContext
Asset-B (new Owner)-write----> RuntimeStatus (activate/stable)
```

## 5. 状態遷移ルール

`RuntimeStatus.status` は以下を使う。

- `OwnerStable`
- `OwnerReleasing`
- `OwnerActivating`

遷移は次の順序で固定する。

1. `OwnerStable(curr=A)`  
2. Aが切替条件成立で `epoch++`
3. Aが `RuntimeContext(epoch, owner=A, payload)` を書く
4. Aが `OwnerReleasing(curr=A, next=B, epoch)` を書く
5. Bが `status=OwnerReleasing && next=B && epoch一致` を検知
6. Bが context復元成功後 `OwnerActivating(curr=B, epoch)` を書く
7. Bが `OwnerStable(curr=B, epoch)` を書く
8. Aは `curr!=A` を確認して standby化

## 6. 一貫性条件（必須）

- 単一書き手ルール:
  - 遷移開始（`OwnerReleasing`）は **現在Ownerのみ** が書ける
- `epoch` 整合:
  - 復元は `RuntimeStatus.epoch == RuntimeContext.epoch` のときのみ実施
- 受け側起動条件:
  - `status == OwnerReleasing`
  - `next_owner_node_id == self`
  - `epoch` 一致

1つでも満たさなければ復元しない。

## 7. コンテキスト仕様

`RuntimeContext.context[]` には既存の `HakoniwaMujocoContext` バイト列を格納する。  
推奨ヘッダ:

- `magic`
- `format_version`
- `payload_len`
- `checksum`
- `mujoco_version`（`MUJOCO_VERSION.txt`）

復元時は以下をチェックし、不一致時は拒否する。

- `format_version`
- `payload_len/checksum`
- `mujoco_version`
- `config_hash`（利用可能なら）

### 7.1 PDU定義ファイル更新方針

本リポジトリ側の `pdudef/pdutypes` に、次の2チャネルを追加する。

- `RuntimeStatus` (`hako_msgs/ExecutionUnitRuntimeStatus`)
- `RuntimeContext` (`hako_msgs/ExecutionUnitRuntimeContext`)

設計ルール:

- `RuntimeStatus` は固定長に近い制御情報として扱う
- `RuntimeContext` は可変長 `context[]` を使うため、運用上は「最大長を決めた固定領域」として `pdu_size` を確保する

### 7.2 RuntimeContext の pdu_size 設計

`ExecutionUnitRuntimeContext` の型ベースサイズは `16` bytes（生成定義の値）を使用する。  
PDU総サイズは次で決める。

- `pdu_size(RuntimeContext) = 24 (MetaData) + 16 (base) + MAX_CONTEXT_BYTES`

ここで:

- `24` はHakoniwa PDUメタデータ固定長
- `16` は `hako_msgs/ExecutionUnitRuntimeContext` のベースサイズ
- `MAX_CONTEXT_BYTES` は今回の `HakoniwaMujocoContext` 最大シリアライズ長（将来拡張分を含む安全マージン込み）

実装注意:

- `MAX_CONTEXT_BYTES` を超える保存データは送信拒否（fail fast）
- 受信側も `payload_len <= MAX_CONTEXT_BYTES` を検証してから復元
- `pdudef` の値は `pdu_size.py` のベースサイズ変更時に再計算する

## 8. 切替トリガ

初期実装は位置ベースで十分。

- 例: `pos_x >= X_release` で A -> B
- 例: `pos_x <= X_home` で B -> A

トリガ判定と遷移実行は、Owner側だけが行う。

## 9. 失敗時動作

- 復元失敗時:
  - standby側は `OwnerActivating` を書かない
  - 現Ownerは `OwnerReleasing` タイムアウトで `OwnerStable(curr=self)` にロールバック
- epoch不一致:
  - 待機継続（古いcontextを適用しない）
- 不正遷移検知:
  - ログ出力して遷移拒否

## 10. 実装ステップ

1. アセット起動オプション追加
   - `--node-id`
   - `--initial-owner`（true/false）
2. `RuntimeStatus`/`RuntimeContext` I/Oをアセット内に実装
3. Owner判定ガードを制御ループに追加
4. 位置トリガで A<->B 切替
5. 結合テストで `epoch` / `phase` / 速度連続性を検証

## 10.1 rd-lite クラス構成

実装は `include/rd_lite/rd_lite.hpp` を基点に、次の責務で分割する。

- `RdLiteConfig`
  - ノードID、初期Owner、切替閾値、`max_context_bytes` を保持
- `IRuntimeStatusStore` / `HakoPduRuntimeStatusStore`
  - `RuntimeStatus` の読書き
- `IRuntimeContextStore` / `HakoPduRuntimeContextStore`
  - `RuntimeContext` の読書き
- `RdLiteOwnershipFSM`
  - 位置閾値ベースの切替判定
- `RdLiteCoordinator`
  - release/activate 手順を実行
  - `epoch` 整合チェック
  - save/restore コールバック境界

## 11. 成果判定

- 同一ノードで A->B->A を複数回切替できる
- split-brain（同時Owner）が発生しない
- 復元後の連続性（位置/速度/加速度/phase）が許容範囲内
- ログで `epoch` の単調増加が確認できる

---

この設計は **RDフル実装の代替ではなく前提実装**。  
将来は `hakoniwa-rd-core` の commit-point 駆動と接続し、分散版に昇格させる。
