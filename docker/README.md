# Docker for hakoniwa-mujoco-robots

Ubuntu 24.04 ベースの開発/実行環境です。

## イメージ作成

```bash
bash docker/create-image.bash
```

`hakoniwa-core-pro` は Docker ビルド時に GitHub から submodule 含めて clone し、ビルド/インストールします。
必要なら以下で取得元とブランチ(タグ)を指定できます。

```bash
HAKO_CORE_PRO_REF=main bash docker/create-image.bash
```

## コンテナ起動

```bash
bash docker/run.bash
```

GUI起動制御:

- `HAKO_DOCKER_GUI=auto` (デフォルト):  
  macOS は XQuartz 経由GUIを試行、Linux は `DISPLAY` と `/tmp/.X11-unix` が使える場合のみGUI、それ以外は headless(Xvfb)
- `HAKO_DOCKER_GUI=on`: GUIを強制（LinuxでX11が使えない場合はエラー）
- `HAKO_DOCKER_GUI=off`: headless(Xvfb)を強制

例:

```bash
HAKO_DOCKER_GUI=off bash docker/run.bash
```

macOSでGUI表示する場合:

```bash
open -a XQuartz
xhost +localhost
HAKO_DOCKER_GUI=on bash docker/run.bash
```

XQuartz 側で `Preferences > Security > Allow connections from network clients` を有効化し、
XQuartz を再起動してから実行してください。

## ビルド

コンテナ内で実行:

```bash
bash build.bash
```

## 実行（自動操縦）

本リポジトリは C++ シミュレータと Python コントローラを別ターミナルで起動します。

1. ターミナル1（コンテナ内）:

```bash
./src/cmake-build/main_for_sample/forklift/forklift_sim
```

2. ターミナル2（ホスト側で別シェルを開いて）:

```bash
bash docker/attach.bash
```

コンテナに入ったら:

```bash
mkdir -p images
python3 -m python.forklift_api_control config/safety-forklift-pdu-compact.json config/monitor_camera_config.json
```
