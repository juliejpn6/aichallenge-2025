# 🏎️ AI Challenge 2025 - カスタム自動運転システム

## 📌 プロジェクト概要

このリポジトリは、**自動運転AIチャレンジ2025**向けにカスタマイズされた自動運転システムです。  
Autoware-Microをベースに、高速走行とカーブ走行を最適化した制御システムを実装しています。

## ✨ 主な特徴

### 1. 時間ベースPure Pursuit制御
- 従来の距離ベースから**時間ベース**のLookahead方式に変更
- 高速走行時の安定性が大幅に向上
- 道路タイプ（直線・緩カーブ・急カーブ）に応じた自動調整

### 2. 自動ラップ管理システム
- ゴールライン通過を自動検出
- 周回ごとに速度を段階的に増加
- CSVファイルの自動切り替え機能

### 3. プログラム初心者にも優しい設計
- すべてのパラメータに日本語コメント付き
- launchファイルで簡単に調整可能
- ビルド不要でパラメータ変更可能

---

## 📁 リポジトリ構成

```
aichallenge-2025/
├── README.md                          # このファイル（プロジェクト概要）
├── docs/                              # ドキュメント
│   ├── PURE_PURSUIT.md               # Pure Pursuit制御の詳細説明
│   ├── GOAL_CHECKER.md               # Goal Line Checkerの詳細説明
│   ├── PARAMETERS.md                 # パラメータ一覧と調整ガイド
│   └── SETUP.md                      # セットアップ手順
│
├── simple_pure_pursuit.cpp            # Pure Pursuit制御ノード（メイン処理）
├── simple_pure_pursuit.hpp            # Pure Pursuitヘッダーファイル
├── goal_line_checker_node.cpp         # ゴールライン判定ノード
├── reference_launch.xml               # メインのlaunchファイル
└── goal_checker_launch.xml            # Goal Checker用launchファイル
```

---

## 🚀 クイックスタート

### 1. 環境要件
- Ubuntu 24.04
- ROS 2（Autoware-Micro対応版）
- 自動運転AIチャレンジ環境

### 2. セットアップ
詳細な手順は [docs/SETUP.md](docs/SETUP.md) を参照してください。

```bash
# このリポジトリをクローン
cd /aichallenge/workspace/src/aichallenge_submit/
git clone https://github.com/juliejpn6/aichallenge-2025.git

# ファイルを配置
# （詳細はSETUP.mdを参照）

# ビルド
cd /aichallenge/workspace
colcon build

# 実行
ros2 launch aichallenge_submit_launch reference.launch.xml
```

### 3. パラメータ調整
ビルド不要で調整できます！

- **速度調整**: `goal_checker_launch.xml` を編集
- **走行調整**: `reference_launch.xml` のPure Pursuitセクションを編集

詳細は [docs/PARAMETERS.md](docs/PARAMETERS.md) を参照してください。

---

## 📖 ドキュメント

| ドキュメント | 内容 |
|-------------|------|
| [docs/PURE_PURSUIT.md](docs/PURE_PURSUIT.md) | Pure Pursuit制御の仕組みと設定方法 |
| [docs/GOAL_CHECKER.md](docs/GOAL_CHECKER.md) | Goal Line Checkerの仕組みと設定方法 |
| [docs/PARAMETERS.md](docs/PARAMETERS.md) | 全パラメータの詳細説明と調整ガイド |
| [docs/SETUP.md](docs/SETUP.md) | セットアップとGithubへのアップロード手順 |

---

## 🎯 推奨される使い方

### 初めての方
1. まず [docs/SETUP.md](docs/SETUP.md) でセットアップ
2. デフォルト設定で走行テスト
3. [docs/PARAMETERS.md](docs/PARAMETERS.md) を見ながら少しずつ調整

### パラメータ調整したい方
1. [docs/PARAMETERS.md](docs/PARAMETERS.md) で調整したい項目を確認
2. launchファイルを編集
3. 再起動して効果を確認（ビルド不要！）

### 仕組みを理解したい方
1. [docs/PURE_PURSUIT.md](docs/PURE_PURSUIT.md) でPure Pursuitの原理を学習
2. [docs/GOAL_CHECKER.md](docs/GOAL_CHECKER.md) で自動管理の仕組みを理解
3. ソースコードを読んでさらに深く理解

---

## 🔧 主な変更点（デフォルトのAutoware-Microから）

### Pure Pursuit制御
- ✅ 時間ベースLookahead方式に変更（`lookahead_gain` → `lookahead_time`）
- ✅ 曲率に応じた道路タイプ自動判定機能
- ✅ 道路タイプ別の速度制限機能
- ✅ ステアリング遅延補償機能
- ✅ コーナー出口での自動Lookahead延長機能

### Goal Line Checker
- ✅ ゴールライン通過の自動検出
- ✅ 周回ごとの自動速度増加
- ✅ CSVファイル自動切り替え
- ✅ launchファイルで速度設定可能

---

## 📊 パラメータ調整の基本

### 速度を変えたい
→ `goal_checker_launch.xml` の以下を編集：
```xml
<arg name="initial_speed" default="28"/>      <!-- 初期速度 (km/h) -->
<arg name="speed_increment" default="2"/>     <!-- 周回ごとの増加幅 (km/h) -->
<arg name="max_speed" default="32"/>          <!-- 最高速度 (km/h) -->
```

### カーブで膨らむ/曲がりすぎる
→ `reference_launch.xml` の以下を編集：
```xml
<!-- 緩やかなカーブの速度制限 -->
<param name="gentle_curve_speed_limit_factor" value="0.4"/>
<!-- 0.3～0.5で調整。小さいほど安定、大きいほど速い -->

<!-- 急カーブの速度制限 -->
<param name="sharp_curve_speed_limit_factor" value="0.0"/>
<!-- 0.0～0.3で調整。急カーブは慎重に！ -->
```

詳細は [docs/PARAMETERS.md](docs/PARAMETERS.md) を参照してください。

---

## 🐛 トラブルシューティング

### Q1: ビルドエラーが出る
→ [docs/SETUP.md](docs/SETUP.md) の依存関係を確認してください

### Q2: パラメータを変更したのに反映されない
→ ビルドは不要です。ノードを再起動してください
```bash
# Ctrl+Cで停止後、再度実行
ros2 launch aichallenge_submit_launch reference.launch.xml
```

### Q3: カーブで曲がれない
→ 速度が速すぎる可能性があります。`speed_limit_factor`を下げてください

### Q4: 低速時に不安定
→ `lookahead_min_distance`を大きくしてください（2.0 → 3.0）

---

## 📝 変更履歴

### v1.0.0 (2025-10-22)
- 初回リリース
- 時間ベースPure Pursuit制御を実装
- Goal Line Checker自動管理機能を実装
- プログラム初心者向けドキュメント整備

---

## 👥 開発者向け情報

### コードの構造
- `simple_pure_pursuit.cpp`: Pure Pursuit制御のメインロジック
- `simple_pure_pursuit.hpp`: クラス定義とメンバ変数
- `goal_line_checker_node.cpp`: ゴール判定と自動パラメータ変更

### 主要な関数
- `onTimer()`: メイン制御ループ（100Hz）
- `calculateCurvature()`: 3点から曲率を計算
- `classifyRoadType()`: 道路タイプを判定
- `adjustFactorForCorneringState()`: ステアリング状態に応じた調整

---

## 🤝 コントリビューション

このプロジェクトは自動運転AIチャレンジ向けのカスタマイズ版です。  
改善提案やバグ報告は Issue または Pull Request でお願いします。

---

## 📄 ライセンス

このプロジェクトは、自動運転AIチャレンジの規約に従います。

---

## 🎓 参考資料

- [自動運転AIチャレンジ公式サイト](https://www.jsae.or.jp/jaaic/)
- [Autoware Documentation](https://autowarefoundation.github.io/autoware-documentation/)
- [Pure Pursuit アルゴリズム解説](https://www.ri.cmu.edu/pub_files/pub3/coulter_r_craig_1992_1/coulter_r_craig_1992_1.pdf)

---

## 📞 お問い合わせ

質問や不明点がありましたら、Issueを作成してください。  
プログラム初心者の方も歓迎です！わかりやすく説明します。

---

**Happy Racing! 🏁**
