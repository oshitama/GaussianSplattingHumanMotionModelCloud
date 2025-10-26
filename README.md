# Gaussian Splatting Human Motion (Headless)

このパックは **GitHub を使わず Codex のクラウド上だけで**ビルド → 実行 → 検証まで回すための最小構成です。
- **CMake** でビルドし、実行ファイルと生成物は **プロジェクト直下**に出力します。
- 評価は `gen_trace.csv` の `dist_goal` 列を見て行います。

## 使い方（Codex/ローカル共通）
```bash
chmod +x run.sh
./run.sh            # tempo=1.0（既定）
./run.sh 1.2        # 例: テンポを1.2に
```
- 生成物とダンプ: `./gs_dump/`（`gen_trace.csv`, `gen_init.json`, `keyframes.jsonl` など）
- 実行ログ: `./run.log`

## ディレクトリ構成（抜粋）
- `GSModel*.{h,cpp}` …… ガウシアンスプラット動作モデル本体（編集対象）
- `HumanBody.* / SimpleHuman.* / BVH.*` …… 既存ライブラリ（基本は非編集）
- `vecmath/` …… ベクトル・行列演算ユーティリティ（あなたの環境のものを同梱）
- `motion_rikiya/` …… 小規模 BVH（テストデータ）
- `run.sh` / `eval.py` …… ビルド・実行・評価スクリプト
- `DEV_BRIEF.md` …… 開発者向け要点・仕様・設計
- `EVAL.md` …… 評価基準の詳細
- `CODEX_INSTRUCTIONS.md` …… Codex への具体的な指示
- `GUARD_EDIT_SCOPE.md` …… 編集許可範囲の明示
- `BUG_REPORT.md` …… 失敗時の報告テンプレート

## 前提
- エントリ実行ファイルは `GSModelTestMain`（または自動検出）。引数 `<dump_dir> <tempo>` を受け取り、
  `<dump_dir>/gen_trace.csv`（列名 `dist_goal` 必須）を出力します。
- OpenGL/GLUT 呼び出しは無効化済み（スタブ不要）。
- CMake は生成物を **プロジェクト直下**に出力します。

_Updated: 2025-10-26_
