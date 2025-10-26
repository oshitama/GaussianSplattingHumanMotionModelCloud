# EVAL（評価基準）

## スモーク（初期開発〜自動検証）
- 合格条件：
  - `imprAbs > 0` **または** `last < 0.02`（= 2cm）
- `eval.py` が `gen_trace.csv` の `dist_goal` を読み、要約を標準出力：
  - `start/min/last/imprAbs/imprRel`

## ストリクト（モデルが安定してから）
- 例：`last < 0.02` かつ `imprRel > 0.15` を要求
- 生成パラメタの目安：`max_steps >= 1000`, `dt <= 1/60`, `v_floor_mps ≈ 0.05`

## 失敗時に確認するもの
1. `run.log` の例外・警告
2. `gen_trace.csv` の先頭/末尾（`alpha_mode`, `events` の推移）
3. `start` と `goal` の取り方（遠すぎ/不整合がないか）

## よくある原因
- `dist_goal` の列名変更 → **禁止**（`eval.py` が読めなくなる）
- ダンプ先パスが固定 → `run.sh` 引数の `<dump_dir>` を尊重すること

_Updated: 2025-10-26_
