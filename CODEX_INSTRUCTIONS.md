# CODEX INSTRUCTIONS（Codex への明示手順）

## 目的
- `./run.sh` を成功させる（スモーク基準）
- 生成が「**各ステップで必ず目標へ前進**」するように改善する

## 手順
1. 次を実行：
   ```bash
   chmod +x run.sh
   ./run.sh
   ```
2. 失敗したら：
   - `run.log` と `gs_dump/gen_trace.csv` の**先頭/末尾 各10行**を要約
   - **原因仮説 → 修正方針 → パッチ**（unified diff 形式）を提示
   - **クラウド上の作業領域を直接編集**して `./run.sh` を再実行（自動修正可）
3. 成功したら：
   - `start/min/last/imprAbs/imprRel` を要約
   - 前進が小さい場合は、αグリッドの粒度・`dt` バックオフ・速度床/クランプ等の**改善提案**を継続

## 重要ルール
- `gen_trace.csv` の列名 **`dist_goal`** を変更しない
- 実行は `GSModelTestMain <dump_dir> <tempo>` を想定
- 変更対象は **`GSModel*.{h,cpp}`** を主とし、他は必要最小限に留める

## 望ましいログ出力（`gen_trace.csv`）
- `step,t_sec,dist_goal,delta_goal,alpha,alpha_mode,step_norm,dt,`  
  `splat_id,v_ref,v_min,v_max,used_speed,v_floor,events`

## 参考
- 詳細仕様は `DEV_BRIEF.md`、評価は `EVAL.md` を参照。

_Updated: 2025-10-26_
