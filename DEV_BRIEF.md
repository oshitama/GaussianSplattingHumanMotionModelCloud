# DEV BRIEF（開発要点 / 仕様 / 設計）

## 目的
- **少量データ**から開始しても破綻なく動く動作生成
- 高次元姿勢空間での **Gaussian Splat**（占有/遷移）の集合として可到達領域をモデル化
- **モードレス**（なるべく単一モデルで全域を扱う）を志向
- MVPでは「**各ステップで必ず目標へ前進**」を保証

## API（外部仕様）
- 学習: `GSModel::Fit(HumanBody body, std::vector<Motion> motions, const TrainOptions& topt)`
- 生成: `GSModel::Generate(const Posture& start, const Posture& goal, const GenerateOptions& gopt)` → `KeyframeMotion`

## 生成アルゴリズム（MVP）
- **αグリッド探索**: α ∈ {0, 0.25, 0.5, 0.75, 1}
  - `v_step = (1-α) * v_model + α * unit_to_goal * speed_target`
  - `speed_target = clamp(v_norm_ref, v_norm_min, v_norm_max)`
  - **速度床** `v_floor_mps` を `max(v_floor_mps, v_norm_min)` で適用
- **フォールバック**:
  - 最良αでも悪化する → `α=1` を再試行
  - なおNG → `dt` を **1/2 バックオフ**（最大2回）
  - 停滞（改善なしが連続） → **数ステップ `α=1` 固定**で強制前進
- **収束判定**:
  - `dist_goal <= eps_goal` または `max_steps` 到達

## ダンプ仕様（検証のため必須）
- `gen_trace.csv`（必須列）  
  `step,t_sec,dist_goal,delta_goal,alpha,alpha_mode,step_norm,dt,`  
  `splat_id,v_ref,v_min,v_max,used_speed,v_floor,events`
- `gen_init.json`（推奨キー）  
  `dt,max_steps,eps_goal,v_floor_mps`

## 非機能・制約（MVP）
- 目標のワールド位置/向きの最終合わせは未実装（将来拡張）
- `vecmath/` は最小限の演算に留める（互換性重視）
- ヘッドレス専用（描画コードは無効）

## 今後の拡張の例
- 相対向きでの探索（開始・目標の向きがサンプルと異なっても探索可能に）
- 足接地制約・予備動作（フットワーク）生成
- スプラット合併/分割の自動化によるスケール性強化

_Updated: 2025-10-26_
