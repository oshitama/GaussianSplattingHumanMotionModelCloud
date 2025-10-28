#!/usr/bin/env bash
set -euo pipefail

# ==== 設定（必要なら上書き可） ====
TEMPO="${1:-1.0}"          # 第1引数: 生成テンポ(省略可)
DUMP_DIR="gs_dump"         # ダンプ出力先（実行ファイルの引数1）
BUILD_DIR="build"          # CMakeビルドディレクトリ
CMAKE_BUILD_TYPE="Release" # Release推奨
# ===================================

echo "[run.sh] tempo=$TEMPO  dump=$DUMP_DIR  build=$BUILD_DIR"

# 生成物クリーン（プロジェクト直下に出す設計なので最低限の掃除）
rm -rf "$DUMP_DIR"
mkdir -p "$BUILD_DIR"

# CMake 構成 & ビルド（実行ファイルはプロジェクト直下に出す想定）
cmake -S . -B "$BUILD_DIR" -DCMAKE_BUILD_TYPE="$CMAKE_BUILD_TYPE"
cmake --build "$BUILD_DIR" -j

# 実行ファイルを特定（優先順に探索）
pick_exe() {
  local candidates=("app_headless" "GSModelTestMain" "SimpleHuman" "GSHeadless")
  for c in "${candidates[@]}"; do
    if [[ -x "./$c" ]]; then echo "./$c"; return 0; fi
  done
  # 直下の実行可能ファイルから“最近更新”を拾う（.sh/.pyは除外）
  local f
  f=$(find . -maxdepth 1 -type f -perm -111 ! -name "*.sh" ! -name "*.py" -printf "%T@ %p\n" \
        | sort -nr | awk 'NR==1{print $2}')
  if [[ -n "${f:-}" ]]; then echo "$f"; return 0; fi
  return 1
}

EXE=$(pick_exe) || { echo "[run.sh] ERROR: 実行ファイルが見つかりません"; exit 10; }
echo "[run.sh] exe=$EXE"

# 実行（標準出力を保存）
set +e
"$EXE" "$DUMP_DIR" "$TEMPO" 2>&1 | tee run.log
CODE=${PIPESTATUS[0]}
set -e
echo "[run.sh] exit=$CODE"
if [[ $CODE -ne 0 ]]; then
  echo "[run.sh] ERROR: 実行が異常終了しました"; exit $CODE
fi

# トレースの在処を決定（標準パス → 無ければ探索）
TRACE="$DUMP_DIR/gen_trace.csv"
if [[ ! -f "$TRACE" ]]; then
  echo "[run.sh] gen_trace.csv not at $TRACE. Searching..."
  TRACE=$(find . -type f -name "gen_trace.csv" | head -n 1 || true)
  [[ -z "$TRACE" ]] && { echo "[run.sh] ERROR: gen_trace.csv が見つかりません"; exit 20; }
fi
echo "[run.sh] trace=$TRACE"

# 進捗評価
python3 "$(dirname "$0")/eval.py" "$TRACE"
RC=$?
echo "[run.sh] eval exit=$RC"
exit $RC
