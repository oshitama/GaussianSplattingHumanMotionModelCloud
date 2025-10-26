# BUG REPORT（失敗時の報告テンプレ）

## 失敗区分
- Build / Run / Eval（いずれか）

## 実行コマンド
```bash
./run.sh <tempo>
```

## ログ抜粋
- `run.log` …… 先頭/末尾 各20行
- `gs_dump/gen_trace.csv` …… 先頭/末尾 各10行

## 要約
- start/min/last/imprAbs/imprRel:

## 原因仮説
（例：速度床が高すぎ／α探索の粒度不足／dt バックオフ不足 など）

## 提案修正
（パッチの概要を記載。実パッチは unified diff で添付）

_Updated: 2025-10-26_
