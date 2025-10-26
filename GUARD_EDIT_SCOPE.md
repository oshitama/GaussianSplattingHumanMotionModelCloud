# GUARD EDIT SCOPE（編集許可範囲）

- ✅ 変更 **可**：
  - `GSModel*.{h,cpp}`（学習・生成・ダンプのロジック）
  - `CMakeLists.txt`, `run.sh`, `eval.py`
  - ドキュメント（`*.md`）

- ⚠️ 原則 **非変更**：
  - `HumanBody.*`, `SimpleHuman.*`, `BVH.*`, `vecmath/*`  
    ※ ビルド/実行に必要な最小修正は可（理由を記載）

- ❌ **禁止**：
  - OpenGL/GLUT 関連（今回のヘッドレス構成では不要）

理由：既存ライブラリの安定性維持とデバッグ容易化のため。

_Updated: 2025-10-26_
