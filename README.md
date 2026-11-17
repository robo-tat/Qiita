# Qiita

Qiita記事の原稿と、記事に対応する静的シミュレーターを管理します。

## 構成

```text
articles/
└── 01-control-intro/
    ├── README.md
    └── images/belt.png
docs/
├── index.html
└── control-intro/
    ├── index.html
    ├── style.css
    └── script.js
AGENTS.md
README.md
LICENSE
tests/model.test.js
```

`control-intro` はベル直シミュレーターです。記事本文は未作成です。`tests/` は計算処理の検証用です。ログ・バックアップ・履歴ファイルは保存しません。

## ローカル確認

リポジトリ直下で実行します。

```sh
python3 -m http.server 8000 --directory docs
```

ブラウザで `http://localhost:8000/` を開きます。ビルド・依存パッケージのインストールは不要です。通常のブラウザでは `docs/control-intro/index.html` の直接起動も可能です。

## GitHub Pages

リポジトリの **Settings → Pages** で、**Deploy from a branch → main → /docs → Save** を選択します。デプロイ完了後のURLは以下です。

- 一覧：`https://robo-tat.github.io/Qiita/`
- ベル直：`https://robo-tat.github.io/Qiita/control-intro/`

`docs` は公開元フォルダー名であり、公開URLには入りません。

## 検証・更新

```sh
node --test tests/model.test.js
```

同ゲインの一致、収束、飽和・端点処理、入力検証、解析解との一致を確認します。CSS・JavaScript変更時はHTML内の `?v=` を、変更後ファイルのSHA-256先頭12桁に更新してください。

## モデルの範囲

ベル直はベルトで加速台が駆動されている区間を、質量と粘性抵抗で近似します。切り離し、衝突、ベルト弾性、モータの電気回路は含みません。端点処理は数値上の制限であり、実機の性能評価には使えません。
