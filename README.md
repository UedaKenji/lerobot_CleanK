# lerobot-cleank

`lerobot` をベースに自前ロボットへ移植するための派生プロジェクトです。開発環境はこのリポジトリ直下に専用の `.venv` を作り、`pyproject.toml` で指定した git 依存 (`lerobot`) を `uv pip install -e .` で自動取得する方針に統一します。

## 事前準備

- Python 3.10
- [uv](https://github.com/astral-sh/uv) をインストールし、`PATH` に通しておくこと
- Git でこのリポジトリを clone できること

## セットアップ手順（初回のみ）

1. リポジトリを取得:
   ```bash
   git clone <このリポジトリのURL> lerobot_CleanK
   cd lerobot_CleanK
   ```
2. プロジェクト専用の仮想環境を作成:
   ```bash
   uv venv .venv --python 3.10
   ```
3. 仮想環境を有効化:
   ```bash
   source .venv/bin/activate
   ```
4. 依存関係をインストール:
   ```bash
   uv pip install -e .
   ```
   このコマンドで `pyproject.toml` に記載された `lerobot @ git+https://github.com/UedaKenji/lerobot.git@feature/CleanK`
   が自動的に取得・インストールされます。

## 日常的な使い方

- 作業前に必ず `source .venv/bin/activate` でローカル環境を有効化し、そのシェル上でスクリプト実行やテストを行う。
- 依存が更新された場合は再度 `uv pip install -e .` を実行して同期する。
- IDE やエディタの Python インタープリタ設定は `.venv/bin/python` を指すようにする。

## lerobot 本体を直接編集したい場合（任意）

- lerobot リポジトリを別途 clone し、同じ `.venv` 内で `uv pip install -e ../lerobot` を実行すると、CleanK と lerobot を同時に editable で触れます。
- あるいは一時的に `[tool.uv.sources]` を手元の `pyproject.toml` に追加してローカルパスを参照する方法もありますが、チーム共有用のコミットには含めないでください。

## コントリビュート指針

- 追加の Python 依存は必ず `pyproject.toml` に追記し、チーム全体で同じ `uv pip install -e .` 手順で再現できるようにする。
- 必要に応じて `uv pip freeze > requirements.lock` などでバージョンを固定し、再現性を向上させる。
