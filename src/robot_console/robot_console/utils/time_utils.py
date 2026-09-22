"""時間関連のユーティリティ。"""

from datetime import datetime, timedelta, timezone

# 内部保持はUTCで統一し、画面表示だけ日本時間へ変換する。
JST = timezone(timedelta(hours=9))


def now() -> datetime:
    """UTC タイムゾーンの現在時刻を返す。"""

    return datetime.now(tz=timezone.utc)


def format_local_time(value: datetime) -> str:
    """日時を日本時間の `HH:MM:SS` 文字列へ変換する.

    Args:
        value (datetime): 変換対象の日時. tzinfo を持たない場合は UTC として扱う.

    Returns:
        str: 日本時間の `HH:MM:SS` 文字列.
    """

    if value.tzinfo is None:
        value = value.replace(tzinfo=timezone.utc)
    return value.astimezone(JST).strftime('%H:%M:%S')
