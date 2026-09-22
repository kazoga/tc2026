# sourceして使用する。システムのQtやaptデータベースは変更しない。
if [ -z "${1:-}" ]; then
  echo '使用法: source activate_ui_runtime.bash <展開先runtime>' >&2
  return 1
fi
ui_runtime_root="$(realpath "$1")"
if [ ! -f "$ui_runtime_root/qt_config.rcc" ]; then
  echo 'Qt資源設定が未生成です。prepare_ui_runtime.pyを実行してください' >&2
  unset ui_runtime_root
  return 1
fi
export PYTHONPATH="$ui_runtime_root/usr/lib/python3/dist-packages${PYTHONPATH:+:$PYTHONPATH}"
export LD_LIBRARY_PATH="$ui_runtime_root/usr/lib/x86_64-linux-gnu${LD_LIBRARY_PATH:+:$LD_LIBRARY_PATH}"
export QTWEBENGINEPROCESS_PATH="$ui_runtime_root/usr/lib/x86_64-linux-gnu/qt5/libexec/QtWebEngineProcess"
export ROBOT_CONSOLE_QT_RESOURCE="$ui_runtime_root/qt_config.rcc"
export PATH="$ui_runtime_root/usr/bin:$PATH"
unset ui_runtime_root
