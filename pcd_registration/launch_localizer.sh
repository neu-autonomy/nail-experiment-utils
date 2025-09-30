conda activate nail-utils
LD_PRELOAD="$CONDA_PREFIX/lib/libstdc++.so.6" \
LD_LIBRARY_PATH="$CONDA_PREFIX/lib:${LD_LIBRARY_PATH}" \
"$CONDA_PREFIX/bin/python" /home/alan/nail-experiment-utils/pointcloud_localizer_gui.py
