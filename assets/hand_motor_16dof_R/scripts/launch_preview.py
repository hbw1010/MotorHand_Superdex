import sys,runpy
from pathlib import Path
root=Path(__file__).resolve().parents[1]
try:
    import numpy,scipy,yaml
except ImportError:
    sys.path.insert(0,'C:\\Users\\current robotics\\Documents\\Codex\\2026-10-13\\error-code-unsupported-country-region-territory\\work')
    import bootstrap
runpy.run_path(str(root/'scripts/viewer_server.py'),run_name='__main__')
