"""加载用当前 SuperDex 头文件和 Python 环境构建的抓握扩展。"""

from functools import lru_cache
from importlib.machinery import EXTENSION_SUFFIXES
from importlib.util import module_from_spec, spec_from_file_location
from pathlib import Path


@lru_cache(maxsize=1)
def load_native():
    import superdex.physics as sdp

    build = Path(__file__).with_name("native") / "build"
    for suffix in EXTENSION_SUFFIXES:
        path = build / ("_ve33_native" + suffix)
        if path.is_file():
            spec = spec_from_file_location("_ve33_native", path)
            module = module_from_spec(spec)
            spec.loader.exec_module(module)
            if getattr(module, "runtime_api", None) != 6:
                raise RuntimeError("本地抓握 C++ 扩展版本过旧，请按 G4 README 重新编译")
            if module.double_precision != sdp.USE_DOUBLE_PRECISION:
                raise RuntimeError("C++ 抓握扩展与当前 SuperDex 的浮点精度不同，请重新编译")
            return module
    raise RuntimeError("缺少 C++ 抓握扩展，请按 G4 README 中的本地抓握步骤编译")
