"""
patch_simplefoc.py —— 编译前自动修补 SimpleFOC 与新 Arduino STM32 core 的不兼容

问题：
  SimpleFOC 2.4.0 的 stm32_mcu.cpp 调用 enableTimerClock(handle)，
  其中 handle 是 TIM_HandleTypeDef*。
  但 SimpleFOC 自己的 enableTimerClock(TIM_HandleTypeDef*) 定义被包在
  #if defined(__MBED__) 里，在 STM32Duino 下不参与编译。
  于是链接到了 Arduino core 的同名函数 enableTimerClock(TIM_TypeDef*)，
  类型不匹配，编译失败：
      cannot convert 'TIM_HandleTypeDef*' to 'TIM_TypeDef*'

修复：
  enableTimerClock(handle)  ->  enableTimerClock(handle->Instance)
  （与 core 自己的用法一致：timer.c:46  enableTimerClock(htim_base->Instance)）

用 PlatformIO 的 extra_scripts 机制在编译前打补丁，库重装也不会丢。
"""
import os

Import("env")  # noqa: F821  (PlatformIO SCons 环境)

PATCHES = [
    (
        os.path.join("src", "drivers", "hardware_specific", "stm32", "stm32_mcu.cpp"),
        "enableTimerClock(handle);",
        "enableTimerClock(handle->Instance);",
    ),
]


def patch_library():
    libdeps = os.path.join(env.subst("$PROJECT_DIR"), ".pio", "libdeps")
    if not os.path.isdir(libdeps):
        return

    done = 0
    for root, _dirs, _files in os.walk(libdeps):
        if "Simple FOC" not in root:
            continue
        for rel, old, new in PATCHES:
            path = os.path.join(root, rel)
            if not os.path.isfile(path):
                continue
            with open(path, "r", encoding="utf-8", errors="ignore") as f:
                text = f.read()
            if new in text:
                continue                      # 已打过补丁
            if old in text:
                text = text.replace(old, new)
                with open(path, "w", encoding="utf-8") as f:
                    f.write(text)
                print("[patch_simplefoc] patched: %s" % path)
                done += 1

    if done == 0:
        print("[patch_simplefoc] nothing to patch (already OK)")


patch_library()
