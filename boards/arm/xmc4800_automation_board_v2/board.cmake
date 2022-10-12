# SPDX-License-Identifier: Apache-2.0
#
# Copyright (c) 2022 Andriy Gelman <andriy.gelman@gmail.com>

board_set_debugger_ifnset(jlink)
board_set_flasher_ifnset(jlink)

board_runner_args(jlink "--device=XMC4800-2048")

include(${ZEPHYR_BASE}/boards/common/pyocd.board.cmake)
include(${ZEPHYR_BASE}/boards/common/jlink.board.cmake)
include(${ZEPHYR_BASE}/boards/common/openocd.board.cmake)
