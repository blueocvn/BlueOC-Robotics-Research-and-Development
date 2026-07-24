# Vendored — feetech_ros2_driver

This package is a **vendored copy** of an upstream third-party driver, committed
into this repo as source (its own `.git` was stripped) so the handover is
self-contained and preserves local edits.

- **Upstream:** https://github.com/JafarAbdi/feetech_ros2_driver
- **Vendored at commit:** `18aed7fb26d3e2b4c0b47762f39d8698b7032422`
  ("Bump actions/setup-python from 6 to 7 (#35)")
- **License:** see `LICENSE` (unchanged from upstream)

## Local modifications (this fork)

Two files carry local edits on top of the upstream commit above — do **not**
overwrite them by blindly re-pulling upstream:

| File | Change |
|------|--------|
| `include/feetech_ros2_driver/feetech_ros2_driver.hpp` | +3 lines |
| `src/feetech_ros2_driver.cpp` | +9 / −1 lines |

This is the `ros2_control` hardware interface used to drive the real SO-ARM 101
follower. To update from upstream, diff carefully and re-apply these edits rather
than replacing the files wholesale.
