# CARLA SIL tests

This directory contains the earlier CARLA Software-in-the-Loop AEB, TTC, navigation and multi-vehicle test scripts. They are retained as development and validation history.

## Result location

CARLA / early SIL CSV results are stored centrally in:

```text
edge-path-project/results/
```

Most newer scripts resolve this directory from `__file__`. Two older historical scripts (`target_speed_test.py` and `target_speed_test_ttc.py`) still use a working-directory-relative `results/...` path.

To keep their output in the central results directory, run legacy CARLA SIL tests from `edge-path-project/`:

```bash
cd edge-path-project
python3 aeb_sil/target_speed_test.py
```

or:

```bash
cd edge-path-project
python3 aeb_sil/target_speed_test_ttc.py
```

Do not run those two scripts from inside `aeb_sil/`, otherwise they can create a second `aeb_sil/results/` directory.
