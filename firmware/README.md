# libautumn.a

Run from the project root:

```bash
make libautumn
```

This builds `bin/autumn.a` from `src/autumn/` and copies it to `firmware/libautumn.a` for template distribution and cold linking.

Until you run this once, `make` still links the library from `bin/autumn.a` when building in library mode (`IS_LIBRARY:=1` in the Makefile).
