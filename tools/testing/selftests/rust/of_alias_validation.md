# Local OF alias validation

This validation branch adds compile probes above the standalone
`b4/rust-of-alias` patch. The upstream patch changes only `rust/kernel/of.rs`.
Its base is `8a36f53fd5ec3eff01f1b637d42cbdc3821d0493`.

Run the following from the imager workspace root. The build directories are
separate from the source worktrees.

```sh
export PATH=/home/kwang/.cargo/bin:$PATH

make -C out/linux-of-alias O="$PWD/out/linux-of-alias-riscv-build" ARCH=riscv LLVM=1 allnoconfig
out/linux-of-alias/scripts/config --file out/linux-of-alias-riscv-build/.config -e RUST
make -C out/linux-of-alias O="$PWD/out/linux-of-alias-riscv-build" ARCH=riscv LLVM=1 olddefconfig
make -C out/linux-of-alias O="$PWD/out/linux-of-alias-riscv-build" ARCH=riscv LLVM=1 -j2 rust/kernel.o drivers/of/base.o

make -C out/linux-of-alias O="$PWD/out/linux-of-alias-x86-no-of-build" ARCH=x86_64 LLVM=1 allnoconfig
out/linux-of-alias/scripts/config --file out/linux-of-alias-x86-no-of-build/.config -e 64BIT -e RUST -d OF
make -C out/linux-of-alias O="$PWD/out/linux-of-alias-x86-no-of-build" ARCH=x86_64 LLVM=1 olddefconfig
make -C out/linux-of-alias O="$PWD/out/linux-of-alias-x86-no-of-build" ARCH=x86_64 LLVM=1 -j2 rust/kernel.o

python3 out/linux-of-alias-validation/tools/testing/selftests/rust/of_alias_compile.py \
    --of-build out/linux-of-alias-riscv-build \
    --no-of-build out/linux-of-alias-x86-no-of-build \
    --output out/linux-of-alias-checks/api-probes

uv run pytest
```

Results on 2026-10-28 with Clang 19.1.7, rustc 1.98.1 and bindgen 0.73.2:

- RISC-V, `CONFIG_RUST=y`, `CONFIG_OF=y`: Rust kernel library and C OF
  implementation compiled successfully. The public helper accepts both
  `&Device` and `&Device<Bound>` and returns `Result<u32>`.
- x86-64, `CONFIG_RUST=y`, `CONFIG_OF=n`: Rust kernel library compiled
  successfully. `of::DeviceId` remains usable. Calling `of::alias_id()`
  produces the expected missing-function error because it is gated on OF.
- `uv run pytest`: 118 passed.
- `b4 prep --check`: 1 success, 0 warnings, 0 errors. Strict checkpatch,
  rustfmt and `git diff --check` also passed for the upstream patch.

Both kernel library builds reported the existing unused `Flags::from_raw`
warning in `rust/kernel/io/resource.rs`.

The compile probes use the real kernel metadata and saved Kbuild compiler
arguments. They do not boot a kernel or exercise runtime alias lookup.
Source review confirms that a missing device node returns `ENODEV`, and
negative C results reach `Error::from_errno()` before conversion to `u32`.
The C lookup takes `of_mutex`; the Rust documentation states that it may sleep.

Build logs, public-API probe output, b4 metadata and the one-patch export are
under `out/linux-of-alias-checks/` in the imager workspace.
