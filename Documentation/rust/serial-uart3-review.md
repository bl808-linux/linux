# UART3-only console review

Reviewed the twelve-patch console series based on iomem-fix b7ac81a9d691,
and the local Serial KUnit additions, on 2026-10-28. Applied the saved
Sashiko multi-stage, callstack, pointer-guard, locking, false-positive and
severity prompts from the imager workspace. This was a manual code review,
not a run of the Sashiko service. Structured stage notes and build/check
logs are in `out/linux-console-checks/uart3-only/` in that workspace.

## Result

No concrete correctness regression was established in the reviewed paths.
This is a source review plus build/host-test result, not a claim of complete
coverage or new hardware validation.

## Paths and evidence checked

- Intent and interface: only UART3 is described. Its driver has one line,
  registers index 0, and uses ttyBFLB0. Generic serial abstractions still
  support multiple lines; tests deliberately use a two-line fake driver.
- OF: no change to rust/kernel/of.rs relative to the iomem-fix base.
  of_find_node_opts_by_path accepts absolute paths; of_console_check matches
  the resulting node and passes the supplied line to add_preferred_console.
  The board's direct path therefore does not require an alias lookup.
- Registration: register_port bounds-checks before allocation/publication.
  PortData owns its hardware and a Driver Arc. serial_core_register_port
  cleans up port/controller devices on registration failure. Duplicate lines
  are rejected before state->uart_port is changed.
- Devres failure: devres::register first allocates its KBox; allocation failure
  drops the incoming Registration. register_foreign uses
  devm_add_action_or_reset, whose error path invokes the registered callback.
  Both paths therefore reach uart_remove_one_port after successful publication.
- Removal: serial_core_remove_one_port unregisters the console, hangs up TTY
  users, drains state->refcount and clears state->uart_port before returning.
  unregister_console_locked removes the console with hlist_del_init_rcu and
  calls synchronize_srcu before the Rust allocation can be freed.
- Reference budget: the devres Registration retains PortData; PortData retains
  the Driver Arc and parent ARef. A console reader pins callback execution
  with console_srcu. Teardown removes visibility and drains that reader before
  dropping the hardware and Driver reference. No unpinned use-after-free path
  was identified in this ordering.
- Locking: uart_change_line_settings, uart_set_options and uart_resume_port
  call set_termios without holding the UART spinlock. The Rust adapter takes
  uart_port_lock_irqsave before invoking Hardware::set_termios. tx_empty does
  not acquire this lock, including when called by a caller already holding it.
- Emergency output: uart_port_console_lock uses a trylock during an oops.
  Failure proceeds without a LockedPort, and PortGuard only unlocks when
  acquisition succeeded. Racy emergency MMIO is the documented best-effort
  behavior; it is not a guarantee of serialized hardware writes.
- Pointer guards: console_port checks driver, state and index before indexing;
  private_data has the hardware type fixed by Driver<T>. Compile probes reject
  borrowed hardware and escaped device/lock references.
- Hardware/arithmetic: the existing UART3 address, direct PLIC source 20,
  fixed 40 MHz baud clock and polling register sequence are retained. Probe
  rejects zero clock; baud/divisor calculations retain their bounds and use
  NonZeroU32 for serial-core timeout updates. No IRQ, RX or DMA support is added.
- Validation: examined all six KUnit cases, including duplicate registration,
  failed data initialization, invalid lines, callback lock contexts and SRCU
  removal workers. The SRCU worker waits after returning from the hardware
  callback, so the completion wait is not inside the UART spinlock.

## Rejected candidates and limits

The proposed callback UAF is contradicted by the C SRCU drain and state-ref
wait above. The proposed recursive termios spinlock is contradicted by the
three actual C call sites. The proposed leaked registration on devres failure
is contradicted by KBox ownership and devm_add_action_or_reset cleanup. These
conclusions use implementation bodies rather than relying on comments.

Polling-only TTY startup returns EOPNOTSUPP by design. UART2 and MCU forwarding
are outside this series. Runtime KUnit execution and a fresh native/stdout
board boot were not performed in this review; compiling KUnit is not recorded
as a runtime test pass. Historical board runs do not validate this new source.

Per-patch checkpatch retains new-file MAINTAINERS reminders (the final patch
adds the entries) and an existing short SoC Kconfig-help warning. Kernel builds
retain the base-tree unused Flags::from_raw warning. These are separate from
the correctness findings above.

## SerialOps refactor and board validation (2026-10-28)

The follow-up refactor keeps the twelve-patch RFC, folds changes into the
abstraction/consumer introductions, and retains the RFC prefix in b4.
`serial::SerialOps` follows `pwm::PwmOps` naming. Only `tx_empty`,
`set_termios` and `console_write` are required in this polling-only series.

The typed `Driver<T>`, private `PortData<T>` and callback adapter enforce
hardware dispatch. `Port<'_>` now represents the callback context without
a redundant hardware type parameter. Private console and termios modules
separate their C contracts without adding public registration APIs.
`Port` and `LockedPort` cannot be sent or shared between threads. Owned
registration, devres rollback, UART locking and SRCU teardown remain in
place; failed emergency trylock still provides no locked token.

Compared the local PWM, serdev, PHY and platform abstractions. The extension
policy is to keep one registration owner and compose capabilities when
real users require them. A non-console UART consumer would justify an
opt-in console trait/adapter; IRQ-driven TX/RX would introduce their own
queue and lock contracts. Optional individual C hooks may use `#[vtable]`,
as in serdev and PHY. No unused capability traits or full-UART stubs are
introduced by this refactor.

Validation used the actual BL808 M1s Dock, booted from verified RAM uploads:

- Upstream RISC-V Image.gz and local RISC-V KUnit Image.gz/board DTB built.
  ARM64 also compiled; QEMU was not run.
- `uv run pytest`: 122 passed, including real-kernel API compile probes
  checking that both callback contexts reject Send and Sync.
- All six Serial KUnit cases passed on the board, including callback lock
  contexts, registration failures and console removal waiting for SRCU.
- SBI earlycon handed off to the native ttyBFLB0 UART3 console.
- Eight unbind/rebind cycles completed; all 32 racing printk records were
  retained in the ring and all 32 stable records arrived on the wire.
- Timer interrupt count advanced from 10156 to 10898. Unsupported userspace
  TTY I/O was checked. No kernel/KUnit failure was observed.
- The board was restored to UART3 U-Boot. All five existing SD file sizes
  and CRCs were checked before and after the run and matched.
- b4 exported twelve patches with RFC subjects; checkpatch reported zero
  errors and the same seven existing per-patch warnings. Reapplying the
  export reproduced the upstream tree exactly.

The board image was built from validation commit
9572355e6e7ef561baae331a29be09fd18605ff7, containing upstream commit
4d0a78753e49a0df3857c40f3e7b15682f8a8274. This subsequent documentation
commit changes no tested kernel code. The imager workspace retains artifact
hashes, source identity, raw UART logs and structured results under
`out/linux-console-checks/serialops/`. The board run covered the SBI handoff
mode; no additional native/stdout boot matrix is claimed.
