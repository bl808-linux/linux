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
