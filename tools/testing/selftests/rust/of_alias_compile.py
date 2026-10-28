#!/usr/bin/env python3
# SPDX-License-Identifier: GPL-2.0
"""Local compile checks for the OF alias API against built kernel metadata."""

import argparse
import os
from pathlib import Path
import shlex
import subprocess


ALIAS = '''#![no_std]
#![allow(missing_docs)]
pub fn alias(dev: &kernel::device::Device) -> kernel::error::Result<u32> {
    kernel::of::alias_id(dev, c"serial")
}
pub fn bound_alias(dev: &kernel::device::Device<kernel::device::Bound>)
    -> kernel::error::Result<u32>
{
    kernel::of::alias_id(dev, c"serial")
}
'''

DEVICE_ID = '''#![no_std]
#![allow(missing_docs)]
pub fn compatible() -> kernel::of::DeviceId {
    kernel::of::DeviceId::new(c"test,device")
}
'''


def check(build, output, name, source, expected_error=None):
    build = build.resolve()
    saved = (build / 'rust/.kernel.o.cmd').read_text().splitlines()[0]
    command = shlex.split(saved.split(':=', 1)[1])
    command = command[command.index('rustc'):]
    if ';' in command:
        command = command[:command.index(';')]
    command = [arg for arg in command if not arg.startswith('--emit=')]
    command[command.index('--crate-name') + 1] = name.replace('-', '_')
    original = next(arg for arg in command if arg.endswith('/rust/kernel/lib.rs'))
    probe = output / (name + '.rs')
    probe.write_text(source)
    command[command.index(original)] = str(probe)
    command += ['--extern=kernel=' + str(build / 'rust/libkernel.rmeta'),
                '--emit=metadata=' + str(output / (name + '.rmeta'))]
    result = subprocess.run(command, cwd=build, text=True, capture_output=True,
                            env=os.environ | {'RUSTC_BOOTSTRAP': '1', 'OBJTREE': str(build)})
    (output / (name + '.log')).write_text(result.stdout + result.stderr)
    if expected_error is None:
        assert result.returncode == 0, result.stderr
    else:
        assert result.returncode != 0 and expected_error in result.stderr, result.stderr
    print(name + ': PASS')


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--of-build', required=True, type=Path)
    parser.add_argument('--no-of-build', required=True, type=Path)
    parser.add_argument('--output', required=True, type=Path)
    args = parser.parse_args()
    output = args.output.resolve()
    output.mkdir(parents=True, exist_ok=True)
    for build, enabled in [(args.of_build, True), (args.no_of_build, False)]:
        config = (build / '.config').read_text().splitlines()
        assert 'CONFIG_RUST=y' in config
        assert ('CONFIG_OF=y' in config) == enabled
    check(args.of_build, output, 'of-enabled', ALIAS)
    check(args.no_of_build, output, 'of-disabled-device-id', DEVICE_ID)
    check(args.no_of_build, output, 'of-disabled-alias-unavailable', ALIAS,
          'cannot find function `alias_id` in module `kernel::of`')


if __name__ == '__main__':
    main()
