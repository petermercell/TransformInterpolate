# TransformInterpolate

A transform interpolation plugin for Foundry's Nuke. Dissolves between concatenated transforms of two inputs while properly concatenating downstream and producing correct motion blur.

**Original Author:** [Hendrik Proosa](https://gitlab.com/hendrikproosa/transformdissolve)  
**Compiled by:** [Peter Mercell](https://petermercell.com)

## Overview

TransformInterpolate takes two inputs (B and A) and linearly interpolates between their concatenated transforms based on a factor value. This allows for smooth transitions between different transform states while maintaining proper motion blur.

### Key Features

- **Linear transform interpolation** between two input transform chains
- **Working motion blur** — including blur from animating just the factor value when both input transforms are static
- **Mixed input support** — handles transforming from CornerPin to Transform and vice versa
- **Multiple transforms** — works with chained transforms in a row
- **Identity fallback** — if an input isn't derived from a Transform op, its matrix defaults to identity (no effect)

## Installation

### Using Pre-compiled Binaries

1. Download the appropriate binary from the `COMPILED` folder for your Nuke version and platform
2. Copy the `.so` (Linux), `.dylib` (macOS), or `.dll` (Windows) file to your Nuke plugin path
3. Add to your `init.py`:
   ```python
   nuke.pluginAddPath('/path/to/TransformInterpolate')
   ```
4. Add to your `menu.py`:
   ```python
   nuke.menu('Nodes').addCommand('Transform/TransformInterpolate', 'nuke.createNode("TransformInterpolate")')
   ```

### Building from Source

Build instructions are provided in `building_step_by_step.txt`. Platform-specific CMake files are included:

- `CMakeLists_LINUX.txt` — Linux build configuration
- `CMakeLists_MAC.txt` — macOS build configuration  
- `CMakeLists_WIN.txt` — Windows build configuration

#### Requirements

- CMake 3.10+
- Nuke NDK (matching your target Nuke version)
- C++11 compatible compiler

## Compatibility

| Platform | Status |
|----------|--------|
| Linux 64-bit | ✅ Supported |
| Windows 64-bit | ✅ Supported |
| macOS | ✅ Supported |

**Note:** Nuke Indie does not support native NDK plugins due to licensing restrictions.

## Known Limitations

- Interpolation may produce unexpected results when the difference between input transforms is very large
- Best results are achieved when interpolating between transforms with similar characteristics

## Credits

- **Original Plugin Development:** [Hendrik Proosa](https://gitlab.com/hendrikproosa)
- **Compilation & Distribution:** [Peter Mercell](https://petermercell.com)

## License

Please refer to the original author's licensing terms. This repository provides compiled binaries and build configurations for convenience.

## Links

- [Original Source (GitLab)](https://gitlab.com/hendrikproosa/transformdissolve)
- [Nukepedia Page](http://www.nukepedia.com/plugins/transform/transforminterpolate)
