# Module dependencies

Declare dependencies inside a module's `<dep>` element:

```xml
<dep>
  <depends>ahrs_common,@imu,@gps|@mag</depends>
  <provides>ahrs</provides>
  <suggests>gps_ubx,mag_hmc58xx</suggests>
  <recommends>@navigation</recommends>
  <conflicts>ahrs_float_dcm</conflicts>
</dep>
```

- A plain name identifies a module XML file; `@name` identifies a functionality.
  `<provides>ahrs</provides>` declares `@ahrs` (omit `@` in `provides`).
- Commas mean **all requirements** must hold. A single positive module name
  loads that module recursively; a functionality requires a loaded provider.
- Pipes mean **any alternative** suffices. `@gps|@mag` accepts either provider;
  `i2c|spi_master` accepts either loaded bus module. Alternatives do not
  automatically load every named module.
- A leading `!` means **none of these names may be present**.
  `!@gps|@mag` means neither GPS nor magnetometer functionality may be present.
  Negation applies to the whole pipe list. `&`, parentheses, and embedded `!`
  are unsupported. Whitespace around names is ignored by the Python parser.
- `suggests` lists candidate modules whose provided functionalities can satisfy
  missing positive requirements. Compatible candidates are added recursively;
  all matching candidates may be selected. Suggestions cannot repair negation.
- `recommends` orders already loaded modules before this module; it does not
  load them or require their presence.
- `conflicts` rejects named modules or functionalities (use `@` for the latter)
  present in the final selection. Prefer this for incompatibilities.

For example, `ins_alt_float` requires `@imu,@gps`: both must be available.
`ahrs_float_cmpl_quat` requires `ahrs_common,@imu,@gps|@mag`: the common module,
IMU functionality, and at least one of GPS or magnetometer functionality.

Modules load only when a `<makefile>` matches the target and firmware. Target
filters use the same pipe/leading-negation syntax against one target name.
Dependencies and recommendations determine final order; cycles raise errors.

## Python API

```python
from module_dependencies import ModuleResolver

result = ModuleResolver().resolve(["baro_bmp280", "i2c"], target="ap",
                                  include_implicit=False)
for load_type, module in result.modules:
    print(load_type, module.name)  # U: explicit, D: dependency, S: suggested

providers = ModuleResolver().find_providers("gps")  # "@gps" also accepted
for module in providers:
    print(module.name, module.filename)
```

`find_providers` scans configured search paths recursively and returns all
modules declaring the functionality, without target/firmware filtering or
dependency resolution. Earlier paths override later files with the same
relative filename. An unknown functionality returns an empty list.

The default search path includes external module directories from
`PAPARAZZI_MODULES_PATH`, then `PAPARAZZI_HOME/conf/modules`. Pass `paths=[...]`
for an explicit search path. With `include_implicit=True` (default), existing
`targets/<target>`, `firmwares/<firmware>`, and `boards/<board>` modules are roots.
`result.unloaded` records incompatible modules; invalid selections raise
`DependencyError`.

This library resolves module selections; it does not parse airframes, merge
makefile settings, or replace the existing OCaml generators. It always validates
requirements. Negation uses absence from the complete available-name set,
correcting the OCaml resolver's single-name evaluation. Existing module files
currently contain no negated dependencies.
