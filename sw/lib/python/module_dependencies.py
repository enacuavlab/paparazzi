"""Resolve module XML dependencies (see module_dependencies.md).

SPDX-License-Identifier: GPL-2.0-or-later
"""

from dataclasses import dataclass
from pathlib import Path
import os
import xml.etree.ElementTree as ET

from pprz_env import PAPARAZZI_HOME


class DependencyError(ValueError):
    """An invalid declaration or unsatisfiable module selection."""


def comma_list(text):
    return tuple(s.strip() for s in (text or '').split(',') if s.strip())


@dataclass(frozen=True)
class Expression:
    alternatives: tuple
    negated: bool = False

    @classmethod
    def parse(cls, text):
        text = text.strip()
        negated = text.startswith('!')
        if negated:
            text = text[1:]
        alternatives = tuple(s.strip() for s in text.split('|'))
        if any(not s or any(c in s for c in '!&()') for s in alternatives):
            raise DependencyError('Invalid dependency expression: ' + text)
        return cls(alternatives, negated)

    def evaluate(self, available):
        present = any(name in available for name in self.alternatives)
        return not present if self.negated else present

    def __str__(self):
        return ('!' if self.negated else '') + '|'.join(self.alternatives)


@dataclass(frozen=True)
class Module:
    name: str
    filename: Path
    requires: tuple
    recommends: tuple
    conflicts: tuple
    provides: tuple
    suggests: tuple
    makefiles: tuple

    @classmethod
    def from_file(cls, filename):
        filename = Path(filename)
        root = ET.parse(filename).getroot()
        dep = root.find('dep')

        def values(tag):
            return comma_list(dep.findtext(tag)) if dep is not None else ()

        return cls(root.attrib['name'], filename,
                   tuple(Expression.parse(x) for x in values('depends')),
                   tuple(Expression.parse(x) for x in values('recommends')),
                   values('conflicts'), tuple('@' + x for x in values('provides')),
                   values('suggests'),
                   tuple((m.get('target'), m.get('firmware'))
                         for m in root.findall('makefile')))

    def compatible(self, target, firmware):
        return any((targets is None or Expression.parse(targets).evaluate({target}))
                   and (fw is None or fw == firmware or firmware == 'none')
                   for targets, fw in self.makefiles)


@dataclass
class Resolution:
    modules: list  # Dependency order, as (load type U/D/S, Module) pairs.
    unloaded: list


class ModuleResolver:
    """Resolve explicit module names for one target and firmware.

    XML loading is cached; each resolve call starts a fresh dependency graph.
    This does not parse aircraft/airframe files or generate makefile settings.
    """

    def __init__(self, paths=None):
        if paths is None:
            paths = []
            for base in os.getenv('PAPARAZZI_MODULES_PATH', '').split(os.pathsep):
                if base:
                    paths.extend(sorted(p for p in (Path(base) / 'modules').iterdir()
                                        if p.is_dir()))
            paths.append(Path(PAPARAZZI_HOME) / 'conf/modules')
        self.paths = tuple(Path(p) for p in paths)
        self._cache = {}

    def load(self, name):
        name = os.fspath(name)
        if name not in self._cache:
            path = Path(name if name.endswith('.xml') else name + '.xml')
            candidates = [path] if path.is_absolute() or name.startswith('.') else [
                base / path for base in self.paths]
            filename = next((p for p in candidates if p.is_file()), None)
            if filename is None:
                raise DependencyError('Module not found: ' + name)
            self._cache[name] = Module.from_file(filename)
        return self._cache[name]

    def find_providers(self, functionality):
        """Return all modules declaring functionality, with or without '@'.

        Search configured paths recursively, in path priority order, without
        target/firmware filtering or dependency resolution. Earlier paths
        override later files with the same relative filename.
        """
        functionality = '@' + functionality.strip().lstrip('@')
        providers, seen = [], set()
        for base in self.paths:
            for filename in sorted(base.rglob('*.xml')):
                relative = filename.relative_to(base)
                if relative in seen:
                    continue
                seen.add(relative)
                module = self.load(filename.resolve())
                if functionality in module.provides:
                    providers.append(module)
        return providers

    def resolve(self, names, target, firmware='none', board=None,
                include_implicit=True):
        """Return ordered modules; raise DependencyError on invalid selections.

        Optional target/firmware/board modules are included when present.
        Unlike the OCaml discovery mode, requirements are always validated.
        """
        roots = [self.load(name) for name in names]
        if include_implicit:
            for kind, name in (('targets', target), ('firmwares', firmware), ('boards', board)):
                if name:
                    filename = next((p / kind / (name + '.xml') for p in self.paths
                                     if (p / kind / (name + '.xml')).is_file()), None)
                    if filename:
                        roots.append(self.load(filename))
        selected = {}
        while True:
            loaded, unloaded, visiting, requirements, suggestions = {}, {}, set(), [], {}

            def visit(module, load_type):
                name = module.name
                if name in visiting:
                    raise DependencyError('Cyclic dependency: ' + name)
                if name in loaded or name in unloaded:
                    return
                visiting.add(name)
                for suggestion in module.suggests:
                    candidate = self.load(suggestion)
                    suggestions[candidate.name] = candidate
                if not module.compatible(target, firmware):
                    unloaded[name] = module
                else:
                    for expr in module.requires:
                        if not expr.negated and len(expr.alternatives) == 1 and not expr.alternatives[0].startswith('@'):
                            visit(self.load(expr.alternatives[0]), 'D')
                        requirements.append((module, expr))
                    loaded[name] = (load_type, module)
                visiting.remove(name)

            for module in roots:
                visit(module, 'U')
            for module in selected.values():
                visit(module, 'S')
            available = set(loaded)
            available.update(p for _, m in loaded.values() for p in m.provides)
            for _, module in loaded.values():
                for conflict in module.conflicts:
                    if conflict in available:
                        raise DependencyError(f'{module.name} conflicts with {conflict}')
            additions = {}
            missing = []
            for module, expr in requirements:
                if expr.evaluate(available):
                    continue
                if expr.negated:
                    raise DependencyError(f'{module.name}: forbidden dependency present ({expr})')
                candidates = [m for m in suggestions.values()
                              if m.name not in selected and m.name not in loaded
                              and m.compatible(target, firmware)
                              and expr.evaluate(set(m.provides))]
                if not candidates:
                    missing.append(f'{module.name}: unsatisfied dependency {expr}')
                additions.update((m.name, m) for m in candidates)
            if not additions:
                if missing:
                    raise DependencyError(missing[0])
                break
            selected.update(additions)

        # Recommendations order existing modules; they never load new ones.
        edges = {}
        for name, (_, module) in loaded.items():
            deps = set()
            for expr in module.requires + module.recommends:
                if not expr.negated:
                    for term in expr.alternatives:
                        if term in loaded:
                            deps.add(term)
                        deps.update(n for n, (_, m) in loaded.items() if term in m.provides)
            edges[name] = deps
        ordered, done = [], set()
        while len(done) < len(loaded):
            ready = [n for n in loaded if n not in done and edges[n] <= done]
            if not ready:
                raise DependencyError('Un-orderable modules: ' + ', '.join(n for n in loaded if n not in done))
            for name in ready:
                ordered.append(loaded[name])
                done.add(name)
        return Resolution(ordered, list(unloaded.values()))
