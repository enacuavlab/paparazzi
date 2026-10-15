"""Tests for the standalone Python module dependency resolver."""
import sys
from pathlib import Path
import tempfile
import unittest

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / 'sw/lib/python'))
from module_dependencies import DependencyError, Expression, Module, ModuleResolver


class DependencyTests(unittest.TestCase):
    def setUp(self):
        self.directory = tempfile.TemporaryDirectory()
        self.addCleanup(self.directory.cleanup)
        self.path = Path(self.directory.name)
        self.resolver = ModuleResolver([self.path])

    def module(self, name, dep='', target=None):
        target_attr = f' target="{target}"' if target else ''
        (self.path / (name + '.xml')).write_text(
            f'<module name="{name}"><dep>{dep}</dep><makefile{target_attr}/></module>')

    def resolve(self, *names):
        return self.resolver.resolve(names, 'ap', include_implicit=False)

    def test_find_providers(self):
        self.module('gps_a', '<provides>gps,position</provides>')
        self.module('gps_b', '<provides>gps</provides>', target='sim')
        self.module('other', '<provides>imu</provides>')
        nested = self.path / 'nested'
        nested.mkdir()
        (nested / 'gps_c.xml').write_text(
            '<module name="gps_c"><dep><provides>gps</provides></dep></module>')
        providers = self.resolver.find_providers('gps')
        self.assertEqual([m.name for m in providers], ['gps_a', 'gps_b', 'gps_c'])
        self.assertEqual(self.resolver.find_providers('@gps'), providers)
        self.assertEqual(self.resolver.find_providers('absent'), [])
        # A file in an earlier search directory overrides the same relative file.
        override = self.path / 'override'
        override.mkdir()
        (override / 'gps_a.xml').write_text(
            '<module name="replacement"><dep><provides>imu</provides></dep></module>')
        resolver = ModuleResolver([override, self.path])
        self.assertNotIn('gps_a', [m.name for m in resolver.find_providers('gps')])

    def test_negation_uses_whole_set(self):
        expression = Expression.parse('!@gps|@mag')
        self.assertTrue(expression.evaluate(set()))
        self.assertTrue(expression.evaluate({'@imu'}))
        self.assertFalse(expression.evaluate({'@gps', '@imu'}))
        self.assertFalse(expression.evaluate({'@mag'}))

    def test_negation_cannot_select_unrelated_suggestion(self):
        self.module('consumer', '<depends>!@gps</depends><suggests>imu</suggests>')
        self.module('gps', '<provides>gps</provides>')
        self.module('imu', '<provides>imu</provides>')
        with self.assertRaisesRegex(DependencyError, 'forbidden'):
            self.resolve('consumer', 'gps')
        self.assertEqual([m.name for _, m in self.resolve('consumer').modules], ['consumer'])

    def test_suggestion_chain_and_order(self):
        self.module('consumer', '<depends>@a,@b</depends><suggests>a</suggests>')
        self.module('a', '<provides>a</provides><depends>@b</depends><suggests>b</suggests>')
        self.module('b', '<provides>b</provides>')
        result = self.resolve('consumer')
        self.assertEqual([(lt, m.name) for lt, m in result.modules],
                         [('S', 'b'), ('S', 'a'), ('U', 'consumer')])

    def test_missing_conflicts_cycles_and_compatibility(self):
        self.module('a', '<depends>b</depends>')
        self.module('b', '<depends>a</depends>')
        with self.assertRaisesRegex(DependencyError, 'Cyclic'):
            self.resolve('a')
        self.module('other', target='!ap|sim')
        result = self.resolve('other')
        self.assertEqual(result.modules, [])
        self.assertEqual([m.name for m in result.unloaded], ['other'])
        self.module('missing', '<depends>@absent</depends>')
        with self.assertRaisesRegex(DependencyError, 'unsatisfied'):
            self.resolve('missing')
        self.module('conflict', '<conflicts>@gps</conflicts>')
        self.module('gps', '<provides>gps</provides>')
        with self.assertRaisesRegex(DependencyError, 'conflicts'):
            self.resolve('conflict', 'gps')

    def test_real_modules_and_optional_recommendations(self):
        for path in (ROOT / 'conf/modules').rglob('*.xml'):
            Module.from_file(path)
        resolver = ModuleResolver([ROOT / 'conf/modules'])
        result = resolver.resolve(['baro_bmp280', 'i2c'], 'ap', include_implicit=False)
        names = [m.name for _, m in result.modules]
        self.assertLess(names.index('i2c'), names.index('baro_bmp280'))
        self.module('optional', '<recommends>absent</recommends>')
        self.assertEqual(len(self.resolve('optional').modules), 1)


if __name__ == '__main__':
    unittest.main()
