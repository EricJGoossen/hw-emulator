"""The example and the built-in presets validate against schemas/."""

import glob
import json
import os
import unittest

ROOT = os.path.join(os.path.dirname(__file__), "..", "..")

try:
    import jsonschema
    import yaml
    from referencing import Registry, Resource
except ImportError:  # pragma: no cover
    jsonschema = None


@unittest.skipIf(jsonschema is None, "needs jsonschema and pyyaml")
class Schemas(unittest.TestCase):
    def setUp(self):
        self.schemas = {}
        for path in glob.glob(os.path.join(ROOT, "schemas", "*.schema.json")):
            with open(path) as f:
                self.schemas[os.path.basename(path)] = json.load(f)
        self.registry = Registry().with_resources(
            [(s["$id"], Resource.from_contents(s)) for s in self.schemas.values()])

    def check(self, schema, path):
        validator = jsonschema.Draft202012Validator(self.schemas[schema], registry=self.registry)
        with open(path) as f:
            errors = [e.message for e in validator.iter_errors(yaml.safe_load(f))]
        self.assertEqual(errors, [], path)

    def test_the_example(self):
        example = os.path.join(ROOT, "examples", "turret_testbed")
        self.check("sim.schema.json", os.path.join(example, "sim.yaml"))
        self.check("devices.schema.json", os.path.join(example, "devices.yaml"))

    def test_the_presets(self):
        for path in glob.glob(os.path.join(ROOT, "presets", "*.yaml")):
            self.check("presets.schema.json", path)

    def test_an_unknown_device_param_is_caught(self):
        validator = jsonschema.Draft202012Validator(self.schemas["devices.schema.json"], registry=self.registry)
        bad = {"devices": [{"name": "m", "type": "dji_can_motor", "transport": {"bus": "can0", "id": 0x201},
                            "bind": {"motor": "m", "encoder": "e"}, "params": {"comand_scale": 1}}]}
        self.assertTrue(list(validator.iter_errors(bad)))


if __name__ == "__main__":
    unittest.main()
